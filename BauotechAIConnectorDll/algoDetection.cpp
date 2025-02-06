#include <windows.h>
#include <thread>
#include <mutex>
#include <chrono>
#include  <numeric>

#include <cuda_runtime.h>


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/videoio.hpp"
#ifdef USE_CUDA
#include <opencv2/cudaarithm.hpp>
#endif

// config file
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp> 


#include "files.hpp"
#include "AlgoApi.h"
#include "CObject.hpp"

#include "yolo/YOLO_mngr.hpp"
#include "alert.hpp"
#include "concluder.hpp"
#include "logger.hpp"


#include "utils.hpp"
#include "database.hpp"
#include "config.hpp"

/*
#include "mog.hpp"
#include "trackerBasic.hpp"
#include "MotionTrack.hpp"
#include "prediction.hpp"
*/

#include "algoDetection.hpp"

#define MAX_PERSON_DIM	cv::Size(40, 90) //  CONST
#define MAX_OBJ_DIM	cv::Size(350, 350) //  CONST	


#ifdef _DEBUG
#pragma comment(lib, "opencv_core4100d.lib")
#pragma comment(lib, "opencv_highgui4100d.lib")
#pragma comment(lib, "opencv_video4100d.lib")
#pragma comment(lib, "opencv_videoio4100d.lib")
#pragma comment(lib, "opencv_imgcodecs4100d.lib")
#pragma comment(lib, "opencv_imgproc4100d.lib")
#pragma comment(lib, "opencv_tracking4100d.lib")
#pragma comment(lib, "opencv_dnn4100d.lib")
#pragma comment(lib, "opencv_bgsegm4100d.lib")
#ifdef USE_CUDA
#pragma comment(lib, "opencv_cudabgsegm4100d.lib")
#pragma comment(lib, "cudart.lib")
//#pragma comment(lib, "cuda.lib")
#endif
#else
#pragma comment(lib, "opencv_core4100.lib")
#pragma comment(lib, "opencv_highgui4100.lib")
#pragma comment(lib, "opencv_video4100.lib")
#pragma comment(lib, "opencv_videoio4100.lib")
#pragma comment(lib, "opencv_imgcodecs4100.lib")
#pragma comment(lib, "opencv_imgproc4100.lib")
#pragma comment(lib, "opencv_tracking4100.lib")
#pragma comment(lib, "opencv_dnn4100.lib")
#pragma comment(lib, "opencv_bgsegm4100.lib")
#ifdef USE_CUDA
#pragma comment(lib, "opencv_cudabgsegm4100.lib")
#pragma comment(lib, "cudart.lib")
#endif
#endif
#pragma comment(lib, "Version.lib")




#define NO_MOTION_DETECTED       (m_params.motionDetectionType > 0 && m_BGSEGoutput.empty())

namespace  ALGO_DETECTOPN_CONSTS {
	const int MIN_CONT_AREA = 20 * 10;
	const int MAX_CONT_AREA = 1000 * 1000;
	const float goodAspectRatio = 0.5;
	const float aspectRatioTolerance = 0.2;
	const int   MIN_PIXELS_FOR_MOTION = 6 * 6; // 10 * 10;
}

/*---------------------------------------------------------------------------------------------
								U T I L S
---------------------------------------------------------------------------------------------*/

const std::vector<cv::Scalar> colors = { cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0) };

void cObject_2_pObject(CObject cObject, ALGO_DETECTION_OBJECT_DATA* pObjects)
{
	pObjects->ID = cObject.m_ID;
	pObjects->X = cObject.m_bbox.x;
	pObjects->Y = cObject.m_bbox.y;
	pObjects->Width = cObject.m_bbox.width;
	pObjects->Height = cObject.m_bbox.height;

	pObjects->ObjectType = (int)cObject.m_label;

	pObjects->DetectionPercentage = cObject.isMoving() ? int(cObject.m_confidence * 100.) : int(cObject.m_confidence * -100.); // DDEBUG set confidence Negative in case of Static object

}


bool showBGMask(cv::Mat  bgMask)
{
	cv::Mat display;

	if (bgMask.empty())
		return false;

	if (bgMask.size().width > 600)
		cv::resize(bgMask, display, cv::Size(0, 0), 0.5, 0.5);
	else
		display = bgMask;

	cv::imshow("bgMask", display);
	//cv::waitKey(1);

	return true;
}


void setConfigDefault(Config &params)
{
	params.debugLevel = 0;
	//params.showTruck = 0;
	params.modelFolder = "C:/SRC/BauoSafeZone/config_files/";
	params.motionDetectionType = 0;
	params.MLType = 10;
	params.MHistory = 100;
	params.MvarThreshold = 580.0;
	params.MlearningRate = -1;
	params.stepMotionFrames = 1;
	params.stepDetectionFrames = 2;
}


	int personsDetected(std::vector<YDetection> outputs)
	{
		int counter = 0;
		for (auto detection : outputs) {
			if (detection.class_id == Labels::person)
				counter++;
		}


		return counter;
	}

#ifdef USE_CUDA
	int _checkForGPUs()
	{

		using namespace cv::cuda;

		std::cout << "--------------------------";
		std::cout << "GPU INFO : ";
		printShortCudaDeviceInfo(getDevice());
		int cuda_devices_number = cv::cuda::getCudaEnabledDeviceCount();
		cout << "CUDA Device(s) Number: " << cuda_devices_number << endl;
		
		DeviceInfo _deviceInfo;
		bool _isd_evice_compatible = _deviceInfo.isCompatible();
		cout << "CUDA Device(s) Compatible: " << _isd_evice_compatible << endl;
		std::cout << "--------------------------";
		return cuda_devices_number;
		return 0;
	}

		
#else
	int _checkForGPUs() { return 0; }
	size_t GetGraphicDeviceVRamUsage(int _NumGPU) { return size_t(0); }
#endif 
	void debugSaveParams(int w, int h, int imgSize, int pixelWidth, float scaleDisplay, Config params)
	{
		std::ofstream debugFile("c:\\tmp\\algoapi.txt");

		debugFile << w << " , " << h << " , " << imgSize << " , " << pixelWidth << " , " << scaleDisplay << " , " << params.useGPU << " , " << params.MLType << " , "  << params.stepDetectionFrames << "\n";
		debugFile.close();

	}

	void ERROR_BEEP()
	{
		Beep(1200, 1200); // error beep
		Beep(1200, 1200); // error beep
	}



/*---------------------------------------------------------------------------------------------
 *						D E T E C T O R       C L A S S  
 *--------------------------------------------------------------------------------------------*/

void CDetector::setCamerasInfo(std::vector <CAlert> camerasInfo)
{  
	m_camerasInfo = camerasInfo;

	m_motionOnly = 0;

	m_lables_to_detect.clear();

	for (auto alert : m_camerasInfo) {
		m_lables_to_detect.push_back(alert.m_label);

		if (alert.m_motionType == 0)
			m_motionOnly = 1;
	}


}

bool CDetector::InitGPU()
{
	std::string modelName = m_params.modelFolder + "/" + YOLO_MODEL_NAME;
	if (!m_yolo.init(modelName ,true))
	{
		std::cout << "Cant init YOLO net , quit \n";
		return false;
	}

	return true;	
}


bool CDetector::init(int camIndex, int w, int h, int imgSize, int pixelWidth, int invertImage, float scaleDisplay)
{
	m_width = w;
	m_height = h;
	m_colorDepth = imgSize / (w * h);
	m_cameraID = camIndex;
	m_invertImg = invertImage;

	m_colorDepth = pixelWidth; // in Byte unit / 8;


	setConfigDefault(m_params);
	APIData apiData;
	FILE_UTILS::readConfigFile(FILES::CONFIG_FILE_NAME, m_params, apiData);
	debugSaveParams(w, h, imgSize, pixelWidth, scaleDisplay, m_params);

	if (m_params.useGPU == 0)
		m_isCuda = false;
	else
		m_isCuda = _checkForGPUs() > 0;

	// Handle ROI and active polygons:
	//-------------------------------
	for (auto& camInf : m_camerasInfo) {
		if (camInf.m_polyPoints.empty() || !camInf.checkPolygon(w, h)) {
			//CHECK_warning(false, std::string("Error , No camera.json or bad definitions" + std::string(cameraConfig)).c_str());
			camInf.m_polyPoints = { cv::Point(0,0), cv::Point(w - 1,0), cv::Point(w - 1,h - 1), cv::Point(0,h - 1) };
		}
		camInf.m_bbox = cv::boundingRect(camInf.m_polyPoints);

		m_detectionTypes.push_back((Labels)camInf.m_label);
	}


	for (int i = 0; i < apiData.camID.size();i++) {
		if (apiData.camID[i] == m_cameraID) {
			setMinPersonDim(apiData.personHeight[i]);
		}
	}

	// remove duplicated lables:
	std::sort(m_detectionTypes.begin(), m_detectionTypes.end());
	// Remove duplicate values from vector
	m_detectionTypes.erase(std::unique(m_detectionTypes.begin(), m_detectionTypes.end()), m_detectionTypes.end());


	// ROI joined all bboxes of this camera 
	m_camROI = setcamerasROI(m_camerasInfo);
	if (1)
		// Find best RIO for YOLO operation 
		m_camROI = m_yolo.optimizeRect(m_camROI, cv::Size(m_width, m_height));

	for (auto& camInf : m_camerasInfo) {
		for (auto& point : camInf.m_polyPoints)
			point -= m_camROI.tl();
	}

		
	m_decipher.clear();

	for (auto camInf : m_camerasInfo)
		m_decipher.set(camInf);
	//---------------------------------------------------------------------------------------


		// if (m_isCuda) MessageBoxA(0, "RUN WITH GPU", "Info", MB_OK);   else  MessageBoxA(0, "RUN WITOUT GPU", "Info", MB_OK);


		if (m_params.MLType > 0)
		{
			std::string modelName = m_params.modelFolder + "/" + YOLO_MODEL_NAME;

			if (!YOLO_MNGR) {
				if (!m_yolo.init(modelName, m_isCuda)) {
					LOGGER::log(DLEVEL::ERROR2, "Cant load ML model : " + m_params.modelFolder);
					return false;
				}
			}
			else  // tolo_mngr
				; // init of yolo moved to loadBalancer when YOLO is managed 

		}

		// MOG2 
		if (m_params.motionDetectionType ==  1)  {
			int emphasize = CONSTANTS::MogEmphasizeFactor;
			m_bgSeg.init(m_params.MHistory, m_params.MvarThreshold, false, emphasize, m_isCuda);
			m_bgSeg.setLearnRate(m_params.MlearningRate);
		}	

		int debugLevel = 1;
		if (m_params.trackerType > -1)
			m_tracker.init(m_params.useGPU, TRACKER::scale, debugLevel); //siam tracker 
		
		
		m_decipher.init(m_cameraID, cv::Size(m_width, m_height), m_params.debugLevel);
		m_decipher.setPersonDim(MAX_PERSON_DIM); 



		// Alloce buffer
		size_t sizeTemp(m_width * m_height * m_colorDepth);
		if (m_data == NULL)
			m_data = malloc(sizeTemp);

		return true;
		}

	/*------------------------------------------------------------
	* Process a frame : run:
	* (1) BGSEG (MOG2) motion detection 
	* (2) YOLO 
	* (3) COnsolidate both ib the Decipher class 
	* return number of tracked objects 
	 ------------------------------------------------------------*/
	int CDetector::processFrame(cv::Mat &frame_)
	{

		cv::Mat frame = frame_;

		//--------------------------------------------------------------------------------------------------------------------------
		// BG seg detection
		// Currently , motion used only for checkIfStatic() function (filter stataic objects if motion detected within its BBOX)
		//--------------------------------------------------------------------------------------------------------------------------

		m_bgMask.setTo(0);
		m_BGSEGoutput.clear();
		if (timeForMotion()) {
			// Reset BGSEG flags
			m_motionDetectet = 0;

			m_bgMask = m_bgSeg.process(frame);
			if (!m_bgMask.empty()) {
				int bgMaskNoZeroes = cv::countNonZero(m_bgMask);
				m_motionDetectet = bgMaskNoZeroes > ALGO_DETECTOPN_CONSTS::MIN_PIXELS_FOR_MOTION;
				std::vector <cv::Rect>  BGSEGoutput = detectByContours(m_bgMask);
				for (auto obj : BGSEGoutput) {
					if (obj.area() <= MAX_OBJ_DIM.width* MAX_OBJ_DIM.height)
						m_BGSEGoutput.push_back(obj);
					else 
						m_BGSEGoutputLarge.push_back(obj);
				}

				m_lastFrameNum_motion = m_frameNum;

			}
		}

		//--------------------
		// YOLO detection
		// 
		// "YOLO_MNGR" use pre-defined YOLO detectors, yolo2 is a singleton -
		// that choose one of the predefined detectors 
		//--------------------
		m_Yolotput.clear();
		if (timeForDetection()) { 
			if (YOLO_MNGR) {
				auto yolo2 = ST_yoloDetectors::getInstance();

				if (yolo2 == nullptr) 
					LOGGER::log(DLEVEL::ERROR2, std::string("Failed to get YOLO resource , cam = " + std::to_string(m_cameraID)));
				else {
					try {
						yolo2->detect(frame, m_Yolotput, m_lables_to_detect);
					}
					catch (const std::exception& err) {
						LOGGER::log(DLEVEL::ERROR2, std::string("YOLO DETECTION Exception : " + std::string(__FILE__) + " line:" + std::to_string(__LINE__)));
						LOGGER::log(DLEVEL::ERROR2, err.what());
					}


					//if (m_Yolotput.size() > 1)    LOGGER::log(DLEVEL::_SYSTEM_INFO, std::string("Objects found = " + std::to_string(m_Yolotput.size())));
				}
			}
			else
				m_yolo.detect(frame, m_Yolotput);

			m_lastFrameNum_detection = m_frameNum;

		}

		//-------------------
		// Track:
		//-------------------
		if (m_params.trackerType > -1) {
			m_TrackerObjects.clear();
			m_tracker.setNewFrame(frame); // must come before all tracker activities 
		}

		if (timeForTracking()) {
			m_tracker.track(frame, m_TrackerObjects, m_frameNum);
			m_lastFrameNum_tracking = m_frameNum;
		}

		/* update time stamp
		for (auto obj : m_TrackerObjects)
			obj.m_ts = m_timeStamp;
		*/

		std::vector <int>   TrkDuplicateInds = m_decipher.add(m_TrackerObjects, m_Yolotput, m_frameNum, m_timeStamp);


		int mode = timeForDetection() ? 1 : 0;

		/*-------------------------------------------
		 * Deciphering YOLO &  tracker objects , 
		 * Build the final detectedObject list 
		 --------------------------------------------*/
		m_decipher.consolidate(mode, m_BGSEGoutput);

		if (m_params.trackerType > -1) {
			// Tracker postprocess:
			// 1. Prune objects (Decipher decision)
			std::vector debugIDs = m_decipher.getIDStoPrune();
			if (!debugIDs.empty())
				m_tracker.prune(debugIDs);

			m_tracker.setROIs(m_decipher.getTrkObjsToRenew());
		}
		//-------------------------------------------------------------
		// Set fresh ROIs  to tracker (in case YOLO succeed to detect)
		//-------------------------------------------------------------
		if (m_Yolotput.size() > 0 && m_params.trackerType > -1) {			 
#if 1 // SIAM tracker 
			
			// 2. Add new objects, update (refrash) "bad" ROIs:
			m_tracker.setROIs(m_decipher.getBadROITrackerObject(m_frameNum));
			m_tracker.setROIs(m_decipher.getNewObjects(m_frameNum));

#else
			// missing basic-tracker-API function: m_tracker.setROIs(m_decipher.getNewObjects(m_frameNum), m_frame);
			// 'Basic' Tracker
			std::vector <cv::Rect> bboxes;
			std::vector <int> objIDs, labels;
			//basicTracker
				for (auto label : m_detectionTypes)
					for (auto obj : m_decipher.getObjects(label)) {
						bboxes.push_back(obj.m_bbox);
						objIDs.push_back(obj.m_ID);
						labels.push_back(obj.m_label);
					}

				m_tracker.clear(); // remove all ROIs
				m_tracker.setROIs(bboxes, objIDs, labels, m_frame);
#endif 
		}

		if (1) // DDEBUG SHOW BG_MASK 
		{
			if (m_params.debugLevel > 1 && !m_bgMask.empty()) {
				cv::Mat display;
				if (m_bgMask.size().width > 600)
					cv::resize(m_bgMask, display, cv::Size(0, 0), 0.5, 0.5);
				else
					display = m_bgMask;

				cv::imshow("m_bgMask", display);
				cv::waitKey(1);
			}
		}

		int tracked_count=0;
		for (auto label : m_detectionTypes)
			tracked_count += m_decipher.getObjects(label).size();
 

		return tracked_count;
	}


	/*-----------------------------------------------------------------------------------------------
	* Main Detector function
	* Return : number of objects detected (including all polygons and types)
	* 
	* 
	 -----------------------------------------------------------------------------------------------*/
	int CDetector::process(cv::Mat frameRaw, ALGO_DETECTION_OBJECT_DATA* pObjects, int frameNum, uint64_t timeStamp)
	{
		std::vector <CObject>	detectedObjs;

		pObjects->ObjectType = -1;

		if (m_invertImg == 1)
			cv::flip(frameRaw, m_frameOrg, 0);
		else
			m_frameOrg = frameRaw;

		try {
			m_frameROI = m_frameOrg(m_camROI);
		}
		catch (const std::exception& err) {
			LOGGER::log(DLEVEL::ERROR1, "Exception : " + std::string(__FILE__)+ " line:" + std::to_string(__LINE__));
			LOGGER::log(DLEVEL::ERROR1, err.what());
		}

		if (m_params.scale != 1.)
			cv::resize(m_frameROI, m_frame, cv::Size(0, 0), m_params.scale, m_params.scale); // performance issues 
		else
			m_frame = m_frameROI;

		try {
			int objects_tracked = processFrame(m_frame);
		}
		catch (const std::exception& err) {
			LOGGER::log(DLEVEL::ERROR1, "Exception : " + std::string(__FILE__) + " line:" + std::to_string(__LINE__) + err.what());
		}


		try {
			if (m_decipher.getObjects().size() > 0) {

				detectedObjs = m_decipher.getStableObjects(1., &m_frame);

				int alertObjectsNum = m_decipher.getAlertObjectsNum();

				// scale back to original image size:
				for (auto& obj : detectedObjs) {
					obj.m_bbox = UTILS::scaleBBox(obj.m_bbox, 1. / m_params.scale);
					obj.m_bbox.x += m_camROI.x;
					obj.m_bbox.y += m_camROI.y;

					//obj.m_ts = timeStamp;
				}


				// Note: limit the number of objects to const MAX_OBJECTS!
				for (int k = 0; k < detectedObjs.size() && k < MAX_OBJECTS; k++)
					cObject_2_pObject(detectedObjs[k], &pObjects[k]);

				if (1) // DDEBUG SAVE DETECTION  IMAGE 
					if (detectedObjs.size() > 0)
						debugSaveDetectionsImages(detectedObjs);

			}
		}
		catch (const std::exception& err) {
					LOGGER::log(DLEVEL::ERROR1, "Exception END OF FUNCTION : " + std::string(__FILE__) + " line:" + std::to_string(__LINE__) + err.what());
				}
		
		//m_frameNum = frameNum;
		m_frameNum++; // temp - while frameNum for server is missing !!!!!  15-9-24


		m_timeStamp = timeStamp;

		return detectedObjs.size();
	}



	std::vector <cv::Rect> CDetector::detectByContours(cv::Mat bgMask)
	{
		// Find blobs using CONTOURS 
		std::vector < std::vector<cv::Point>> contours, good_contours;
		//std::vector <cv::Rect> eyeROIs;
		std::vector<cv::Rect>    newROIs;
		std::vector<cv::Vec4i> hierarchy;

		findContours(bgMask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		// Contour analysis
		for (auto cont : contours) {
			cv::Rect box = cv::boundingRect(cont);
			int area = contourArea(cont, false);

			// Filters:
			if (area < ALGO_DETECTOPN_CONSTS::MIN_CONT_AREA || area > ALGO_DETECTOPN_CONSTS::MAX_CONT_AREA)
				continue;

			UTILS::checkBounderies(box, bgMask.size());
			newROIs.push_back(box);
		}

		return newROIs;
	}


	bool CDetector::motionDetected(cv::Mat mask)
	{
		return cv::countNonZero(mask) > ALGO_DETECTOPN_CONSTS::MIN_PIXELS_FOR_MOTION;
	}



	// Get YOLO ML interval
	bool CDetector::timeForTracking()
	{ 
		return (m_params.trackerType >= 0 &&  !timeForDetection() &&  (m_frameNum+1) % m_params.trackerStep == 0);
	}

	// Get motion (bgseg) interval 
	// Check motion only for motionOnly detection 
	bool CDetector::timeForMotion()
	{
		return (m_params.motionDetectionType > 0 && m_motionOnly && (m_frameNum - m_lastFrameNum_motion) >= m_params.stepMotionFrames);
	}

	/*-------------------------------------------------------------------------
	 * Get detection (YOLO) interval
	 * 	detect in case:
	 * Not implemented ! (1) Person had been detected in prev frame
	 * Not implemented ! (2) Motion had been detected
	 * (3) Distance from last frame larger than the intraval ('.stepDetectionFrames')
	 * (4) Const intraval cycle arrived
	-------------------------------------------------------------------------*/
	bool CDetector::timeForDetection()
	{
		if (m_params.MLType <= 0)
			return false;

		if (m_motionOnly && NO_MOTION_DETECTED)  // No motion detected while in motionOnly mode - dont YOLO
			return false;


		if ((m_frameNum - m_lastFrameNum_detection) >= m_params.stepDetectionFrames) 
			return true;

		// otherwise 
		return   false;
	}


	/*--------------------------------------------------------------------------------------
	* Save detection image with detection BOXes
	* switch:
	* m_params.debugTraceCamID -1 (default)		- dont save
	* m_params.debugTraceCamID 1000				- Save all cameras
	* m_params.debugTraceCamID range(0..999)	- save only this camera ID
	*
	 --------------------------------------------------------------------------------------*/
	int CDetector::debugSaveDetectionsImages(std::vector <CObject>	detectedObjs, std::vector <int> camIDs)
	{
		//if (!camIDs.empty() && std::find(camIDs.begin(), camIDs.end(), m_cameraID) == camIDs.end())  return 0;
		if (m_params.debugTraceCamID < 0)
			return 0;

		if (m_params.debugTraceCamID != 1000 && m_cameraID != m_params.debugTraceCamID)
			return 0;

		std::string fname = "c:\\tmp\\debug\\detection_" + std::to_string(m_cameraID) + "_" + std::to_string(m_frameNum) + ".png";
		cv::Mat display = m_frame.clone();

		for (auto obj : detectedObjs) 
			cv::rectangle(display, obj.m_bbox, cv::Scalar(255, 0, 100), 3);

		if (display.size().width > 800) {
			float scale = 800. / (float)display.size().width;
			cv::resize(display, display, cv::Size(0, 0), scale, scale);
		}

		cv::imwrite(fname, display);
		
		return detectedObjs.size();

	}
