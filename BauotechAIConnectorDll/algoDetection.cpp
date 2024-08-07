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
#include "yolo/yolo.hpp"
#include "alert.hpp"
#include "concluder.hpp"


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

#define MAX_PERSON_DIM	cv::Size(40, 90) // DDEBUG CONST
#define MAX_OBJ_DIM	cv::Size(350, 350) // DDEBUG CONST	
#define OLD_ROI false


#ifdef _DEBUG
#pragma comment(lib, "opencv_core470d.lib")
#pragma comment(lib, "opencv_highgui470d.lib")
#pragma comment(lib, "opencv_video470d.lib")
#pragma comment(lib, "opencv_videoio470d.lib")
#pragma comment(lib, "opencv_imgcodecs470d.lib")
#pragma comment(lib, "opencv_imgproc470d.lib")
#pragma comment(lib, "opencv_tracking470d.lib")
#pragma comment(lib, "opencv_dnn470d.lib")
#pragma comment(lib, "opencv_bgsegm470d.lib")
#ifdef USE_CUDA
#pragma comment(lib, "opencv_cudabgsegm470d.lib")
//#pragma comment(lib, "cuda.lib")
#endif
#else
#pragma comment(lib, "opencv_core470.lib")
#pragma comment(lib, "opencv_highgui470.lib")
#pragma comment(lib, "opencv_video470.lib")
#pragma comment(lib, "opencv_videoio470.lib")
#pragma comment(lib, "opencv_imgcodecs470.lib")
#pragma comment(lib, "opencv_imgproc470.lib")
#pragma comment(lib, "opencv_tracking470.lib")
#pragma comment(lib, "opencv_dnn470.lib")
#pragma comment(lib, "opencv_bgsegm470.lib")
#ifdef USE_CUDA
#pragma comment(lib, "opencv_cudabgsegm470.lib")
#endif
#endif



namespace  ALGO_DETECTOPN_CONSTS {
	const int MIN_CONT_AREA = 20 * 10;
	const int MAX_CONT_AREA = 1000 * 1000;
	const float goodAspectRatio = 0.5;
	const float aspectRatioTolerance = 0.2;
	const int   MIN_PIXELS_FOR_MOTION = 10*10;
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

	pObjects->DetectionPercentage = cObject.m_moving > 0 ? 0 : 9999; // DDEBUG DDEBUG mark static as 9999  under 'DetectionPercentage'

}


void setConfigDefault(Config &params)
{
	params.debugLevel = 0;
	//params.showTruck = 0;
	params.modelFolder = "C:/SRC/BauoSafeZone/config_files/";
	params.motionType = 1;
	params.MLType = 10;
	params.MHistory = 100;
	params.MvarThreshold = 580.0;
	params.MlearningRate = -1;
	params.skipMotionFrames = 1;
	params.skipDetectionFrames = 3;
	//params.skipDetectionFrames2 = 3;
}

bool readConfigFile(std::string ConfigFName, Config &conf)
{
	if (!FILE_UTILS::file_exists(ConfigFName))  {
		std::cout << "WARNING : Can't find Config.ini file, use default values \n";
		return false;
	}

	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(ConfigFName, pt);
	// [GENERAL]
	conf.videoName = pt.get<std::string>("GENERAL.video", conf.videoName);
	conf.roisName = pt.get<std::string>("GENERAL.rois", conf.roisName);
	conf.waitKeyTime = pt.get<int>("GENERAL.delay-ms", conf.waitKeyTime);
	conf.record = pt.get<int>("GENERAL.record", conf.record);
	conf.demoMode = pt.get<int>("GENERAL.demo", conf.demoMode);
	conf.debugLevel = pt.get<int>("GENERAL.debug", conf.debugLevel);
	//conf.showTruck = pt.get<int>("GENERAL.showTruck", conf.showTruck);
	conf.showMotion = pt.get<int>("GENERAL.showMotion", conf.showMotion);
	conf.camROI = to_array<int>(pt.get<std::string>("GENERAL.camROI", "0,0,0,0"));
	

		//---------
		// ALGO:
		//---------
		// [OPTIMIZE]  Optimization
	conf.skipMotionFrames = pt.get<int>("ALGO.stepMotion", conf.skipMotionFrames);
	conf.skipDetectionFrames = pt.get<int>("ALGO.stepDetection", conf.skipDetectionFrames);
	conf.skipDetectionFramesInMotion = pt.get<int>("ALGO.stepDetectionInMotion", conf.skipDetectionFramesInMotion);
	conf.skipDetectionFramesInMotion = pt.get<int>("ALGO.stepTracking", conf.skipDetectionFramesInMotion);
	std::vector <int> motionROI_vec = to_array<int>(pt.get<std::string>("ALGO.motionROI", "0,0,0,0"));
	if (motionROI_vec[2] > 0) // width
		conf.motionROI = cv::Rect(motionROI_vec[0], motionROI_vec[1], motionROI_vec[2], motionROI_vec[3]);

	conf.modelFolder = pt.get<std::string>("ALGO.modelFolder", conf.modelFolder);
	conf.scale = pt.get<float>("ALGO.scale", conf.scale);
	conf.MHistory = pt.get<int>("ALGO.MHistory", conf.MHistory);
	conf.MvarThreshold = pt.get<float>("ALGO.MvarThreshold", conf.MvarThreshold);
	conf.MlearningRate = pt.get<float>("ALGO.MlearningRate", conf.MlearningRate);
	conf.motionType = pt.get<int>("ALGO.motion", conf.motionType);
	conf.trackerType = pt.get<int>("ALGO.tracker", conf.trackerType);
	conf.trackerStep = pt.get<int>("ALGO.trackerStep", conf.trackerStep);
	conf.MLType = pt.get<int>("ALGO.ML", conf.MLType);
	conf.useGPU = pt.get<int>("ALGO.useGPU",1) ;

	/*
	conf.prediction = pt.get<int>("ALGO.predict", conf.prediction);
	conf.onlineTracker = pt.get<int>("ALGO.onlineTracker", conf.onlineTracker);
	//conf.displayScale = pt.get<float>("GENERAL.scaleDisplay", conf.displayScale);
	//conf.beep = pt.get<int>("GENERAL.beep", conf.beep);
	//conf.shadowclockDirection = pt.get<float>("GENERAL.shadow-hand", conf.shadowclockDirection);
	//conf.showTime = pt.get<int>("GENERAL.show-time", conf.showTime);
	//conf.showBoxesNum = pt.get<int>("GENERAL.show-boxes-num", conf.showBoxesNum);
	//std::vector <int> v = pt.get<std::vector <int>>("GENERAL.show-boxes-num");
	*/
	return true;
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
	int checkForGPUs()
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

	/*
	size_t GetGraphicDeviceVRamUsage(int _NumGPU)
	{
		//cudaSetDevice(_NumGPU);

		size_t l_free = 0;
		size_t l_Total = 0;
		cudaError_t error_id = cudaMemGetInfo(&l_free, &l_Total);

		return (l_Total - l_free);
	}
	*/

#else
	int checkForGPUs() { return 0; }
	size_t GetGraphicDeviceVRamUsage(int _NumGPU) { return size_t(0); }
#endif 
	void debugSaveParams(int w, int h, int imgSize, int pixelWidth, float scaleDisplay, Config params)
	{
		std::ofstream debugFile("c:\\tmp\\algoapi.txt");

		debugFile << w << " , " << h << " , " << imgSize << " , " << pixelWidth << " , " << scaleDisplay << " , " << params.useGPU << " , " << params.MLType << " , " << params.skipDetectionFramesInMotion << " , "  << params.skipDetectionFrames << "\n";
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
}

bool CDetector::InitGPU()
{

	if (!m_yolo.init(m_params.modelFolder,true)) 
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
	m_cameraIndex = camIndex;
	m_invertImg = invertImage;

	m_colorDepth = pixelWidth; // in Byte unit / 8;


	setConfigDefault(m_params);
	FILE_UTILS::readConfigFile(FILES::CONFIG_FILE_NAME, m_params);
	debugSaveParams(w, h, imgSize, pixelWidth, scaleDisplay, m_params);

	if (m_params.useGPU == 0)
		m_isCuda = false;
	else
		m_isCuda = checkForGPUs() > 0;

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

	// remove duplicated lables:
	std::sort(m_detectionTypes.begin(), m_detectionTypes.end());
	// Remove duplicate values from vector
	m_detectionTypes.erase(std::unique(m_detectionTypes.begin(), m_detectionTypes.end()), m_detectionTypes.end());


	// ROI joined all bboxes of this cfamera 
	m_camROI = setcamerasROI(m_camerasInfo);

	for (auto& camInf : m_camerasInfo) {
		for (auto& point : camInf.m_polyPoints)
			point -= m_camROI.tl();
	}

	if (0) // Fix too small ROI 
		if (m_yolo.getVersion() == 8)
		{
			if (m_camROI.width < YOLO8_INPUT_WIDTH)
				m_camROI.width = YOLO8_INPUT_WIDTH;
			if (m_camROI.height < YOLO8_INPUT_HEIGHT)
				m_camROI.height = YOLO8_INPUT_HEIGHT;
		}
		else if (m_yolo.getVersion() == 5) {
			if (m_camROI.width < YOLO5_INPUT_WIDTH)
				m_camROI.width = YOLO5_INPUT_WIDTH;
			if (m_camROI.height < YOLO5_INPUT_HEIGHT)
				m_camROI.height = YOLO5_INPUT_HEIGHT;
}

		
	m_decipher.clear();

		for (auto camInf : m_camerasInfo) 
				m_decipher.set(camInf.m_polyPoints, camInf.m_label, camInf.m_motionType, camInf.m_maxAllowed, camInf.m_ployID); // (std::vector<cv::Point > polyPoints, int label, int max_allowed)
			//---------------------------------------------------------------------------------------


		// if (m_isCuda) MessageBoxA(0, "RUN WITH GPU", "Info", MB_OK);   else  MessageBoxA(0, "RUN WITOUT GPU", "Info", MB_OK);


		if (m_params.MLType > 0)
		{
			if (0) /// DDEBUG
				m_params.modelFolder = "C:/src/YOLO/yoloONNX/";
			if (!m_yolo.init(m_params.modelFolder, m_isCuda)) {
				std::cout << "Cant init YOLO net , quit \n";

				return false;
			}
		}

		// MOG2 
		if (m_params.motionType ==  1)  {
			int emphasize = CONSTANTS::MogEmphasizeFactor;
			m_bgSeg.init(m_params.MHistory, m_params.MvarThreshold, false, emphasize, m_isCuda);
			m_bgSeg.setLearnRate(m_params.MlearningRate);
		}	

		int debugLevel = 1;
		if (m_params.trackerType > -1)
			m_tracker.init(m_params.useGPU, TRACKER::scale, debugLevel); //siam tracker 
		
		m_decipher.init(cv::Size(m_width, m_height), m_params.debugLevel);
		m_decipher.setPersonDim(MAX_PERSON_DIM); // DDEBUG CONST



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
		// Reset process flags
		m_bgMask.setTo(0);
		m_BGSEGoutput.clear();
		m_motionDetectet = 0;

		cv::Mat frame = frame_;

		/*
		cv::Mat trkFrame;
		trkFrame = m_frame;
		*/


		// BG seg detection
		//--------------------
#if 1
		if (timeForMotion()) {
			m_bgMask = m_bgSeg.process(frame);
			if (!m_bgMask.empty()) {
				m_motionDetectet = cv::countNonZero(m_bgMask) > ALGO_DETECTOPN_CONSTS::MIN_PIXELS_FOR_MOTION;
				std::vector <cv::Rect>  BGSEGoutput = detectByContours(m_bgMask);
				for (auto obj : BGSEGoutput) {
					if (obj.area() <= MAX_OBJ_DIM.width* MAX_OBJ_DIM.height)
						m_BGSEGoutput.push_back(obj);
					else 
						m_BGSEGoutputLarge.push_back(obj);
				}

			}
		}
#endif 

		// YOLO detection
		//--------------------
		m_Yolotput.clear();
		if (timeForDetection()) {
			m_yolo.detect(frame, m_Yolotput);
		}

		for (auto obj : m_Yolotput)
			if (obj.class_id == 2)
				int debug = 10;

		// Track:
		//-------------------
		if (m_params.trackerType > -1) {
			m_TrackerObjects.clear();
			m_tracker.setNewFrame(frame); // must come before all tracker activities 
		}

		if (timeForTracking()) {
			m_tracker.track(frame, m_TrackerObjects, m_frameNum);
		}

		std::vector <int>   TrkDuplicateInds = m_decipher.add(m_TrackerObjects, m_Yolotput, m_frameNum);


		int mode = timeForDetection() ? 1 : 0;
		/*-------------------------------------------
		 * Deciphering YOLO &  tracker objects , 
		 * Build the final detectedObject list 
		 --------------------------------------------*/
		m_decipher.track(mode);

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
			/* {// DDEBUG DDEBUG DDEBUG 
				auto yoloObjects = m_decipher.getObjects(m_frameNum);
				for (auto &obj : yoloObjects) {
					obj.m_bbox = resizeBBox(obj.m_bbox, 1.4);
				}
				m_tracker.setROIs(yoloObjects);
			}
			*/

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

		if (m_params.debugLevel > 1 && !m_bgMask.empty()) {
			cv::Mat display;
			cv::resize(m_bgMask, display, cv::Size(0, 0), 0.5, 0.5);
			cv::imshow("m_bgMask", display);
		}

		int tracked_count=0;
		for (auto label : m_detectionTypes)
			tracked_count += m_decipher.getObjects(label).size();
 

		return tracked_count;
	}



	int CDetector::process(cv::Mat frameRaw, ALGO_DETECTION_OBJECT_DATA* pObjects)
	{
		std::vector <CObject>	sirenObjs;

		pObjects->ObjectType = -1;

		if (m_invertImg == 1)
			cv::flip(frameRaw, m_frameOrg, 0);
		else 
			m_frameOrg = frameRaw; 


		m_frameROI = m_frameOrg(m_camROI);

		if (m_params.scale != 1.)
			cv::resize(m_frameROI, m_frame, cv::Size(0, 0), m_params.scale, m_params.scale); // performance issues 
		else
			m_frame = m_frameROI;

		int objects_tracked = processFrame(m_frame);

		if (m_decipher.getObjects().size() > 0) {

			sirenObjs = m_decipher.getSirenObjects(1., &m_frame);

			// scale back to original image size:
			for (auto& obj : sirenObjs) {
				obj.m_bbox = UTILS::scaleBBox(obj.m_bbox, 1. / m_params.scale);
				obj.m_bbox.x += m_camROI.x;
				obj.m_bbox.y += m_camROI.y;
			}


			for (int k = 0; k < sirenObjs.size(); k++)
				cObject_2_pObject(sirenObjs[k], &pObjects[k]);
		}

		m_frameNum++;


		return sirenObjs.size();
	}

	int CDetector::process(void* dataOrg, ALGO_DETECTION_OBJECT_DATA* pObjects)
	{
		std::vector <CObject>	sirenObjs;


		//pObjects->frameNum = m_frameNum;
		pObjects->ObjectType = -1;

		m_data = dataOrg; // No buffering - use original buffer for processing 

		// Convert image PTR to opencv MAT
		//------------------------------------
		m_frameOrg = converPTR2MAT(m_data, m_height, m_width, m_colorDepth);
 
		m_frameROI = m_frameOrg(m_camROI);

		cv::resize(m_frameROI, m_frame, cv::Size(0, 0), m_params.scale, m_params.scale); // performance issues 


		int objects_tracked = processFrame(m_frame);

		if (m_decipher.getObjects().size() > 0) {
			sirenObjs = m_decipher.getSirenObjects(1. / m_params.scale, &m_frame);


			//if (sirenObjs.size() > 0) 
			for (int k=0; k < sirenObjs.size();k++)
				cObject_2_pObject(sirenObjs[k], &pObjects[k]);
		}

#if 0
		if (doDrawing) {
			// Draw overlays :
			//draw(m_frameOrg, m_Yolotput , 1.0 / m_params.scale);
			if (m_params.showMotion)
				draw(m_frameOrg, m_BGSEGoutput, 1.0 / m_params.scale);

			draw(m_frameOrg, sirenObjs, 1.0 / m_params.scale);
			drawInfo(m_frameOrg);
		}
#endif 
		m_frameNum++;

		return sirenObjs.size();
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

			// FIlter outliers 
			//rbox = cv::minAreaRect(cont);
			/*
			attrib.aspectRatio = attrib.rbox.size.height / attrib.rbox.size.width;
			attrib.perimeter = cv::arcLength(cv::Mat(contours[i]), true);
			attrib.thickness = attrib.perimeter / attrib.area;
			attrib.close = (hierarchy[i][2] < 0 && hierarchy[i][3] < 0) ? -1 : 1;
			attrib.len = MAX(attrib.rbox.size.width, attrib.rbox.size.height);
			attrib.topLevel = hierarchy[i][3] == -1; // no paretn
			*/
			//cv::Rect debug = scaleBBox(box, 1. / 0.5);

			UTILS::checkBounderies(box, bgMask.size());
			newROIs.push_back(box);
		}

		return newROIs;
	}


	bool CDetector::motionDetected(cv::Mat mask)
	{
		return cv::countNonZero(mask) > ALGO_DETECTOPN_CONSTS::MIN_PIXELS_FOR_MOTION;
	}



	bool CDetector::timeForTracking()
	{ 
		return (m_params.trackerType >= 0 &&  !timeForDetection() &&  (m_frameNum+1) % m_params.trackerStep == 0);
	}

	// Get motion (bgseg) interval 
	bool CDetector::timeForMotion()
	{
		return (m_params.motionType > 0 && !timeForDetection() &&  (m_frameNum+1) % m_params.skipMotionFrames == 0);
	}

	/*-------------------------------------------------------------------------
	 * Get detection (YOLO) interval
	 * 	detect in case:
	 * (1) Person had been detected in prev frame
	 * (2) Motion had been detected
	 * (3) Const intraval cycle arrived
	-------------------------------------------------------------------------*/
	bool CDetector::timeForDetection()
	{
		if (m_params.MLType <= 0)
			return false;

		// Intraval cycle arrived:
		//if (m_cycleNum  % m_params.skipDetectionFrames == 0)
		if (m_frameNum % m_params.skipDetectionFrames == 0)
		return true;

		// otherwise 
		return   false;

#if 0
		if (m_BGSEGoutput.size() > 0) {
			// Motion had been detected: 
			if (m_frameNum % m_params.skipDetectionFramesInMotion == 0)
				return true;
	}

		// Person had been detected in prev frame:
		if (m_decipher.numberOfPersonsOnBoard() > 0)
			if (m_frameNum % m_params.skipDetectionFrames == 0)
				return true;
#endif 
	}


