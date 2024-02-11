#include <windows.h>
#include <thread>
#include <mutex>
#include <chrono>
#include  <numeric>


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


#include "const.hpp"
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

	pObjects->ObjectType = cObject.m_label;
}


void setConfigDefault(Config &params)
{
	params.debugLevel = 0;
	params.showTruck = 0;
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
	conf.showTruck = pt.get<int>("GENERAL.showTruck", conf.showTruck);
	conf.showMotion = pt.get<int>("GENERAL.showMotion", conf.showMotion);
	conf.camROI = to_array<int>(pt.get<std::string>("GENERAL.camROI", "0,0,0,0"));
	// [OPTIMIZE]  Optimization
	conf.skipMotionFrames = pt.get<int>("ALGO.stepMotion", conf.skipMotionFrames);
	conf.skipDetectionFrames = pt.get<int>("ALGO.stepDetection", conf.skipDetectionFrames);
	conf.skipDetectionFramesInMotion = pt.get<int>("ALGO.stepDetectionInMotion", conf.skipDetectionFramesInMotion);
	//---------
	// ALGO:
	//---------
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

	int checkForGPUs()
	{
#ifdef USE_CUDA

		using namespace cv::cuda;

		std::cout << "--------------------------";
		std::cout << "GPU INFO : ";
		printShortCudaDeviceInfo(getDevice());
		int cuda_devices_number = getCudaEnabledDeviceCount();
		cout << "CUDA Device(s) Number: " << cuda_devices_number << endl;
		
		DeviceInfo _deviceInfo;
		bool _isd_evice_compatible = _deviceInfo.isCompatible();
		cout << "CUDA Device(s) Compatible: " << _isd_evice_compatible << endl;
		std::cout << "--------------------------";
		return cuda_devices_number;
#else
		return 0;
#endif 
	}



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
bool CDetector::InitGPU()
{

	if (!m_yolo.init(m_params.modelFolder,true)) 
	{
		std::cout << "Cant init YOLO5 net , quit \n";
		//std::cout << "Cant init YOLO8 net , quit \n";
		return false;
	}
	return true;
	
}


bool CDetector::init(int camIndex, int w, int h, int imgSize , int pixelWidth, char* cameraConfig, float scaleDisplay)
{
		m_width = w;
		m_height = h;
		m_colorDepth = imgSize / (w*h);
		m_cameraIndex = camIndex;

		m_colorDepth = pixelWidth; // in Byte unit / 8;

			
		setConfigDefault(m_params);
		readConfigFile(CONSTANTS::CONFIG_FILE_NAME, m_params);
		debugSaveParams(w, h, imgSize, pixelWidth, scaleDisplay, m_params);

		if (m_params.useGPU == 0) 
			m_isCuda = false;
		else 
			m_isCuda = checkForGPUs() > 0;	 

		// Handle ROI and active polygons:
		//-------------------------------
		int camCount = readCamerasJson(cameraConfig, m_camerasInfo , m_cameraIndex);
		CHECK_exception(camCount > 0, std::string("Error , No camera.json was found in " + std::string(cameraConfig)).c_str());

		if (OLD_ROI) //* OLD ROI setting from config file 
		{
			if (m_params.camROI[2] > 0 && m_params.camROI[3] > 0 && m_params.camROI[0] + m_params.camROI[2] < w && m_params.camROI[1] + m_params.camROI[3] < h) {
				m_camROI = cv::Rect(m_params.camROI[0], m_params.camROI[1], m_params.camROI[2], m_params.camROI[3]);
			}
			else
				m_camROI = cv::Rect(0, 0, w, h);
		}
		else // NEW camROI using polygon points:
		{
			// Set ROI according to Polygon + Fix points to ROI
			if (m_camerasInfo[0].m_polyPoints.empty())
				m_camROI = cv::Rect(0, 0, w, h);
			else {
				m_camROI = cv::boundingRect(m_camerasInfo[0].m_polyPoints);  // DDEBUG missing Cam ID
				for (auto& point : m_camerasInfo[0].m_polyPoints)
					point -= m_camROI.tl();
			}
		}

		for (auto camInf : m_camerasInfo) 
				m_decipher.set(camInf.m_polyPoints, camInf.m_label, camInf.m_maxAllowed); // (std::vector<cv::Point > polyPoints, int label, int max_allowed)
			//---------------------------------------------------------------------------------------


		// if (m_isCuda) MessageBoxA(0, "RUN WITH GPU", "Info", MB_OK);   else  MessageBoxA(0, "RUN WITOUT GPU", "Info", MB_OK);


		if (m_params.MLType > 0)
		{
			if (!m_yolo.init(m_params.modelFolder, m_isCuda)) {
				std::cout << "Cant init YOLO net , quit \n";

				return false;
			}
		}

		// MOG2 
		if (m_params.motionType > 0)
		{
			int emphasize = CONSTANTS::MogEmphasizeFactor;
			m_bgSeg.init(m_params.MHistory, m_params.MvarThreshold, false, emphasize, m_isCuda);
			m_bgSeg.setLearnRate(m_params.MlearningRate);
		}	

		if (1) {
			m_decipher.init(m_params.debugLevel);
			m_decipher.setPersonDim(MAX_PERSON_DIM); // DDEBUG CONST
		}


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

		// BG seg detection
		//--------------------
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

		// YOLO detection
		//--------------------
		m_Youtput.clear();
		if (timeForDetection()) {
			m_yolo.detect(frame, m_Youtput);
		}
		
		//if (m_Youtput.size() > 0)   std::cout << "Yolo detect objects = " << m_Youtput.size() << "\n"; // DDEBUG 

		m_decipher.add(m_BGSEGoutput, m_Youtput, m_frameNum); // add & match

		m_decipher.track(); // consolidate detected objects 


		if (m_params.debugLevel > 1 &&  !m_bgMask.empty())
			cv::imshow("m_bgMask", m_bgMask);

		int tracked_count;
		tracked_count = m_decipher.getObjects(-1, Labels::car).size();
		//tracked_count = m_BGSEGoutput.size();
		return tracked_count;
	}



	int CDetector::process(cv::Mat frameRaw, ALGO_DETECTION_OBJECT_DATA* pObjects)
	{
		std::vector <CObject>	sirenObjs;


		//pObjects->frameNum = m_frameNum;
		pObjects->ObjectType = -1;

		m_frameOrg = frameRaw; // ?? 

		m_frameROI = m_frameOrg(m_camROI);

		cv::resize(m_frameROI, m_frame, cv::Size(0, 0), m_params.scale, m_params.scale); // performance issues 

		int objects_tracked = 0;

		objects_tracked = processFrame(m_frame);

		if (objects_tracked > 1)
			int debug = 10;

		if (m_decipher.getObjects(m_frameNum).size() > 0) {
			if (OLD_ROI) // DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG 
				sirenObjs = m_decipher.getSirenObjects(1. / m_params.scale);
			else {
				sirenObjs = m_decipher.getSirenObjects(1.);
				//sirenObjs = m_decipher.getObjects(m_frameNum);

				// scale back to original image size:
				for (auto& obj : sirenObjs) {
					obj.m_bbox = scaleBBox(obj.m_bbox, 1. / m_params.scale);
					obj.m_bbox.x += m_camROI.x;
					obj.m_bbox.y += m_camROI.y;
				}
			}
			

			for (int k = 0; k < sirenObjs.size(); k++)
				cObject_2_pObject(sirenObjs[k], &pObjects[k]);
		}

#if 0
		if (doDrawing) {
			// Draw overlays :
			//draw(m_frameOrg, m_Youtput , 1.0 / m_params.scale);
			if (m_params.showMotion)
				draw(m_frameOrg, m_BGSEGoutput, 1.0 / m_params.scale);

			draw(m_frameOrg, sirenObjs, 1.0 / m_params.scale);
			drawInfo(m_frameOrg);
		}
#endif 
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

		int objects_tracked = 0;

		objects_tracked = processFrame(m_frame);

		if (objects_tracked > 1)
			int debug = 10;

		if (m_decipher.getObjects(m_frameNum).size() > 0) {
			sirenObjs = m_decipher.getSirenObjects(1. / m_params.scale);


			//if (sirenObjs.size() > 0) 
			for (int k=0; k < sirenObjs.size();k++)
				cObject_2_pObject(sirenObjs[k], &pObjects[k]);
		}

#if 0
		if (doDrawing) {
			// Draw overlays :
			//draw(m_frameOrg, m_Youtput , 1.0 / m_params.scale);
			if (m_params.showMotion)
				draw(m_frameOrg, m_BGSEGoutput, 1.0 / m_params.scale);

			draw(m_frameOrg, sirenObjs, 1.0 / m_params.scale);
			drawInfo(m_frameOrg);
		}
#endif 
		m_frameNum++;

		if (sirenObjs.size() > 1)
			int debug = 10;

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

		//m_objects.clear(); // DDEBUG 

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


	// Get motion (bgseg) interval 
	bool CDetector::timeForMotion()
	{
		return (m_params.motionType > 0 && m_frameNum % m_params.skipMotionFrames == 0);
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

		// Motion had been detected: 
		if (m_BGSEGoutput.size() > 0) {
			if (m_frameNum % m_params.skipDetectionFramesInMotion == 0)
				return true;
		}
		// Intraval cycle arrived:
		else if (m_frameNum % m_params.skipDetectionFrames == 0)
			return true;
#if 0
		// Person had been detected in prev frame:
		if (m_decipher.numberOfPersonsOnBoard() > 0)
			if (m_frameNum % m_params.skipDetectionFrames == 0)
				return true;
#endif 

		// otherwise 
		return   false;

	}


#if 0
	/*---------------------------------------------------------------------------------------------
	 *			DRAW FUNCTIONS 
	 ---------------------------------------------------------------------------------------------*/
	void CDetector::draw(cv::Mat& img, std::vector <CObject> detections, float scale)
	{
		cv::Scalar  color(0, 0, 0);

		for (auto obj : detections) {
			auto box = scaleBBox(obj.m_bbox, scale);
			box += cv::Point(m_camROI.x, m_camROI.y);
			auto classId = obj.m_finalLabel;
			//--------------------------------------------------------------
			// Colors: 
			//        RED for person, 
			//        BLUE for moving YOLO object (other than Person), 
			//		  White for motion tracking 
			//--------------------------------------------------------------
			//if (obj.m_finalLabel == Labels::person)
			color = cv::Scalar(0, 0, 255); // colors[classId % colors.size()];

			cv::rectangle(img, box, color, 3);
			// Add lablel
			if (obj.m_finalLabel != Labels::nonLabled) {
				cv::rectangle(img, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
				cv::putText(img, m_yolo.getClassStr(classId).c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
			}
		}

	}
	void CDetector::draw_old(cv::Mat &img, float scale)
	{
		cv::Scalar  color(0, 0, 0);

		for (auto obj : m_decipher.getPersonObjects(m_frameNum)) {
			auto box = scaleBBox(obj.m_bbox, scale);
			box += cv::Point(m_camROI.x, m_camROI.y);
			auto classId = obj.m_finalLabel;
			//--------------------------------------------------------------
			// Colors: 
			//        RED for person, 
			//        BLUE for moving YOLO object (other than Person), 
			//		  White for motion tracking 
			//--------------------------------------------------------------
			// Person objects
			if (obj.m_finalLabel == Labels::person)
				color = cv::Scalar(0, 0, 255); // colors[classId % colors.size()];
			// BGSeg stable objects
			else if (obj.m_finalLabel == Labels::nonLabled) // motion
				color = cv::Scalar(200, 200, 200);
			else
				continue;

			cv::rectangle(img, box, color, 3);
			// Add lablel
			if (obj.m_finalLabel != Labels::nonLabled) {
				cv::rectangle(img, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
				cv::putText(img, m_yolo.getClassStr(classId).c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
			}
		}


		// Draw other Labeled objects (only moving objects)
		if (m_params.showTruck > 0) {
			bool showOnlyWhileMoving = m_params.showTruck == 1;
			for (auto obj : m_decipher.getVehicleObjects(m_frameNum, showOnlyWhileMoving)) {
				auto box = scaleBBox(obj.m_bbox, scale);
				box += cv::Point(m_camROI.x, m_camROI.y);
				auto classId = obj.m_finalLabel;

				if (classId == Labels::train || classId == Labels::bus) // DDEBUg
					classId = Labels::truck;

				// Draw vehicles and othe 
				if (classId == Labels::truck) {
					color = cv::Scalar(155, 155, 0);
					cv::Point debug = centerOf(box);
					cv::putText(img, "T", centerOf(box), cv::FONT_HERSHEY_SIMPLEX, 2.5, color, 10);

				}
				else {
					color = cv::Scalar(255, 0, 0);
					cv::rectangle(img, box, color, 3);
					// Add lablel
					cv::rectangle(img, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
					cv::putText(img, m_yolo.getClassStr(classId).c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
				}
			}
		}
	}

	void CDetector::draw(cv::Mat &img, std::vector<YDetection> Youtput, float scale)
	{
		for (int i = 0; i < Youtput.size(); ++i) {
			auto detection = Youtput[i];
			if (detection.class_id == Labels::person || detection.class_id == Labels::car || 
				detection.class_id == Labels::truck || detection.class_id == Labels::bus)
			{
				auto box = scaleBBox(detection.box, scale);
				box += cv::Point(m_camROI.x, m_camROI.y);
				auto classId = detection.class_id;
				const auto color = colors[classId % colors.size()];
				cv::rectangle(img, box, color, 3);

				cv::rectangle(img, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
				cv::putText(img, m_yolo.getClassStr(classId).c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
			}
		}
	}

	// Draw ROI
	void CDetector::draw(cv::Mat &img, std::vector<cv::Rect>  rois, float scale)
	{
		cv::Scalar color(0, 255, 0);

		for (auto roi : rois) {
			cv::Rect fixROI = scaleBBox(roi, scale) + cv::Point(m_camROI.x, m_camROI.y);
			cv::rectangle(img, fixROI, color, 2);
		}

	}

	// Draw ROI polygon 
	void CDetector::drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale)
	{
		if (contour.empty())
			return;

		cv::Scalar color(0, 255, 0);
		//if (cv::iscorrect(contour)) {
		drawContours(img, std::vector<std::vector<cv::Point> >(1, contour), -1, color, 1, 8);

	}


	void CDetector::drawInfo(cv::Mat &img)
	{
		cv::Scalar color(255, 0, 0);
		// Draw ROI
		cv::rectangle(img, m_camROI, color, 2);

		// Draw alert-polygon 
		if (!m_camerasInfo.empty())
			drawPolygon(img, m_camerasInfo[0].m_polyPoints, 1.); // DDEBUG draw camera[0]


		if (0)
			cv::putText(m_frameOrg, std::to_string(m_frameNum) , cv::Point(20, img.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 255, 0));


	}
#endif 

