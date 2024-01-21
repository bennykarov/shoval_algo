#include <windows.h>
#include <thread>
#include <mutex>
#include <chrono>
#include  <numeric>
//#include<tuple>

#define CUDA_ON // DDEBUG 

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/videoio.hpp"
#ifdef CUDA_ON
#include <opencv2/cudaarithm.hpp>
#endif

// config file
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp> 


#include "const.hpp"
#include "AlgoApi.h"
#include "CObject.hpp"
#include "yolo/yolo5.hpp"
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



#ifdef _DEBUG
#pragma comment(lib, "opencv_core480d.lib")
#pragma comment(lib, "opencv_highgui480d.lib")
#pragma comment(lib, "opencv_video480d.lib")
#pragma comment(lib, "opencv_videoio480d.lib")
#pragma comment(lib, "opencv_imgcodecs480d.lib")
#pragma comment(lib, "opencv_imgproc480d.lib")
#pragma comment(lib, "opencv_tracking480d.lib")
#pragma comment(lib, "opencv_dnn480d.lib")
#pragma comment(lib, "opencv_cudabgsegm480d.lib")
//#pragma comment(lib, "cuda.lib")
//#pragma comment(lib, "opencv_calib3d480d.lib")
//#pragma comment(lib, "opencv_bgsegm480d.lib")
#else
#pragma comment(lib, "opencv_core480.lib")
#pragma comment(lib, "opencv_highgui480.lib")
#pragma comment(lib, "opencv_video480.lib")
#pragma comment(lib, "opencv_videoio480.lib")
#pragma comment(lib, "opencv_imgcodecs480.lib")
#pragma comment(lib, "opencv_imgproc480.lib")
#pragma comment(lib, "opencv_tracking480.lib")
#pragma comment(lib, "opencv_dnn480.lib")
#pragma comment(lib, "opencv_cudabgsegm480.lib")
//#pragma comment(lib, "opencv_world480.lib")
#endif



/*
#ifdef _DEBUG
#pragma comment(lib, "opencv_world480d.lib")
#else
#pragma comment(lib, "opencv_world480.lib")
#endif
*/

//int readCameraJson(std::string fname, std::vector <CAlert>& m_cameras);


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

#if 0
std::vector <CRoi2frame>  readROISfile(std::string fname)
{
	std::vector <CRoi2frame>  rois2frames;
	//int frameNum;
	ifstream roisFile(fname);

	if (!roisFile.is_open())
		return std::vector <CRoi2frame>();

	CRoi2frame newBBox2f;
	int index;
	while (!roisFile.eof()) {
		roisFile >> index >> newBBox2f.frameNum >> newBBox2f.bbox.x >> newBBox2f.bbox.y >> newBBox2f.bbox.width >> newBBox2f.bbox.height;
		if (index > 0 && !roisFile.eof())
			rois2frames.push_back(newBBox2f);
	}

	return rois2frames;
}
#endif 

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
	params.skipDetectionFrames2 = 3;
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
	conf.skipDetectionFrames2 = pt.get<int>("ALGO.stepDetection2", conf.skipDetectionFrames2);
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


	int depth2cvType(int depth)
	{
		switch (depth) {
		case 1:
			return  CV_8UC1;
			break;
		case 2:
			return  CV_8UC2;
			break;
		case 3:
			return  CV_8UC3;
			break;
		case 4:
			return  CV_8UC4;
			break;
		}
	}


	void cObject_2_pObject(CObject cObject, ALGO_DETECTION_OBJECT_DATA *pObjects)
	{
		pObjects->X = cObject.m_bbox.x;
		pObjects->Y = cObject.m_bbox.y;
		pObjects->Width = cObject.m_bbox.width;
		pObjects->Height = cObject.m_bbox.height;

		pObjects->ObjectType = cObject.m_label;
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
#ifdef CUDA_ON

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

		debugFile << w << " , " << h << " , " << imgSize << " , " << pixelWidth << " , " << scaleDisplay << " , " << params.useGPU << " , " << params.MLType << " , " << params.skipDetectionFrames << " , " << params.skipDetectionFrames2 << "\n";
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
		return false;
	}
	return true;
	
}


bool CDetector::init(int w, int h, int imgSize , int pixelWidth, float scaleDisplay)
	{
		m_width = w;
		m_height = h;
		m_colorDepth = imgSize / (w*h);

		if (1)
			m_colorDepth = pixelWidth; // in Byte unit / 8;

			
		setConfigDefault(m_params);
		readConfigFile("C:\\Program Files\\Bauotech\\config.ini", m_params);
		debugSaveParams(w, h, imgSize, pixelWidth, scaleDisplay, m_params);


		if (m_params.useGPU == 0) 
			m_isCuda = false;
		else 
			m_isCuda = checkForGPUs() > 0;	 

		// DDEBUG note: multi camera - shold locate on upper level (multi cams level) -----------
		int camCount = readCamerasJson(CAMERAS_FILE_NAME, m_cameras); 

		if (camCount > 0)
			m_decipher.set(m_cameras[0].m_polyPoints, m_cameras[0].m_label, m_cameras[0].m_maxAllowed); // (std::vector<cv::Point > polyPoints, int label, int max_allowed)
		//---------------------------------------------------------------------------------------


		// if (m_isCuda) MessageBoxA(0, "RUN WITH GPU", "Info", MB_OK);   else  MessageBoxA(0, "RUN WITOUT GPU", "Info", MB_OK);

		if (m_params.camROI[2] > 0 && m_params.camROI[3] > 0 && m_params.camROI[0] + m_params.camROI[2] < w && m_params.camROI[1] + m_params.camROI[3] < h) {
			m_camROI = cv::Rect(m_params.camROI[0], m_params.camROI[1], m_params.camROI[2], m_params.camROI[3]);
		}
		else
			m_camROI = cv::Rect(0, 0, w, h);


		if (m_params.MLType > 0)
		{
			if (!m_yolo.init(m_params.modelFolder, m_isCuda)) {
				std::cout << "Cant init YOLO5 net , quit \n";

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
		//Sleep(1000);

		return true;
		}


	int CDetector::processFrame(cv::Mat &frame_)
	{
		// Reset process flags
		m_bgMask.setTo(0);
		m_BGSEGoutput.clear();
		m_motionDetectet = 0;

#if 0  
		//moved to upper level 
		cv::Mat frame;
		if (frame_.channels() == 4) {
			cv::cvtColor(frame_, frame, cv::COLOR_BGRA2BGR);
		}
		else if (frame_.channels() == 2) {
			cv::cvtColor(frame_, frame, cv::COLOR_YUV2BGR);
		}
		else
			frame = frame_;
#endif 
		cv::Mat frame = frame_;


		if (0)
		{
			imshow("DLL benny", frame);
			cv::waitKey(1);
		}


		// BG seg detection
		//--------------------
		if (timeForMotion()) {
			m_bgMask = m_bgSeg.process(frame);
			if (!m_bgMask.empty()) {
				m_motionDetectet = cv::countNonZero(m_bgMask) > ALGO_DETECTOPN_CONSTS::MIN_PIXELS_FOR_MOTION;
				std::vector <cv::Rect>  BGSEGoutput = detectByContours(m_bgMask);
				for (auto obj : BGSEGoutput) {
					if (obj.area() <= MAX_PERSON_DIM.width*MAX_PERSON_DIM.height)
						m_BGSEGoutput.push_back(obj);
					else 
						m_BGSEGoutputLarge.push_back(obj);
				}

			}
		}
		else
			int debug = 10;

		// YOLO detection
		//--------------------
		if (timeForDetection()) {
			m_Youtput.clear();

			m_yolo.detect(frame, m_Youtput);
			if (m_params.debugLevel > 2 && personsDetected(m_Youtput))
				Beep(900, 10);// DDEBUG 			
		}

		m_decipher.add(m_BGSEGoutput, m_Youtput, m_frameNum); // add & match

		m_decipher.track(); // consolidate detected objects 


		if (m_params.debugLevel > 1 &&  !m_bgMask.empty())
			cv::imshow("m_bgMask", m_bgMask);

		int tracked_count;
		tracked_count = m_BGSEGoutput.size();
		return tracked_count;
	}




	int CDetector::process(void* dataOrg, ALGO_DETECTION_OBJECT_DATA* pObjects)
	{
		std::vector <CObject>	sirenObjs;


		//pObjects->frameNum = m_frameNum;
		pObjects->ObjectType = -1;

		m_data = dataOrg; // No buffering - use original buffer for processing 

		// Convert image PTR to opencv MAT
		cv::Mat frameRaw = cv::Mat(m_height, m_width, depth2cvType(m_colorDepth), m_data);
		if (frameRaw.empty()) {
			std::cout << "read() got an EMPTY frame\n";
			ERROR_BEEP();
			return -1;
		}

		//Convert to operational format = BGR
		//------------------------------------
		if (frameRaw.channels() == 4) {
			cv::cvtColor(frameRaw, m_frameOrg, cv::COLOR_BGRA2BGR);
		}
		else if (frameRaw.channels() == 2) {
			// COLOR_YUV2BGR_Y422  COLOR_YUV2BGR_UYNV  COLOR_YUV2BGR_UYVY COLOR_YUV2BGR_YUY2 COLOR_YUV2BGR_YUYV COLOR_YUV2BGR_YVYU 
			//cv::cvtColor(frameRaw, m_frameOrg, cv::COLOR_YUV2BGR_Y422);
			cv::Mat mYUV(m_height + m_height / 2, m_width, CV_8UC1, (void*)m_data);
			//cv::Mat mRGB(m_height, m_width, CV_8UC3);
			m_frameOrg = cv::Mat(m_height, m_width, CV_8UC3);
			cvtColor(mYUV, m_frameOrg, cv::COLOR_YUV2BGR_YV12, 3);
			int debug = 10;
		}
		else
			m_frameOrg = frameRaw; // ??

#if 1
		if (0) // TEST YUV FORMATS:
		{
			std::vector <int> Convert_gray = { cv::COLOR_YUV2GRAY_YV12,cv::COLOR_YUV2GRAY_IYUV, cv::COLOR_YUV2GRAY_NV21, cv::COLOR_YUV2GRAY_NV12 , \
				 cv::COLOR_YUV2GRAY_I420, cv::COLOR_YUV2GRAY_IYUV, cv::COLOR_YUV420sp2GRAY,  cv::COLOR_YUV420p2GRAY  };

			cv::Mat mYUV(m_height + m_height / 2, m_width, CV_8UC1, (void*)m_data);
			//cv::Mat mRGB(m_height, m_width, CV_8UC3);
			cv::Mat mGray(m_height, m_width, CV_8UC1);
			for (auto conv : Convert_gray) {
				cvtColor(mYUV, mGray, conv, 1);
				int debug = 10;
			}



			std::vector <int> Convert_color = { cv::COLOR_YUV2BGR_NV12,  cv::COLOR_YUV2BGR_NV21, cv::COLOR_YUV2BGR_YV12,  cv::COLOR_YUV2BGR_IYUV };

			cv::Mat mBGR(m_height, m_width, CV_8UC3);
			for (auto conv : Convert_color) {
				cvtColor(mYUV, mBGR, conv, 3);
				int debug = 10;
			}
		}

#endif 


		m_frameROI = m_frameOrg(m_camROI);

		cv::resize(m_frameROI, m_frame, cv::Size(0, 0), m_params.scale, m_params.scale); // performance issues 

		int objects_tracked = 0;

		objects_tracked = processFrame(m_frame);

		/*
		pObjects->reserved1_personsCount = m_decipher.getPersonObjects(m_frameNum).size();
		pObjects->reserved2_motion = objects_tracked; //  m_motionDetectet ? 1 : 0;
		*/

		//if (m_decipher.getPersonObjects(m_frameNum).size() > 0) {
		if (m_decipher.getObjects(m_frameNum).size() > 0) {
			sirenObjs = m_decipher.getSirenObjects(1. / m_params.scale);

			if (sirenObjs.size() > 0) {
				cObject_2_pObject(sirenObjs[0], pObjects);
			}

			//pObjects->reserved1_personsCount = sirenObjs.size();
			//pObjects->reserved1 = sirenObjs.size();
		}


		if (doDrawing) {
			// Draw overlays :
			//draw(m_frameOrg, m_Youtput , 1.0 / m_params.scale);
			if (m_params.showMotion)
				draw(m_frameOrg, m_BGSEGoutput, 1.0 / m_params.scale);

			draw(m_frameOrg, 1.0 / m_params.scale);
			drawInfo(m_frameOrg);
		}

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


	/*---------------------------------------------------------------------------------------------
	 *			DRAW FUNCTIONS 
	 ---------------------------------------------------------------------------------------------*/

	void CDetector::draw(cv::Mat &img, float scale)
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
		if (!m_cameras.empty())
			drawPolygon(img, m_cameras[0].m_polyPoints, 1.); // DDEBUG draw camera[0]


		if (0)
			cv::putText(m_frameOrg, std::to_string(m_frameNum) , cv::Point(20, img.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 1., cv::Scalar(0, 255, 0));


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

		// Person had been detected in prev frame:
		if (m_decipher.numberOfPersonsOnBoard() > 0)
			if (m_frameNum % m_params.skipDetectionFrames == 0)
				return true;
		// Motion had been detected: 
		if (m_BGSEGoutput.size() > 0) {
			if (m_frameNum % m_params.skipDetectionFrames == 0)
					return true;
		}
		// Intraval cycle arrived:
		else if (m_frameNum %  m_params.detectionInterval == 0)
				return true;

		// otherwise 
		return   false;

	}

	int CDetector::getDetectionCount()
	{
		return m_decipher.getPersonObjects(m_frameNum).size(); // not optimized !!!
	}



#if 0
	class CYUV_Converter {
	public:

		int  main__(int argc, char** argv)
		{
			int width;
			int height;
			FILE* fin = NULL;
			struct YUV_Capture cap;
			enum YUV_ReturnValue ret;
			IplImage* bgr;
			if (argc != 4)
			{
				fprintf(stderr, "usage: %s file.yuv width height\n", argv[0]);
				return 1;
			}
			width = atoi(argv[2]);
			height = atoi(argv[3]);
			if (width <= 0 || height <= 0)
			{
				fprintf(stderr, "error: bad frame dimensions: %d x %d\n", width, height);
				return 1;
			}
			fin = fopen(argv[1], "rb");
			if (!fin)
			{
				fprintf(stderr, "error: unable to open file: %s\n", argv[1]);
				return 1;
			}
			ret = YUV_init(fin, width, height, &cap);
			assert(ret == YUV_OK);

			bgr = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
			assert(bgr);

			for (; ;)
			{
				ret = YUV_read(&cap);
				if (ret == YUV_EOF)
				{
					cvWaitKey(0);
					break;
				}
				else if (ret == YUV_IO_ERROR)
				{
					fprintf(stderr, "I/O error\n");
					break;
				}
				cvCvtColor(cap.ycrcb, bgr, CV_YCrCb2BGR);
				cvShowImage(argv[1], bgr);
				cvWaitKey(35);
			}

			return 0;
		}


	};
#endif 