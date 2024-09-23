#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 
#include "opencv2/opencv.hpp"


#include "files.hpp"
#include "logger.hpp"
#include "AlgoApi.h"
#include "config.hpp"
#include "AlgoDetection.hpp"
#include "loadBalancer.hpp"

#include "trackerBasic.hpp" // DDEBUG test

// Includes for DEBUG DISPLAY 
//#include "../ConsoleApplication2/utils.hpp"
#include "../ConsoleApplication2/draw.hpp"

#include "algoProcess.hpp"

//typedef std::chrono::duration<float, std::milli> duration;

float 	g_elapsedSum = 0;
float   g_elapsedMin = 99999;
float   g_elapsedMax = 0;


/*-----------------------------------------------------------------------------------------------
 * Vehicle gather :  "car", "motorbike","aeroplane","bus","truck"
 -----------------------------------------------------------------------------------------------*/
void CAlgoProcess::makeVehicleInfo(std::vector<cv::Point> contour, int MaxAllowed, int motionType, int polygonId, int camID)
{

	int label = getYoloClassIndex("car");
	m_camerasInfo.push_back(CAlert(contour, label, MaxAllowed, motionType, polygonId, camID));
	label = getYoloClassIndex("motorbike");
	m_camerasInfo.push_back(CAlert(contour, label, MaxAllowed, motionType, polygonId, camID));
	label = getYoloClassIndex("bus");
	m_camerasInfo.push_back(CAlert(contour, label, MaxAllowed, motionType, polygonId, camID));
	label = getYoloClassIndex("truck");
	m_camerasInfo.push_back(CAlert(contour, label, MaxAllowed, motionType, polygonId, camID));
	/*label = getYoloClassIndex("train");
	m_camerasInfo.push_back(CAlert(contour, label, MaxAllowed, motionType, polygonId, camID));
	*/
}


CAlgoProcess::~CAlgoProcess()
{
	exit(0); // avoid exception 
	m_active = false;
	//LOGGER::close();
}

/*===========================================================================================
* CAlgoProcess (thread) class
  ===========================================================================================*/
	bool CAlgoProcess::init(int video_index, int width, int height, int image_size, int pixelWidth,  int invertImage)
	{
		//gAICllbacks[videoIndex](videoIndex, aidata, 1, nullptr, 0);
		m_videoIndex = video_index;
		m_width = width;
		m_height = height;
		m_pixelWidth = pixelWidth;
		m_scale = 0.5;


		int badImageSize = imageSize();

		if (m_camerasInfo.empty()) {
			std::cout << "\n\n Error: No camera polygon was found, Ignore this camera !!\n";
			return false;
		}

		if (m_camerasInfo.size() == 1 && m_camerasInfo.back().m_bbox.area() < 10 * 10) {
			// Trick: In case ploygon very small - set ROI to entire frame size!
			m_camerasInfo.back().m_polyPoints.clear();
			m_camerasInfo.back().m_polyPoints.push_back(cv::Point(0, 0));
			m_camerasInfo.back().m_polyPoints.push_back(cv::Point(width-1, 0));
			m_camerasInfo.back().m_polyPoints.push_back(cv::Point(width-1, height-1));
			m_camerasInfo.back().m_polyPoints.push_back(cv::Point(0, height-1));
			m_camerasInfo.back().m_bbox = cv::boundingRect(m_camerasInfo.back().m_polyPoints);

			std::cout << "\n\n Warning: Polygon of camera " << video_index << "found too small , system ignorings this camera!!\n";
		}

		m_tracker.setCamerasInfo(m_camerasInfo);
		bool ok = m_tracker.init(video_index, m_width, m_height, image_size, m_pixelWidth, invertImage, m_scale/*0.5*/);

		m_timer.start();


#if 0
	// TRACKER TEST 		
	CTracker _tracker;
	//_tracker.track_main(R"(C:\Data\office\doubles.ts)", 0);
	_tracker.track_main(R"(D:\BIOMetrics\rec_A022_564_A.avi)", 6);
	
#endif 

		return ok;
	}

	void CAlgoProcess::setCallback(CameraAICallback callback)
	{
		m_callback = callback;
	}

	void CAlgoProcess::WakeUp()
	{
		m_event.Set();
	}

	int CAlgoProcess::imageSize()
	{
		return m_width * m_height * m_pixelWidth; // / 8;
	}




	std::vector <int> CAlgoProcess::addMultiPolygons(std::string DetectionTypeList)
	{
		std::vector <int> labels;
		for (auto& ch : DetectionTypeList)
		{
			if (ch == ',')
				ch = ' ';
		}

		// use std::istringstream to parse the new string
		std::istringstream strm(DetectionTypeList);
		std::string aLabel;
		while (strm >> aLabel)
			labels.push_back(getYoloClassIndex(aLabel));

		return labels;

	}
	/*----------------------------------------------------------------------------------------------------------------------------------
	* API - add polygon before call to polygonInit()
	----------------------------------------------------------------------------------------------------------------------------------*/
	void CAlgoProcess::addPolygon(int CamID, int polygonId, char* DetectionType, int motionType, int MaxAllowed, int Polygon[], int polygonSize)
	{
		std::vector<cv::Point> contour;
		std::vector <int> labels;
		std::string DetectionTypeStr(DetectionType);

		if (DetectionTypeStr.find(',') != std::string::npos)
			labels = addMultiPolygons(DetectionTypeStr);
		else
			labels.push_back(getYoloClassIndex(DetectionType));

		//int label = getYoloClassIndex(DetectionType);

		for (auto label : labels) {
			contour.clear();

			if (label < 0)
				LOGGER::log(DLEVEL::ERROR2, *DetectionType + "Bad DetectionType in addPolygon");


			for (int i = 0; i < polygonSize; i += 2)
				contour.push_back(cv::Point(Polygon[i], Polygon[i + 1]));

			if (label == VEHICLE_CLASS_ID)
				makeVehicleInfo(contour, MaxAllowed, motionType, polygonId, CamID); // push multiple "vehicle" classes 
			else 
				m_camerasInfo.push_back(CAlert(contour, label, motionType, MaxAllowed, polygonId, CamID));
		}


	}

	void CAlgoProcess::polygonClear()
	{
		m_camerasInfo.clear();
	}

	void CAlgoProcess::initPolygons()
	{
		m_tracker.setCamerasInfo(m_camerasInfo);
	}





	/*---------------------------------------------------------------------
	* Start algo thread 
	* input: (1) buffer queue  (2) load balancer class 
	----------------------------------------------------------------------*/
	int CAlgoProcess::run(TSBuffQueue* bufQ, CLoadBalaner *loader)
	{
		m_terminate = false;
		m_active = true;

		m_loader = loader;

		try {

			loader->cameraCounter(1);
			m_thread = std::thread(&CAlgoProcess::run_th2, this, bufQ, loader);
			return m_thread.joinable();
		}
		catch (const std::exception& err) {
			LOGGER::g_logMsg2 << "Main process thread [CAlgoProcess::run_th2()] Crashed : " << err.what();
			LOGGER::log(DLEVEL::ERROR1, LOGGER::g_logMsg2);
		}
	}


/*----------------------------------------------------------------------------------------------------
* Process thread (main loop) : Fetching frame from the Queue , process frame and send the result to...: 
	(1) to server callback
	(2) loadBalamcer thread (via load balancer 'ResQueue' )
*	Run the m_tracker.process() and fills the 'm_Objects' objects
  ---------------------------------------------------------------------------------------------------*/
	int CAlgoProcess::run_th2(TSBuffQueue* bufQ, CLoadBalaner* loader)
	{
		CframeBuffer frameBuff;
		//long frameCount = 0;
		float elapsedSum = 0;
		float elapsedMin = 999999;
		float elapsedMax = 0;


		while (m_terminate == false)
		{
			if (bufQ->front(frameBuff) == false)
			{
				Sleep(10);
				continue;
			}
			break;
		}

		while (!m_terminate)
		{
			if (bufQ->IsEmpty()) {
				m_event.Wait();
			}


			// 3 Leave out conditions:
			//--------------------------
			if (bufQ->pop(frameBuff) == false) {
				continue;
			}

			if (1)
				if (m_frameNum == frameBuff.frameNum)
				{
					LOGGER::log(DLEVEL::WARNING1, "run_th2 got dulpicate frame : m_frameNum == frameBuff.frameNum");
					//if (0) Beep(1200, 10);
					continue;
				}

			m_frameNum = frameBuff.frameNum;

			LOGGER::setFrameNum(m_frameNum);


			try {
				m_timer.sample();
				cv::Mat frameBGR = converPTR2MAT(frameBuff.ptr, m_height, m_width, m_pixelWidth);
				m_objectCount = m_tracker.process(frameBGR, m_Objects, frameBuff.frameNum); // All detected objects
				m_alertCount = m_tracker.getAlertObjectsCount(); // Alert objects - all exceeds maxAllowed 

				if (m_objectCount != m_alertCount)
					int debug = 10;
			}
			catch (const std::exception& err) {
				LOGGER::log(DLEVEL::ERROR1, "Error algProcess main process (convert & m_tracker.process():");
				LOGGER::log(DLEVEL::ERROR1, err.what());
			}


			float elapsed = m_timer.sample();

			//if (m_frameNum % 60 == 0)    std::cout << "Algo duration: " << elapsed << " milliseconds" << std::endl;

			// DDEBUG : for consoleApplication2 : getbjectData() API  
			//---------------------------------------
			bool supportGetbjectData = true; // DDEBUG
			if (supportGetbjectData) {
				std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);
				m_pObjectsAPI.clear();
				for (int i = 0; i < m_objectCount; i++)
					m_pObjectsAPI.push_back(m_Objects[i]);
			}


			int i = 0;
			for (; i < m_objectCount; i++)
				m_Objects[i].frameNum = m_frameNum;
			for (; i < MAX_OBJECTS; i++)
				m_Objects[i].frameNum = -1;

#if 0
			// DDEBUG DDEBUG DISPLAY ================================================================================================================
			if (m_youDraw) {
				float displayScale = 0.5;
				int key = draw(m_height, m_width, frameBuff.ptr, m_pObjectsAPI, std::vector <CAlert_>(), m_frameNum);
				cv::imshow("DLL draw", frameBGR);
				cv::waitKey(1);
			}
#endif 
			//=======================================================================================================================================
			// POST PROCESS AREA:
			//=======================================================================================================================================


			//-------------------------------------------------------------------------
			// -1- send the data to the server callback (only if object was detected) 
			//-------------------------------------------------------------------------
			try {
				if (m_callback != nullptr && m_objectCount > 0)
				{
					if (1) std::string logMsg = "Cam " + std::to_string(m_videoIndex) + " Detects : " + std::to_string(m_objectCount) + "objects"; // DDEBUG DDEBUG PRINT LOGGER::log(DLEVEL::INFO2, logMsg);}
					m_callback(m_videoIndex, &m_Objects[0], m_objectCount, nullptr, 0);  //gAICllbacks[m_videoIndex](m_videoIndex, m_pObjects, m_objectCount, nullptr, 0);
				}
			}
			catch (const std::exception& err) {
				LOGGER::log(DLEVEL::ERROR1, "Error m_callback():");
				LOGGER::log(DLEVEL::ERROR1, err.what());
			}


			//---------------------------------------------------------------------------------------
			// -2- send the data to the load balancer priorityTH thread (via thread safe 'ResQueue')
			//---------------------------------------------------------------------------------------
			//if (m_objectCount > 0)
			{
				CCycle info;
				info.detections = m_objectCount;
				info.alerts = m_alertCount;
				info.camID = m_videoIndex;
				info.activeCamera = 0;
				info.motion = m_objectCount > 0 ? 1 : 0;
				info.timeStamp = m_frameNum; // DDEBUG 
				info.status = OBJECT_STATUS::DONE;
				if (loader->isActive()) {
					loader->ResQueuePush(info);
					loader->ResQueueNotify(); // not used in case of 'naive' sync by counting the 'ResQueue' !!!!
				}
			}


			if (m_videoIndex == 0) // DDEBUG DDEBUG DDEBUG 
			{
				bool debug = loader->isCamInBatchList(m_videoIndex);
				debug = false;
			}

			//frameCount++;
		} // while !terminate

		return m_frameNum > 0;

	}


	int CAlgoProcess::run_sync(void *pData, int frameNum, ALGO_DETECTION_OBJECT_DATA *AIobjects)
	{
		CframeBuffer frameBuff;
		frameBuff.ptr = (char*)pData;
		frameBuff.frameNum = frameNum;

		long frameCount = 0;


		g_elapsedSum = 0;
		g_elapsedMin = 10000;
		g_elapsedMax = 0;

		m_frameNum = frameBuff.frameNum;


		m_timer.start();

		cv::Mat frameBGR = converPTR2MAT(frameBuff.ptr, m_height, m_width, m_pixelWidth);
		m_objectCount = m_tracker.process(frameBGR, m_Objects, frameBuff.frameNum);

		m_timer.sample();

		for (int i=0;i< m_objectCount;i++)
			AIobjects[i] = m_Objects[i];


		//frameCount++;

		if (frameCount % 30 == 0) {
			std::cout << " FPS = " << 1000. / (g_elapsedSum / 30.) << " ( min / max = " << (1000. / g_elapsedMin) << " , " << (1000. / g_elapsedMax) << "\n";
			g_elapsedSum = 0;
			g_elapsedMax = 0;
			g_elapsedMin = 99999;

			frameCount = 0;
		}

		return m_objectCount;

	}

	// DDEBUG DDEBUG functions:
	void CAlgoProcess::fakeCallBack(int m_videoIndex, ALGO_DETECTION_OBJECT_DATA* pObjects, int m_objectCount, void* ptr, int something)
	{
		if (m_objectCount)
			std::cout << "CAlgoProcess::fakeCallBack: detected objects = " << m_objectCount << "\n";

		for (int i = 0; i < m_objectCount; i++)
			std::cout << "			>>	type = " << pObjects[i].ObjectType << "\n";
	}

	/*---------------------------------------------------------------------------------------------------
	* API DEBUG function: getObjectData():
	* return number of objects provided 
	* index = -1 means return all valid objects (m_objectCount)
	 ---------------------------------------------------------------------------------------------------*/
	int CAlgoProcess::getObjectData(int videoIndex, int index, ALGO_DETECTION_OBJECT_DATA* pObjects, int& frameNum)
	{
		if (!m_supportGetbjectData)
			return 0;
		if (index >= (int)m_pObjectsAPI.size()|| (int)m_pObjectsAPI.size() == 0)
			return 0;

		std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);

		if (index < 0) { // return all objects
			memcpy(pObjects, &m_pObjectsAPI[0], sizeof(ALGO_DETECTION_OBJECT_DATA) * m_pObjectsAPI.size());
			frameNum = m_Objects[0].frameNum;
			return m_objectCount;
		}
		else {
			memcpy(pObjects, &m_pObjectsAPI[index], sizeof(ALGO_DETECTION_OBJECT_DATA));
			frameNum = m_Objects[0].frameNum;
			return 1;
		}

	}


	bool CAlgoProcess::terminate()
	{ 
		m_terminate = true;
		WakeUp(); // release wait() of queue
		if (m_thread.joinable())
			m_thread.join();


		polygonClear();

		return true;
	}