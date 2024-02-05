//============================================================================
// ConsoleApplication2.cpp : Shoval_sc test app
//============================================================================

#ifdef _DEBUG
#pragma comment(lib, "opencv_core470d.lib")
#pragma comment(lib, "opencv_highgui470d.lib")
#pragma comment(lib, "opencv_video470d.lib")
#pragma comment(lib, "opencv_videoio470d.lib")
#pragma comment(lib, "opencv_imgcodecs470d.lib")
#pragma comment(lib, "opencv_imgproc470d.lib")
#else
#pragma comment(lib, "opencv_core470.lib")
#pragma comment(lib, "opencv_highgui470.lib")
#pragma comment(lib, "opencv_video470.lib")
#pragma comment(lib, "opencv_videoio470.lib")
#pragma comment(lib, "opencv_imgcodecs470.lib")
#pragma comment(lib, "opencv_imgproc470.lib")
#endif



#include <iostream>
#include <thread>        
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "utils.hpp"
#include "../BauotechAIConnectorDll/AlgoApi.h"

typedef std::chrono::duration<float, std::milli> duration;


// GLOBALS:
std::vector <CAlert_> g_cameraInfos;

std::vector <cv::Scalar> g_colors = { cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255), cv::Scalar(255,255,0), cv::Scalar(255,0,255), cv::Scalar(0,255,255) , cv::Scalar(155,55,0), cv::Scalar(25,0,55), cv::Scalar(110,155,255) };
/*------------------------------------------------------------------------------------------------------
 *									U T I L S     function 
 ------------------------------------------------------------------------------------------------------*/

void drawInfo(cv::Mat& img, CAlert_ camInfo)
{
	cv::Scalar color(255, 0, 0);
	// Draw ROI

	// Draw alert-polygon 
	drawPolygon(img, camInfo.m_polyPoints, 1.); // DDEBUG draw camera[0]

}


/* return cv::waitKey() */
int draw(int height, int width, char *pData, std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjects, int framenum)
{
	static int wait = 1;
	int key;

	auto startGUI = std::chrono::system_clock::now();

	cv::Mat frameAfter = cv::Mat(height, width, CV_8UC3, pData);

	// -1- Draw ROI 	
	if (!g_cameraInfos.empty())
		drawInfo(frameAfter, g_cameraInfos[0]); // DDEBUG draw camera[0]


	for (auto obj : AIObjects) {
		int colorInd = obj.ID % g_colors.size();
		cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], 2);
		cv::putText(frameAfter, std::to_string(obj.ID), cv::Point(obj.X, obj.Y-5), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 255));
	}

	////------------------
	cv::Mat display;
	float scale = 0.7;
	cv::resize(frameAfter, display, cv::Size(0, 0), scale, scale);
	cv::putText(display, std::to_string(framenum), cv::Point(display.cols - 170, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));

	if (!AIObjects.empty()) {
		cv::putText(display, "D", cv::Point(20, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));
		//std::cout << "Frame " << frameNum << " : " << AIObjects[0].x << " , " << AIObjects[0].y << "\n";

	}

	cv::imshow("processed-image", display);
	key = cv::waitKey(wait);
	if (key == 'p')
		wait = -1;
	else
		wait = 1;

	auto endGUI = std::chrono::system_clock::now();

	if ((framenum + 5) % 30 == 0) {
		duration GuiElapsed = endGUI - startGUI;
		std::cout << "Gui duration = " << GuiElapsed.count() << "\n";
	}

	return key;
} 


int main_API(int argc, char* argv[])
{
	std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjectVec;

	float curFPS = 0;

	int frameNum = 0;
	int skipFrames = 0;
	std::string videoName;

	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " <video file name> [skip frames]\n";
		return -1;
	}

	if (argc > 1)
		videoName = argv[1];
	if (argc > 2)
		skipFrames = atoi(argv[2]);

	cv::VideoCapture cap;
	cv::Mat frame, _frame;

	if (!cap.open(videoName)) {
		std::cout << "Can't open file  " << videoName << ")\n";
		return -1;
	}


	// read first image
	cv::Rect debugROI(260, 100, 900, 400);

	cap >> _frame;
	if (0)  // DDEBUG TEST 
		frame = _frame(debugROI); 
	else 
		frame = _frame;

	if (frame.empty()) {
		std::cout << "Can't capture " << videoName << ")\n";   return -1;
	}

	// read camera info - if no ROI - use full frame
	std::string fname = "C:\\Program Files\\Bauotech\\cameras.json";

	int camID = 0; // DDEBUG 

	readCamerasJson(fname, camID, g_cameraInfos);
	for (auto camInf : g_cameraInfos)
		if (camInf.m_polyPoints.empty())
			camInf.m_polyPoints = { cv::Point(0,0), cv::Point(frame.cols,0), cv::Point(frame.cols,frame.rows), cv::Point(0,frame.rows) };


	uint32_t videoIndex = 0;


	BAUOTECH_AND_BENNY_KAROV_ALGO algo = BAUOTECH_AND_BENNY_KAROV_ALGO::ALGO_SHOVAL;
	uint8_t* pData = NULL;
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t pixelWidth = 0; // in Bytes
	uint32_t image_size = 0;
	uint8_t youDraw = 0;
	ALGO_DETECTION_OBJECT_DATA Objects[10];
	ALGO_DETECTION_OBJECT_DATA* pObjects = &(Objects[0]);
	uint32_t objectCount = 0;
	uint32_t* pObjectCount = &objectCount;
	uint32_t alertCount = 0;
	uint32_t* pAlertCount = &alertCount;


	width = frame.cols;
	height = frame.rows;
	image_size = width * height * frame.channels();
	pixelWidth = frame.channels(); //* 8; // DDEBUG : for opencv  RGB format

	// Init
	BauotechAlgoConnector_Init();
	BauotechAlgoConnector_Config(videoIndex,algo,width,height,pixelWidth,image_size,youDraw, nullptr, (char *)"C:\\Program Files\\Bauotech\\cameras.json");


	pData = (uint8_t*)malloc(image_size);
	memcpy(pData, frame.data, image_size);

	int key = 0;
	int wait = 1;

	while (frameNum < skipFrames && !frame.empty()) {
		cap >> _frame;
		if (0)  // DDEBUG TEST 
			frame = _frame(debugROI); 
		else 
			frame = _frame;


		frameNum++;
	}


	auto prev = std::chrono::system_clock::now();

	//-----------------
	// Video Main Loop
	//-----------------
	while (!frame.empty()) {
		memcpy(pData, frame.data, image_size);
		//size_t sizeTemp(frame.cols * frame.rows * 3); // 24 bit
		youDraw = 1; // DDEBUG
		//BauotechAlgoConnector_Run(algo, pData, width, height, pixelWidth, image_size, youDraw, pObjects, pObjectCount);
		int videoInd = 0;
		BauotechAlgoConnector_Run3(videoInd, pData, frameNum); // tsQueue runner

		ALGO_DETECTION_OBJECT_DATA AIObjects[10];
		int objectsDetected = BauotechAlgoConnector_GetAlgoObjectData(0, 0, AIObjects);

		//if (AIObjects[0].frameNum != frameNum)   std::cout << "Process Daly in : " << (frameNum - AIObjects[0].frameNum) << "frames \n";  // DDEBUG

		// DRAW : 
		// Convert list to vector

		if (objectsDetected > 0)
			int debug = 10;
		else if (objectsDetected > 1)
			int debug2 = 10;

		if (AIObjectVec.size() > 1)
			int debug = 10;

		AIObjectVec.clear();
		for (int i = 0; i < objectsDetected; i++)
			AIObjectVec.push_back(AIObjects[i]);


		// DRAW image & detections 
		bool DrawDetections = true;

		if (DrawDetections)
			key = draw(height, width, (char*)pData, AIObjectVec, frameNum);
		else
			key = draw(height, width, (char*)pData, std::vector <ALGO_DETECTION_OBJECT_DATA>(), frameNum);
		if (key == 'q' || key == 27)
			break;

		cap >> _frame;
		if (0)  // DDEBUG TEST 
			frame = _frame(debugROI);
		else
			frame = _frame;


		if (frame.empty()) {
			cap.set(cv::CAP_PROP_POS_FRAMES, 0); // start again from the beginning
			cap >> _frame;
			if (0)  // DDEBUG TEST 
				frame = _frame(debugROI);
			else
				frame = _frame;

		}


		frameNum++;

#if 0
		// DDEBUG : Keep 30 FPS 
		{ 
			
			auto curr = std::chrono::system_clock::now();
			typedef std::chrono::duration<float, std::milli> duration;
			duration elapsed = curr - prev;

			if (elapsed.count() < 30.) 
				Sleep(int(30. - elapsed.count())); // DDEBUG FORCE FPS

			prev = curr;
		}  

#endif 
	}  	// while (!frame.empty())

	// termination
	BauotechAlgoConnector_Release();

}

int main(int argc, char* argv[])
{
	std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjectVec;
	ALGO_DETECTION_OBJECT_DATA AIObjects[10];

	float curFPS = 0;

	int frameNum = 0;
	int skipFrames = 0;
	std::string videoName;

	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " <video file name> [skip frames]\n";
		return -1;
	}

	if (argc > 1)
		videoName = argv[1];
	if (argc > 2)
		skipFrames = atoi(argv[2]);

	cv::VideoCapture cap;
	cv::Mat frame ;

	if (!cap.open(videoName)) {
		std::cout << "Can't open file  " << videoName << ")\n";
		return -1;
	}


	// read first image
	cap >> frame;

	if (frame.empty()) {
		std::cout << "Can't capture " << videoName << ")\n";   return -1;
	}

	// read camera info - if no ROI - use full frame
	std::string fname = "C:\\Program Files\\Bauotech\\cameras.json";

	int camID = 0; // DDEBUG 

	readCamerasJson(fname, camID, g_cameraInfos);
	for (auto camInf : g_cameraInfos)
		if (camInf.m_polyPoints.empty())
			camInf.m_polyPoints = { cv::Point(0,0), cv::Point(frame.cols,0), cv::Point(frame.cols,frame.rows), cv::Point(0,frame.rows) };


	uint32_t videoIndex = 0;


	BAUOTECH_AND_BENNY_KAROV_ALGO algo = BAUOTECH_AND_BENNY_KAROV_ALGO::ALGO_SHOVAL;
	uint8_t* pData = NULL;
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t pixelWidth = 0; // in Bytes
	uint32_t image_size = 0;
	uint8_t youDraw = 0;
	ALGO_DETECTION_OBJECT_DATA Objects[10];
	ALGO_DETECTION_OBJECT_DATA* pObjects = &(Objects[0]);
	uint32_t objectCount = 0;
	uint32_t* pObjectCount = &objectCount;
	uint32_t alertCount = 0;
	uint32_t* pAlertCount = &alertCount;


	width = frame.cols;
	height = frame.rows;
	image_size = width * height * frame.channels();
	pixelWidth = frame.channels();

	// Init
	BauotechAlgoConnector_Init();
	BauotechAlgoConnector_Config_sync(videoIndex, algo, width, height, pixelWidth, image_size, youDraw, nullptr, (char*)"C:\\Program Files\\Bauotech\\cameras.json");


	pData = (uint8_t*)malloc(image_size);
	memcpy(pData, frame.data, image_size);

	int key = 0;
	int wait = 10;

	while (frameNum < skipFrames && !frame.empty()) {
		cap >> frame;
		frameNum++;
	}


	auto prev30 = std::chrono::system_clock::now();
	auto prev = prev30;
	float maxElapsed = 0;
	//-----------------
	// Video Main Loop
	//-----------------
	while (!frame.empty()) {
		memcpy(pData, frame.data, image_size);
		//size_t sizeTemp(frame.cols * frame.rows * 3); // 24 bit
		int videoInd = 0;


		int objectsDetected = BauotechAlgoConnector_Run3_sync(videoInd, pData, AIObjects, frameNum); // tsQueue runner

		// FPS check 
		auto curr = std::chrono::system_clock::now();

		duration elapsed = curr - prev;
		maxElapsed = MAX(maxElapsed, elapsed.count()); // keep max delay 
		prev = curr;

		if (frameNum % 30 == 0) {
			//auto curr = std::chrono::system_clock::now();
			duration elapsed = curr - prev30;

			std::cout << " FPS = " << 1000. / (elapsed.count() / 30.) << "\n";
			std::cout << " Max frame duration = " << maxElapsed << "\n";

			prev30 = curr;
			maxElapsed = 0;

			std::cout << objectsDetected << " Objects were detected \n";

		}


		/*
		ALGO_DETECTION_OBJECT_DATA AIObjects[10];
		int objectsDetected = BauotechAlgoConnector_GetAlgoObjectData(0, 0, AIObjects);
		*/


		//----------------------------------------
		//      DRAW RESULTS :
		//----------------------------------------
		// Convert list to vector
		AIObjectVec.clear();
		for (int i = 0; i < objectsDetected; i++)
			AIObjectVec.push_back(AIObjects[i]);


		// DRAW image & detections 
		bool DrawDetections = false;

		
		if (DrawDetections)
			key = draw(height, width, (char*)pData, AIObjectVec, frameNum);
		//key = draw(height, width, (char*)pData, std::vector <ALGO_DETECTION_OBJECT_DATA>(), frameNum);

		if (key == 'q' || key == 27)
			break;

		if (frameNum == 0)
			cv::waitKey(-1);

		cap >> frame;

		if (frame.empty()) {
			cap.set(cv::CAP_PROP_POS_FRAMES, 0); // start again from the beginning
			cap >> frame;
		}


		frameNum++;

#if 0
		{

			auto curr = std::chrono::system_clock::now();
			typedef std::chrono::duration<float, std::milli> duration;
			duration elapsed = curr - prev;

			if (elapsed.count() < 30.)
				Sleep(int(30. - elapsed.count())); // DDEBUG FORCE FPS

			prev = curr;
		}
#endif 

	}  	// while (!frame.empty())

	// termination

	BauotechAlgoConnector_Release();

}