//============================================================================
// ConsoleApplication2.cpp : Shoval_sc test app
//============================================================================

#ifdef _DEBUG
#pragma comment(lib, "opencv_core480d.lib")
#pragma comment(lib, "opencv_highgui480d.lib")
#pragma comment(lib, "opencv_video480d.lib")
#pragma comment(lib, "opencv_videoio480d.lib")
#pragma comment(lib, "opencv_imgcodecs480d.lib")
#pragma comment(lib, "opencv_imgproc480d.lib")
#else
#pragma comment(lib, "opencv_core480.lib")
#pragma comment(lib, "opencv_highgui480.lib")
#pragma comment(lib, "opencv_video480.lib")
#pragma comment(lib, "opencv_videoio480.lib")
#pragma comment(lib, "opencv_imgcodecs480.lib")
#pragma comment(lib, "opencv_imgproc480.lib")
#endif



#include <iostream>
#include <thread>        
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "utils.hpp"
//#include "../BauotechAIConnectorDll/alert.hpp"

#include "../BauotechAIConnectorDll/AlgoApi.h"

// GLOBALS:
std::vector <CAlert_> g_cameraInfos;


/*------------------------------------------------------------------------------------------------------
 *UTILS function 
 */

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
	static int wait = 10;
	int key;

	cv::Mat frameAfter = cv::Mat(height, width, CV_8UC3, pData);

	// -1- Draw ROI 	
	if (!g_cameraInfos.empty())
		drawInfo(frameAfter, g_cameraInfos[0]); // DDEBUG draw camera[0]


	for (auto obj : AIObjects) {
		cv::Scalar color(0, 0, 255);
		cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), color, 2);	
	}

	////------------------
	cv::Mat display;
	float scale = 0.7;
	cv::resize(frameAfter, display, cv::Size(0, 0), scale, scale);
	cv::putText(display, std::to_string(framenum), cv::Point(display.cols - 100, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));

	if (!AIObjects.empty()) {
		cv::putText(display, "D", cv::Point(20, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));
		//std::cout << "Frame " << frameNum << " : " << AIObjects[0].x << " , " << AIObjects[0].y << "\n";

	}

	cv::imshow("processed-image", display);
	key = cv::waitKey(wait);
	if (key == 'p')
		wait = -1;
	else
		wait = 10;

	return key;
} 


int main(int argc, char* argv[])
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
	cv::Mat frame;

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
	pixelWidth = frame.channels(); //* 8; // DDEBUG : for opencv  RGB format

	// Init
	BauotechAlgoConnector_Init();
	BauotechAlgoConnector_Config(videoIndex,algo,width,height,pixelWidth,image_size,youDraw, nullptr);


	pData = (uint8_t*)malloc(image_size);
	memcpy(pData, frame.data, image_size);

	int key = 0;
	int wait = 10;

	while (frameNum < skipFrames && !frame.empty()) {
		cap >> frame;

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
		//int objectsDetected = BauotechAlgoConnector_GetAlgoObjectData(0, 0, AIObjects);
		int objectsDetected = BauotechAlgoConnector_GetAlgoObjectData(0, -1, AIObjects);

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
		

		cap >> frame;

		if (frame.empty())
			int debug = 10;

		frameNum++;

		if (1) // DDEBUG : Keep 30 FPS 
		{ 
			
			auto curr = std::chrono::system_clock::now();
			typedef std::chrono::duration<float, std::milli> duration;
			duration elapsed = curr - prev;

			if (elapsed.count() < 30.) 
				Sleep(int(30. - elapsed.count())); // DDEBUG FORCE FPS

			prev = curr;
		}  

	}  	// while (!frame.empty())

	// termination

	BauotechAlgoConnector_Release();




}

 