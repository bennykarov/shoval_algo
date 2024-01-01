//============================================================================
// ConsoleApplication2.cpp : Shoval_sc test app
//============================================================================

#include <iostream>
#include <thread>        
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "../BauotechAIConnectorDll/AlgoApi.h"


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




/* return cv::waitKey() */
int draw(int height, int width, char *pData, std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjects)
{
	static int wait = 10;
	int key;

	cv::Mat frameAfter = cv::Mat(height, width, CV_8UC3, pData);

	cv::Mat display;
	float scale = 0.7;
	cv::resize(frameAfter, display, cv::Size(0, 0), scale, scale);

	if (!AIObjects.empty()) {
		cv::putText(display, "P", cv::Point(20, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));
		//std::cout << "Frame " << frameNum << " : " << AIObjects[0].x << " , " << AIObjects[0].y << "\n";

	}

	cv::imshow("processed-image", display);
	key = cv::waitKey(wait);
	//if (frameNum == 0)	key = 'p';
	//if (key == 'q' || key == 27) break;
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

	cap >> frame;

	if (frame.empty()) {
		std::cout << "Can't capture " << videoName << ")\n";

		return -1;
	}


	uint32_t videoIndex = 0;	


	BAUOTECH_AND_BENNY_KAROV_ALGO algo = BAUOTECH_AND_BENNY_KAROV_ALGO::ALGO_SHOVAL;
	uint8_t* pData = NULL;
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t pixelWidth = 0; // always 32
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
	pixelWidth = frame.channels() * 8;

	
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

	while (!frame.empty()) {
		memcpy(pData, frame.data, image_size);
		//size_t sizeTemp(frame.cols * frame.rows * 3); // 24 bit
		youDraw = 1; // DDEBUG
		//BauotechAlgoConnector_Run(algo, pData, width, height, pixelWidth, image_size, youDraw, pObjects, pObjectCount);
		int videoInd = 0;
		BauotechAlgoConnector_Run3(videoInd, pData, frameNum); // tsQueue runner

		ALGO_DETECTION_OBJECT_DATA AIObjects[10];
		int objectsDetected = BauotechAlgoConnector_GetAlgoObjectData(0, 0, AIObjects);

		if (AIObjects[0].frameNum != frameNum) // DDEBUG
			std::cout << "Process Daly in : " << (frameNum - AIObjects[0].frameNum) << "frames \n";

		// DRAW : 
		// Convert list to vector
		AIObjectVec.clear();
		for (int i = 0; i < objectsDetected; i++)
			AIObjectVec.push_back(AIObjects[i]);

		if (1)
		{
			key = draw(height, width, (char*)pData, AIObjectVec);
			if (key == 'q' || key == 27)
				break;
		}
		

		cap >> frame;
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

 