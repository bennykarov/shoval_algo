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
#include "../BauotechAIConnectorDll/files.hpp"
#include "../BauotechAIConnectorDll/AlgoApi.h"
#include "../BauotechAIConnectorDll/config.hpp"
#include "../BauotechAIConnectorDll/timer.hpp"

//std::string cameraFname = "C:\\Program Files\\Bauotech\\AI\\cameras100.json";


// GLOBALS:
int main_camFileGen(int argc, char* argv[]);
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
	static int wait = -1;
	int key;

	cv::Mat frameAfter = cv::Mat(height, width, CV_8UC3, pData);

	// -1- Draw ROI 	
	if (!g_cameraInfos.empty())
		drawInfo(frameAfter, g_cameraInfos[0]); // DDEBUG draw camera[0]

	bool DrawColorPerID = false; // DDEBUG flag

	if (DrawColorPerID)
		for (auto obj : AIObjects) {
			int colorInd = obj.ID % g_colors.size();
			cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], 2);
			cv::putText(frameAfter, std::to_string(obj.ID), cv::Point(obj.X, obj.Y-5), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 255));
		}
	else 
		for (auto obj : AIObjects) {
			cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[0], 2);
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

	return key;
} 







#if 0
//----------------------------------------------------------------------------------------
//  MAIN using async (server) API - including QUEUE mechanism 
//----------------------------------------------------------------------------------------

int main_API(int argc, char* argv[])
{
	uint32_t videoIndex = 0;
	int videosNum = 3;
	std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjectVec;


	int frameNum = 0;
	int skipFrames = 0;
	std::string videoName;

	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " <video file name> [skip frames]\n";
		return -1;
	}

	/// read cmd arguments 
	if (argc > 1)
		videoName = argv[1];
	if (argc > 2)
		skipFrames = atoi(argv[2]);
	bool DrawDetections = true;
	if (argc > 3)
		if (toUpper(std::string(argv[3])) == "NODRAW")
			DrawDetections = false;


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


	readCamerasJson(fname, (int)videoIndex, g_cameraInfos);
	for (auto camInf : g_cameraInfos)
		if (camInf.m_polyPoints.empty())
			camInf.m_polyPoints = { cv::Point(0,0), cv::Point(frame.cols,0), cv::Point(frame.cols,frame.rows), cv::Point(0,frame.rows) };



	BAUOTECH_AND_BENNY_KAROV_ALGO algo = BAUOTECH_AND_BENNY_KAROV_ALGO::ALGO_SHOVAL;

	// Params (global)
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t pixelWidth = 0; // in Bytes
	uint32_t image_size = 0;
	uint8_t youDraw = 0;

	// Params per camera:
	uint8_t* pData[MAX_CAMERAS];
	ALGO_DETECTION_OBJECT_DATA Objects[MAX_CAMERAS][10];
	ALGO_DETECTION_OBJECT_DATA* pObjects[MAX_CAMERAS];
	uint32_t objectCount[MAX_CAMERAS];
	uint32_t* pObjectCount[MAX_CAMERAS];
	uint32_t alertCount[MAX_CAMERAS];
	uint32_t* pAlertCount[MAX_CAMERAS];


	width = frame.cols;
	height = frame.rows;
	image_size = width * height * frame.channels();
	pixelWidth = frame.channels(); //* 8; // DDEBUG : for opencv  RGB format

	for (int i = 0; i < cameras_num; i++) {
		pData[i] = (uint8_t*)malloc(image_size);
		memcpy(pData[i], frame.data, image_size);
	}

	// Init
	BauotechAlgoConnector_Init();

	for (int videoIndex =0; videoIndex < numerOfVideos; videoIndex++)
		BauotechAlgoConnector_Config(videoIndex, algo, width, height, pixelWidth, image_size, youDraw, nullptr, (char*)"C:\\Program Files\\Bauotech\\cameras.json");

	videoIndex = 0;


	int key = 0;
	int wait = 1;

	while (frameNum < skipFrames && !frame.empty()) {
		cap >> _frame;
		//if (0)  frame = _frame(debugROI);   else // DDEBUG TEST 
		frame = _frame;

		frameNum++;
	}


	auto prev = std::chrono::system_clock::now();

	int videoInd = 0;

	//-----------------
	// Video Main Loop
	//-----------------
	while (!frame.empty()) {
		memcpy(pData[videoInd], frame.data, image_size);
		//size_t sizeTemp(frame.cols * frame.rows * 3); // 24 bit
		youDraw = 1; // DDEBUG
		//BauotechAlgoConnector_Run(algo, pData, width, height, pixelWidth, image_size, youDraw, pObjects, pObjectCount);
		BauotechAlgoConnector_Run3(videoInd, pData[videoInd], frameNum); // tsQueue runner

		ALGO_DETECTION_OBJECT_DATA AIObjects[10];
		int objectsDetected = BauotechAlgoConnector_GetAlgoObjectData(0, -1, AIObjects);




		AIObjectVec.clear();
		for (int i = 0; i < objectsDetected; i++)
			AIObjectVec.push_back(AIObjects[i]);


		// DRAW image & detections 

		if (DrawDetections)
			key = draw(height, width, (char*)pData[videoInd], AIObjectVec, frameNum);
		else {
			key = draw(height, width, (char*)pData[videoInd], std::vector <ALGO_DETECTION_OBJECT_DATA>(), frameNum);
			if (frameNum % 60 == 0)
				std::cout << AIObjectVec.size() << " object detected \n";
		}
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


int main_yolo8()
{
	// Load class list.
	vector<string> class_list = load_class_list();;
	Mat frame;
	VideoCapture cap("Resources/sample.mp4");
	Net net;
	load_net_(net, m_isCuda);
	vector<Mat> detections;

	auto start = std::chrono::high_resolution_clock::now();
	int frame_count = 0;
	float fps = -1;
	int total_frames = 0;


	while (1) {
		cap.read(frame);
		// Process the image.
		detections = pre_process(frame, net);
		//cout << "number of detections : " << detections.size() << endl;
		Mat img = post_process(frame, detections, class_list);
		frame_count++;
		total_frames++;
		//// Put efficiency information.
		//// The function getPerfProfile returns the overall time for     inference(t) and the timings for each of the layers(in layersTimes).
		vector<double> layersTimes;
		double freq = getTickFrequency() / 1000;
		double t = net.getPerfProfile(layersTimes) / freq;
		string label = format("Inference time : %.2f ms", t);
		putText(img, label, Point(20, 40), FONT_FACE, FONT_SCALE, RED);

		if (frame_count >= 30)
		{

			auto end = std::chrono::high_resolution_clock::now();
			fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

			frame_count = 0;
			start = std::chrono::high_resolution_clock::now();
		}

		if (fps > 0)
		{

			std::ostringstream fps_label;
			fps_label << std::fixed << std::setprecision(2);
			fps_label << "FPS: " << fps;
			std::string fps_label_str = fps_label.str();

			cv::putText(img, fps_label_str.c_str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
		}
		imshow("Output", img);

		char c = (char)waitKey(1);
		if (c == 27)
			break;
	}
	cap.release();
	destroyAllWindows();

	return 0;
}
#else

/// ////////////////////////////////////////


int main_SHOVAL(int argc, char* argv[])
{
	bool ASYNC = false; // DDEBUG flag 

	std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjectVec;
	ALGO_DETECTION_OBJECT_DATA AIObjects[MAX_CAMERAS][10];

	int videosNum = 1; // DDEBUG 
	videosNum = MIN(videosNum, MAX_CAMERAS);
	int frameNum = 0;
	int skipFrames = 0;
	std::string videoName;
	uint32_t videoIndex = 0;
	int videoTodisplayInd = 0;


	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " <video file name> [skip frames]\n";
		return -1;
	}

	if (argc > 1)
		videoName = argv[1];
	if (argc > 2)
		skipFrames = atoi(argv[2]);

	int DrawDetections = 2; // full display
	if (argc > 3)
		if (toUpper(std::string(argv[3])) == "NODRAW")
			DrawDetections = 0; // no display
		else if (toUpper(std::string(argv[3])) == "NODETECTION")
			DrawDetections = 1;  // display frame w/o detections

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


	// Draw camera ROI
	readCamerasJson(FILES::CAMERAS_FILE_NAME, videoIndex, g_cameraInfos);
	for (auto &camInf : g_cameraInfos) {
		if (camInf.m_polyPoints.empty()  || !camInf.checkPolygon(frame.cols, frame.rows))
			camInf.m_polyPoints = { cv::Point(0,0), cv::Point(frame.cols-1,0), cv::Point(frame.cols-1,frame.rows-1), cv::Point(0,frame.rows-1) };
	}


	BAUOTECH_AND_BENNY_KAROV_ALGO algo = BAUOTECH_AND_BENNY_KAROV_ALGO::ALGO_SHOVAL;
	// Params (global)
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t pixelWidth = 0; // in Bytes
	uint32_t image_size = 0;
	uint8_t youDraw = 0;

	// Params per camera:
	uint8_t* pData[MAX_CAMERAS];
	ALGO_DETECTION_OBJECT_DATA Objects[MAX_CAMERAS][10];
	ALGO_DETECTION_OBJECT_DATA* pObjects[MAX_CAMERAS];
	uint32_t objectCount[MAX_CAMERAS];
	uint32_t* pObjectCount[MAX_CAMERAS];
	uint32_t alertCount[MAX_CAMERAS];
	uint32_t* pAlertCount[MAX_CAMERAS];


	width = frame.cols;
	height = frame.rows;
	image_size = width * height * frame.channels();
	pixelWidth = frame.channels(); //* 8; // DDEBUG : for opencv  RGB format

	for (int i = 0; i < videosNum; i++) {
		pData[i] = (uint8_t*)malloc(image_size);
		memcpy(pData[i], frame.data, image_size);
	}



	// Init
	BauotechAlgoConnector_Init();


	for (int _videoIndex = 0; _videoIndex < videosNum; _videoIndex++) {
		if (ASYNC)
			BauotechAlgoConnector_Config(_videoIndex, algo, width, height, pixelWidth, image_size, youDraw, nullptr, (char*)FILES::CAMERAS_FILE_NAME.c_str());
		else 
			BauotechAlgoConnector_Config_sync(_videoIndex, algo, width, height, pixelWidth, image_size, youDraw, nullptr, (char*)FILES::CAMERAS_FILE_NAME.c_str());

		pData[_videoIndex] = (uint8_t*)malloc(image_size);
		memcpy(pData[_videoIndex], frame.data, image_size);
	}

	int key = 0;
	int wait = 10;

	while (frameNum < skipFrames && !frame.empty()) {
		cap >> frame;
		frameNum++;
	}

	if (1) // DDEBUG
		cv::imwrite("c:\\tmp\\cameraFrame.png", frame);

	CTimer timer, timer30;
	timer.start();
	timer30.start();
	/*
	auto prev30 = std::chrono::system_clock::now();
	auto prev = prev30;
	*/
	float maxElapsed = 0;
	int objectsDetected[MAX_CAMERAS];
	//------------------------------------------------------
	// Video Main Loop
	//------------------------------------------------------
	while (!frame.empty()) {
		// Algo process:
		for (int _videoIndex = 0; _videoIndex < videosNum; _videoIndex++) {
			memcpy(pData[_videoIndex], frame.data, image_size);

			if (ASYNC)
				BauotechAlgoConnector_Run3(_videoIndex, pData[_videoIndex], frameNum); // tsQueue runner
			else 
				objectsDetected[_videoIndex] = BauotechAlgoConnector_Run3_sync(_videoIndex, pData[_videoIndex], AIObjects[_videoIndex], frameNum); // tsQueue runner
		}

		if (ASYNC) {
			for (int _videoIndex = 0; _videoIndex < videosNum; _videoIndex++) {
				objectsDetected[_videoIndex] = BauotechAlgoConnector_GetAlgoObjectData(_videoIndex, -1, AIObjects[_videoIndex]);
			}
		}

		float elapsed = timer.sample();
		maxElapsed = MAX(maxElapsed, elapsed); // keep max delay 

		if (frameNum % 30 == 0) {
			//duration elapsed = curr - prev30;

			float elapsed30 = timer30.sample();
			std::cout << " FPS = " << 1000. / (elapsed30 / 30.) << "\n";
			std::cout << " Max frame duration = " << maxElapsed << "\n";

			//prev30 = curr;
			maxElapsed = 0;
		}

		if (frameNum % 30 == 0) {
			for (int _videoIndex = 0; _videoIndex < videosNum; _videoIndex++)
				std::cout << "camera (" << _videoIndex << "): " << objectsDetected[_videoIndex] << " Objects were detected \n";
		}
		//----------------------------------------
		//      DRAW RESULTS :
		//----------------------------------------
		// Convert list to vector

		AIObjectVec.clear();
		for (int i = 0; i < objectsDetected[videoTodisplayInd]; i++)
			AIObjectVec.push_back(AIObjects[videoIndex][i]);

		if (DrawDetections == 2)
			key = draw(height, width, (char*)pData[videoTodisplayInd], AIObjectVec, frameNum);
		else if (DrawDetections == 1)
			key = draw(height, width, (char*)pData[videoTodisplayInd], std::vector <ALGO_DETECTION_OBJECT_DATA>(), frameNum);


		//------------------
		// Loop control :
		//------------------
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

	}  	// while (!frame.empty())

	// termination

	BauotechAlgoConnector_Release();

}



int main(int argc, char* argv[])
{

	if (1)
		return main_SHOVAL(argc, argv);
	else
		return main_camFileGen(argc, argv);

}


#endif 