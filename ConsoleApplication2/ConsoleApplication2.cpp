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
#pragma comment(lib, "opencv_dnn470d.lib") // main_yoloTest()
#else
#pragma comment(lib, "opencv_core470.lib")
#pragma comment(lib, "opencv_highgui470.lib")
#pragma comment(lib, "opencv_video470.lib")
#pragma comment(lib, "opencv_videoio470.lib")
#pragma comment(lib, "opencv_imgcodecs470.lib")
#pragma comment(lib, "opencv_imgproc470.lib")
#pragma comment(lib, "opencv_dnn470.lib")

#endif



#include <iostream>
#include <thread>        
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "rapidjson/document.h" 
#include "rapidjson/filereadstream.h" 
#include "rapidjson/error/en.h"


// read config file
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp> 


#include "utils.hpp"
#include "../BauotechAIConnectorDll/files.hpp"
#include "../BauotechAIConnectorDll/AlgoApi.h"
#include "../BauotechAIConnectorDll/config.hpp"
#include "../BauotechAIConnectorDll/timer.hpp"
//#include "../BauotechAIConnectorDll/yolo/yolo.hpp"
#include "../BauotechAIConnectorDll/CObject.hpp"  // required by alert.hpp
#include "../BauotechAIConnectorDll/alert.hpp"
#include "../BauotechAIConnectorDll/database.hpp"

#include "yolo_.hpp" 
#include "draw.hpp"

//std::string CAMERAFNAME = "";
//std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\Harchava_PTZ.json)";
std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\Harchava.json)";
//std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\Spikes.json)";
//std::string  CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\fullscreen_person_car.json)";
//std::string  CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\cameras1_sportech.json)";
// std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\double_motion.json)";
//std::string  CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\cameras2_car_motion.json)";
//std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\cameras1_persons.json)";
//std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\camera_staticCars.json)";

//std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\cameras10.json)";
//std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\cameras_Double.json)";
//std::string CAMERAFNAME = R"(C:\Program Files\Bauotech\AI\cameras_multiPoly.json)";


static std::mutex     gCamRequestMtx;
HANDLE g_hConsole;
//int g_motionType = MotionType::OnlyMoving;
int g_motionType = MotionType::MotionOrNot;
int g_skip = 1;
float g_rotateImageAngle = 0.; // DDEBUG roate original image!

int main_siamRPN(int argc, char** argv);
int main_yoloTest();



// GLOBALS:
int main_drawPoly(int argc, char* argv[]);
std::vector <CAlert> g_cameraInfos;
//std::vector <std::string> g_cameraInfosLabelStr; // duplicate te original label string for multiple labels tests
std::vector <int> gVideosToRun; // cams batch 


int NumberOfVideos = 1;  // override by cmd argument 
int videoTodisplayInd = 0;


//std::vector <cv::Scalar> g_colors = { cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255), cv::Scalar(255,255,0), cv::Scalar(255,0,255), cv::Scalar(0,255,255) , cv::Scalar(155,55,0), cv::Scalar(25,0,55), cv::Scalar(110,155,255) };
/*------------------------------------------------------------------------------------------------------
 *									U T I L S     function 
 ------------------------------------------------------------------------------------------------------*/
 
class AppParams {
public:
	std::string cameraJsonName = "";
	std::string videoName = "";
	int cameraNum=1;
	int label=2;
	int motionType=0;
	int startFrame = 0;
};

bool file_exists(std::string fname)
{
	ifstream fin;
	fin.open(fname.c_str());
	return bool(fin);
}

bool readConfigFile(std::string ConfigFName, AppParams& params)
{
	if (!file_exists(ConfigFName)) {
	//if (!GetFileAttributes(stringToWstring_(ConfigFName).c_str()) != (DWORD)-1)) {
		std::cout << "WARNING : Can't find ConsoleApp Config.ini file, use default values \n";
		return false;
	}

	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(ConfigFName, pt);

	params.videoName = pt.get<std::string>("input", params.videoName);
	params.cameraJsonName = pt.get<std::string>("json", params.cameraJsonName);
	params.videoName.erase(std::remove(params.videoName.begin(), params.videoName.end(), '\"'), params.videoName.end());
	params.cameraJsonName.erase(std::remove(params.cameraJsonName.begin(), params.cameraJsonName.end(), '\"'), params.cameraJsonName.end());


	params.cameraNum = pt.get<int>("camNums", params.cameraNum);
	params.label = pt.get<int>("label", params.label);
	params.motionType = pt.get<int>("motionType", params.motionType);
	params.startFrame = pt.get<int>("startFrame", params.startFrame);

	return true;
}
 /*--------------------------------------------------------------------------------------------
 *  Read cameras ROI ( as vector (polygon))
 * Format:
 * camID <>
 * detection-type <>
 * max-allowed <>
 * polygon <point1_X, point1_Y, point2_X, point2_Y, ...>
 * cameraIndex = -1 (or any negative num) means read all cam IDs
  --------------------------------------------------------------------------------------------*/
int readCamerasJson(std::string fname, std::vector <CAlert>& cameras, int cameraIndex)
{
	FILE* fp;
	fopen_s(&fp, fname.c_str(), "rb");

	// Check if the file was opened successfully 
	if (!fp) {
		std::cerr << "Error: unable to open file"
			<< std::endl;
		return 0;
	}

	// Read the file into a buffer 
	char readBuffer[65536];
	rapidjson::FileReadStream is(fp, readBuffer,
		sizeof(readBuffer));

	// Parse the JSON document 
	rapidjson::Document doc;
	doc.ParseStream(is);

	// Check if the document is valid 
	if (doc.HasParseError()) {
		std::cerr << "Error: failed to parse JSON document"
			<< std::endl;
		fprintf(stderr, "\nError(offset %u): %s\n",
			(unsigned)doc.GetErrorOffset(),
			GetParseError_En(doc.GetParseError()));
		fclose(fp);
		Beep(1000, 10);
		return 0;
	}

	// Close the file 
	fclose(fp);


	// Parse file

	rapidjson::Value::ConstValueIterator itr;

	try {
		for (itr = doc.Begin(); itr != doc.End(); ++itr) {
			CAlert camInfo;
			// Access the data in the object
			int camID = itr->GetObject_()["camID"].GetInt();
			if (cameraIndex >= 0 && camID != cameraIndex)
				continue;
			camInfo.m_camID = camID;
			//std::cout << "camID: " << camID << std::endl;
			std::string typeStr = itr->GetObject_()["detection-type"].GetString();
			camInfo.m_label = getYoloClassIndex(typeStr);

			camInfo.m_maxAllowed = itr->GetObject_()["max-allowed"].GetInt();

			if (itr->HasMember("polygon") && itr->GetObject_()["polygon"].IsArray()) {
				std::vector <int> points;
				rapidjson::Value::ConstValueIterator itrP;
				for (itrP = itr->GetObject_()["polygon"].Begin(); itrP != itr->GetObject_()["polygon"].End(); ++itrP)
					points.push_back(itrP->GetInt());

				CHECK_exception(true, "Error in Polygon points list - list has OD elements");

				for (int p = 0; p < points.size(); p += 2)
					camInfo.m_polyPoints.push_back(cv::Point(points[p], points[p + 1]));
			}

			if (itr->HasMember("motionType")) 
				camInfo.m_motionType = itr->GetObject_()["motionType"].GetInt();

			cameras.push_back(camInfo);
		}
	}
	catch (const std::exception& err) {
		std::cout << "Error in parsing file : " << fname << " : " << err.what() << "\n";
	}

	return cameras.size();
}


/*------------------------------------------------------------------------------------------------------
 * Dynamic add poligons:
 * takes the first cam's poly from json file , duplicated it cameraNum times.
 * Switch the label of the cams (hard coded)  person (0) and car (2) 
 ------------------------------------------------------------------------------------------------------*/

int addPolygonsAuto(std::string cameraFName, cv::Mat frame, int cameraNum, int detectionLabel)
{
	CAlert templateCamInf;

	//bool switchLabels = detectionLabel == -2; // negative label means switch labels (odd & even cams)

	// Read json for polygon setting
	readCamerasJson(cameraFName, g_cameraInfos, -1); // read all cam's

	if (g_cameraInfos.empty()) {
		templateCamInf.m_polyPoints = { cv::Point(0,0), cv::Point(frame.cols - 1,0), cv::Point(frame.cols - 1,frame.rows - 1), cv::Point(0,frame.rows - 1) };
		std::cout << " Can't open file " << cameraFName << " use entire image ROI as polygon \n";
	}
	else 
		templateCamInf = g_cameraInfos[0];

	int polygonId = 0;
	templateCamInf.m_label = detectionLabel;
	templateCamInf.m_maxAllowed = 0;


	for (int camID = 0; camID < cameraNum;camID++) {
		if (camID >= MAX_VIDEOS)
			continue;

		templateCamInf.m_camID = camID;

		/* May be Dangerous - keep out !
		if (switchLabels)
		{
			if (templateCamInf.m_camID % 2 == 0)
				templateCamInf.m_label = 2;
			else
				templateCamInf.m_label = 0;
		}
		*/


		std::vector <int> polygonVec;
		for (auto point : templateCamInf.m_polyPoints) {
			polygonVec.push_back(point.x);
			polygonVec.push_back(point.y);
		}

		char* labelsStr = (char*)labelToStr(templateCamInf.m_label); // change for debugging multiple labels 
		int _videoIndex = templateCamInf.m_camID;
		BauotechAlgoConnector_AddPolygon(_videoIndex,
			_videoIndex, //CamID,
			polygonId++,
			labelToStr(templateCamInf.m_label),//DetectionType,  // debugLabel,  
			templateCamInf.m_maxAllowed, // MaxAllowed,
			&(polygonVec[0]),//Polygon,
			templateCamInf.m_polyPoints.size() * 2,
			g_motionType); 
	}

	return g_cameraInfos.size();
}
int addPolygonsFromCameraJson(std::string cameraFName,cv::Mat frame)
{
	int camID = -1; // read all cameras
	readCamerasJson(cameraFName, g_cameraInfos, camID); // read all cam's

	for (auto& camInf : g_cameraInfos) {
		if (camInf.m_polyPoints.empty() || !camInf.checkPolygon(frame.cols, frame.rows))
			camInf.m_polyPoints = { cv::Point(0,0), cv::Point(frame.cols - 1,0), cv::Point(frame.cols - 1,frame.rows - 1), cv::Point(0,frame.rows - 1) };
	}


	int polygonId = 0;

	for (auto camInf : g_cameraInfos) {
		if (camInf.m_camID >= MAX_VIDEOS)
			continue;

		std::vector <int> polygonVec;
		for (auto point : camInf.m_polyPoints) {
			polygonVec.push_back(point.x);
			polygonVec.push_back(point.y);
		}

		char* labelsStr = (char*)labelToStr(camInf.m_label); // change for debugging multiple labels 
		//char debugLabel[] = "car,person";
		int _videoIndex = camInf.m_camID;
		BauotechAlgoConnector_AddPolygon(_videoIndex,
			_videoIndex, //CamID,
			polygonId++,
			labelToStr(camInf.m_label),//DetectionType,  // debugLabel,  
			camInf.m_maxAllowed, // MaxAllowed,
			&(polygonVec[0]),//Polygon,
			camInf.m_polyPoints.size() * 2, 
			camInf.m_motionType); // polygonSize);
	}

	return g_cameraInfos.size();
}





//__cdecl*
void consoleCameraRequestCallback(const uint32_t *camera, int size)
{
	std::unique_lock<std::mutex> lock(gCamRequestMtx);

	gVideosToRun.clear();

	for (int i = 0; i < size; i++)
		gVideosToRun.push_back(camera[i]);

	lock.unlock();
}



// DDEBUG IMAGE PREPROCESS 
cv::Mat tune_imgae(cv::Mat frame, float rotateAngle=0.)
{ 
	bool resize = true;

	if (true)
		return frame; // DDEBUG 

	cv::Mat _frame;
	cv::resize(frame, _frame, cv::Size(640, 480));
	frame = _frame;
	if (rotateAngle != 0) {
		cv::Point2f center((frame.cols - 1) / 2.0, (frame.rows - 1) / 2.0);
		cv::Mat matRotation = cv::getRotationMatrix2D(center, rotateAngle, 1.0);
		cv::Mat rotated_image;
		cv::warpAffine(frame, _frame, matRotation, frame.size());
	}


	return _frame;
}


int main_SHOVAL(int argc, char* argv[])
{
	CDISPLAY disp;

	bool ASYNC = true; // DDEBUG flag
	uint8_t invertImg = 0; //  readConfigFIle(ConfigFName) == 1;
	int configSet = 0;

	std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjectVec;
	ALGO_DETECTION_OBJECT_DATA AIObjects[MAX_VIDEOS][MAX_OBJECTS];

	int frameNum = 0;
	int startSkipFrames = 100;
	std::string videoName;

	uint32_t videoIndex = 0;

	AppParams params;
	int detectionLabel = -1;
	if (readConfigFile("appConfig.ini", params)) {
		// Read from appConfig.ini
		//-----------------------------
		videoName = params.videoName;
		CAMERAFNAME = params.cameraJsonName;
		detectionLabel = params.label;
		g_motionType = params.motionType;
		NumberOfVideos = params.cameraNum;
		startSkipFrames = params.startFrame;
	}
	else {
		// If no config file - use the cmd line:
		//----------------------------------------
		if (argc < 4) {
			std::cout << "Usage: " << argv[0] << " <video file name> <numbero of cameras> <detection class>\n";
			std::cout << "<detection class> = -1    means : takes class from camera.json \n";
			return -1;
		}

		if (argc > 1) {
			videoName = argv[1];
			//videoName.erase(std::remove(videoName.begin(), videoName.end(), '"'), videoName.end());
		}


		if (argc > 3) {
			NumberOfVideos = atoi(argv[2]);
			detectionLabel = atoi(argv[3]);
		}

		if (argc > 4 && (std::string(argv[4]).size() == 1))
			g_motionType = atoi(argv[4]);
	}

	NumberOfVideos = MIN(NumberOfVideos, MAX_VIDEOS);
	videoTodisplayInd = min(videoTodisplayInd, NumberOfVideos - 1);


	
	MessageBoxA(0, std::string("cams=" + std::to_string(NumberOfVideos) + " ; label=" + std::to_string(detectionLabel) + " ; motion=" + std::to_string(g_motionType) + "; start frame="+std::to_string(startSkipFrames)).c_str(), "Info", MB_OK);
	/*
	if (argc > 3)
		startSkipFrames = atoi(argv[3]);
	*/



	int DrawDetections = 2; // full display
	if (argc > 3)
		if (toUpper(std::string(argv[3])) == "NODRAW")
			DrawDetections = 0; // no display
		else if (toUpper(std::string(argv[3])) == "NODETECTION")
			DrawDetections = 1;  // display frame w/o detections


	cv::VideoCapture cap;
	cv::Mat frame;

	if (!cap.open(videoName)) {
		SetConsoleTextAttribute(g_hConsole, 64);
		std::cerr << "ERROR ERROR : Can't open file  " << videoName << ")\n";
		SetConsoleTextAttribute(g_hConsole, 7);
		return -1;
	}


	float ScaleDown = 1.;
	// read first image
	cap >> frame;

	if (ScaleDown != 1.) {
		cv::Mat _frame;
		cv::resize(frame, _frame, cv::Size(0, 0), ScaleDown, ScaleDown);
		frame = _frame;
	}

	else  
		frame = tune_imgae(frame, g_rotateImageAngle);  // DDEBUG  DDEBUG  DDEBUG  DDEBUG 


	if (frame.empty()) {
		std::cout << "END OF VIDEO FILE" << videoName << ")\n";   return -1;
	}

	// read camera info - if no ROI - use full frame
	BAUOTECH_AND_BENNY_KAROV_ALGO algo = BAUOTECH_AND_BENNY_KAROV_ALGO::ALGO_SHOVAL;
	// Params (global)
	uint32_t width = 0;
	uint32_t height = 0;
	uint32_t pixelWidth = 0; // in Bytes
	uint32_t image_size = 0;
	uint8_t youDraw = 0; // DDEBUG DRAW 

	// Params per camera:
	uint8_t* pData[MAX_VIDEOS];
	ALGO_DETECTION_OBJECT_DATA Objects[MAX_VIDEOS][MAX_OBJECTS];
	ALGO_DETECTION_OBJECT_DATA* pObjects[MAX_OBJECTS];
	uint32_t objectCount[MAX_VIDEOS];
	uint32_t* pObjectCount[MAX_VIDEOS];
	uint32_t alertCount[MAX_VIDEOS];
	uint32_t* pAlertCount[MAX_VIDEOS];


	width = frame.cols;
	height = frame.rows;
	image_size = width * height * frame.channels();
	pixelWidth = frame.channels(); //* 8; // DDEBUG : for opencv  RGB format

	// Invert image if required:
	cv::Mat post_frame;
	if (invertImg == 1)
		cv::flip(frame, post_frame, 0);
	else
		post_frame = frame;

	for (int i = 0; i < NumberOfVideos; i++) {
		pData[i] = (uint8_t*)malloc(image_size);
		memcpy(pData[i], post_frame.data, image_size);
	}

	// Init
	//bool loadBalance = true;
	BauotechAlgoConnector_Init();

	BauotechAlgoConnector_SetCameraRequestCallback(consoleCameraRequestCallback);


	int numOfPolygons;

	if (detectionLabel == -1) {// read from camera json
		numOfPolygons = addPolygonsFromCameraJson(CAMERAFNAME, frame);
		if (numOfPolygons == 0)
			numOfPolygons = addPolygonsAuto(CAMERAFNAME, frame, NumberOfVideos, detectionLabel); // THis function add default polygon == full frame

	}
	else 
		numOfPolygons = addPolygonsAuto(CAMERAFNAME, frame, NumberOfVideos, detectionLabel);

	if (numOfPolygons == 0)
		std::cout << "\n*** WARNING : nopolygons had beed defined ! \n";

	for (int i = 0; i < NumberOfVideos; i++)
		gVideosToRun.push_back(i);
	

	for (int _videoIndex : gVideosToRun) {
		if (ASYNC)
			BauotechAlgoConnector_Config(_videoIndex, algo, width, height, pixelWidth, image_size, youDraw, invertImg, nullptr);
		else
			BauotechAlgoConnector_Config_sync(_videoIndex, algo, width, height, pixelWidth, image_size, youDraw, invertImg, nullptr);

		if (0)
		{
			pData[_videoIndex] = (uint8_t*)malloc(image_size);
			memcpy(pData[_videoIndex], frame.data, image_size);
		}
	}

	// DDEBUG make cam 1 activeCam:
	if (0) {
		uint32_t cam = videoTodisplayInd;
		uint32_t type = 1;
		BauotechAlgoConnector_SetCameraType(cam, type);
	}
	
	
	int key = 0;
	int wait = 10;

	while (frameNum < startSkipFrames && !frame.empty()) {
		cap >> frame;
		frameNum++;
	}


	if (startSkipFrames > 0)
		frame = tune_imgae(frame, g_rotateImageAngle);



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
	int objectsDetected[MAX_VIDEOS];
	//------------------------------------------------------
	// Video Main Loop
	//------------------------------------------------------
	int skipEveryframes = 0;
	while (!frame.empty()) {
		
		if (ASYNC)
			Sleep(20); // DDEBUG DDEBUG simulate  RT cameras
		
 
		cv::Mat post_frame;
		if (invertImg == 1)
			cv::flip(frame, post_frame, 0);
		else 
			post_frame = frame;

		// Launch Algo process for all cameras :
		for (int _videoIndex : gVideosToRun) {
			memcpy(pData[_videoIndex], post_frame.data, image_size);

			if (ASYNC)
				BauotechAlgoConnector_Run3(_videoIndex, pData[_videoIndex], frameNum); // tsQueue runner
			else 			
				objectsDetected[_videoIndex] = BauotechAlgoConnector_Run3_sync(_videoIndex, pData[_videoIndex], AIObjects[_videoIndex], frameNum); // tsQueue runner
		}

		// Read results 
		if (ASYNC) {
			//for (int _videoIndex = 0; _videoIndex < NumberOfVideos; _videoIndex++) {			
			for (int _videoIndex : gVideosToRun) {
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

		if (0) // DONT PRINT DETECTIONS 
		if (frameNum % 30*20 == 0) {
			//for (int _videoIndex = 0; _videoIndex < NumberOfVideos; _videoIndex++)
			for (int _videoIndex : gVideosToRun)
				std::cout << "camera (" << _videoIndex << "): " << objectsDetected[_videoIndex] << " Objects had been detected \n";
		}
		//----------------------------------------
		//      DRAW RESULTS :
		//----------------------------------------
		// Convert list to vector

		AIObjectVec.clear();

		for (int i = 0; i < objectsDetected[videoTodisplayInd]; i++) {
			AIObjectVec.push_back(AIObjects[videoTodisplayInd][i]);
			if (AIObjectVec.back().ObjectType != 2)
				int debug = 10;
		}




		if (!AIObjectVec.empty()) { // DDEBUG  DDEBUG DDEBUG DDEBUG - beep every detection!
			//MessageBoxA(0, std::string("find " + std::to_string(AIObjectVec.size()) + "objects ").c_str(), "EXCEPTION!", MB_OK);
				Beep(900, 60);
			int debug = 10;
		}

		if (DrawDetections == 2)
			key = disp.draw(height, width, (char*)pData[videoTodisplayInd], AIObjectVec, g_cameraInfos, frameNum, 0.7, invertImg);
		else if (DrawDetections == 1)
			key = disp.draw(height, width, (char*)pData[videoTodisplayInd], std::vector <ALGO_DETECTION_OBJECT_DATA>(), g_cameraInfos, frameNum, 0.7, invertImg);


		//------------------
		// Loop control :
		//------------------
		if (key == 'q' || key == 27)
			break;

		// Pause on start
		//if (frameNum == 0) cv::waitKey(-1);

		for (int i = g_skip; i > 0;i--) {
			cap >> frame;
			frameNum++;
		}


		cap >> frame;
		frameNum++;

		if (frameNum == 40)
			cv::waitKey(-1); // DDEBUG 


		if (!frame.empty()) {
			if (ScaleDown != 1.) {
				cv::Mat _frame;
				cv::resize(frame, _frame, cv::Size(0, 0), ScaleDown, ScaleDown);
				frame = _frame;
			}
			else  // DDEBUG  DDEBUG  DDEBUG  DDEBUG 
				frame = tune_imgae(frame, g_rotateImageAngle);
			
		}

		if (0) // endless loop : 
		if (frame.empty()) {
			cap.set(cv::CAP_PROP_POS_FRAMES, 0); // start again from the beginning
			cap >> frame;
		}


		// Trick - switch displaycam each 8 sec:
		// if (g_motionType == -2 && (frameNum % (30*8)) == 0)   videoTodisplayInd = (videoTodisplayInd + 1) % NumberOfVideos;


	}  	// while (!frame.empty())

	// termination

	BauotechAlgoConnector_Release();

}




std::stringstream printMsg;
void print()
{
	std::cout << printMsg.str() ;
}

template <typename T, typename... Types>
void print(T var1, Types... var2)
{
	//cout << var1 << endl;

	printMsg << var1 << "\n";


	print(var2...);
}

// Driver code
int main(int argc, char* argv[])
{

	g_hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	// you can loop k higher to see more color choices


	return main_yoloTest();
	//return main_SHOVAL(argc, argv);
	//return main_siamRPN(argc, argv);

	//return main_drawPoly(argc, argv);

}



/*------------------------------------------------------------------------------------
	Test pure YOLO detection and display results	
 ------------------------------------------------------------------------------------*/
int main_yoloTest()
{
	_CYolo8 _yolo;
	CDISPLAY disp;

	bool _isCuda = true;
	std::string modelFolder = "C:\\Program Files\\Bauotech\\dll\\algo\\models";
	std::vector<YDetection> _Yolotput;
	std::string videoName;

	//int key = draw(height, width, (char*)pData[videoTodisplayInd], AIObjectVec, g_cameraInfos, frameNum, 0.7, invertImg);
	cv::VideoCapture cap;
	cv::Mat frame;
	int frameNum = -1;
	int startSkipFrames = 0;

	AppParams params;
	int detectionLabel = -1;
	if (readConfigFile("appConfig.ini", params)) {
		// Read from appConfig.ini
		//-----------------------------
		videoName = params.videoName;
		CAMERAFNAME = params.cameraJsonName;
		detectionLabel = params.label;
		g_motionType = params.motionType;
		NumberOfVideos = params.cameraNum;
		startSkipFrames = params.startFrame;
	}


	if (!cap.open(videoName)) {
		SetConsoleTextAttribute(g_hConsole, 64);
		std::cerr << "ERROR ERROR : Can't open file  " << videoName << ")\n";
		SetConsoleTextAttribute(g_hConsole, 7);
		return -1;
	}


	if (!_yolo.init(modelFolder, _isCuda)) {
		std::cout << "Cant init YOLO net , quit \n";
		return -1;
	}




	cap >> frame;
	frameNum++;
	std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjects;
	while (!frame.empty()) {
		cap >> frame;
		frameNum++;

		_Yolotput.clear();
		if (true)
			_yolo.detect(frame, _Yolotput);
		else
			_yolo.runInference(frame, _Yolotput);

		AIObjects.clear();
		for (auto obj : _Yolotput) {
			if (obj.class_id == 0 || obj.class_id == 2) {
				ALGO_DETECTION_OBJECT_DATA newAIObj;
				newAIObj.ObjectType = obj.class_id;
				newAIObj.X = obj.box.x;
				newAIObj.Y = obj.box.y;
				newAIObj.Width = obj.box.width;
				newAIObj.Height = obj.box.height;
				newAIObj.ID = 0; // pure YOLO - no tracking 
				newAIObj.DetectionPercentage = int(obj.confidence*100.);
				AIObjects.push_back(newAIObj);
			}
		}


		if (frameNum == 40)
			cv::waitKey(-1); // DDEBUG 

		int key = disp.draw(frame, AIObjects, g_cameraInfos, frameNum, 1.0, false);

	}

}