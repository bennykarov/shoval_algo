#pragma once


namespace CONSTANTS {
	int const FPS = 30;
	int const DetectionFPS = 3;
	int const motionDetectionFPS = DetectionFPS;
	int const MogEmphasizeFactor = 4;
	int const StableLen = 4;
	int const minLenForPrediction = 10 + 1; // keep odd for acceleration calc
	int const maxLenForPrediction = FPS + 1;  // keep odd for acceleration calc 
	int const StableLenForPrediction = minLenForPrediction+10; // Prediction doesn't usethe first 10 unstable frames

	// NEWs
	int const DEFAULT_SKIP_FRAMES_BGSEG = 1;
	int const DEFAULT_SKIP_FRAMES_IN_MOTION_YOLO = 2;//  2; // Process yolo in case motion was detected 
	int const DEFAULT_SKIP_FRAMES_YOLO = 30; // 10;   // Process yolo in constant intervals 

};


namespace  SIZES {
	const int minVehicleWidth = 55 * 2;

	const int minHumanWidth = 15 *2;
	const int maxHumanWidth = 50 * 2;
	const int minHumanHeight = 30 * 2;
	const int maxHumanHeight = 60 * 2;
};


struct Config
{
	// Operational 
	std::string videoName;
	std::string roisName;

	std::string modelFolder = "../BauoSafeZone/config_files/";
	//int showTime = 1;
	int showTruck = 0;
	int showMotion = 0;
	int debugLevel = 0;
	float displayScale = 1.;
	// OPtimization
	int skipMotionFrames = CONSTANTS::DEFAULT_SKIP_FRAMES_BGSEG;
	int skipDetectionFramesInMotion = CONSTANTS::DEFAULT_SKIP_FRAMES_IN_MOTION_YOLO; // in case  motion was detected
	int skipDetectionFrames = CONSTANTS::DEFAULT_SKIP_FRAMES_YOLO; // in case  motion was detected 
	//int detectionInterval = CONSTANTS::DEFAULT_INTERVAL_FRAMES_YOLO; // in case NO motion was detected 
	// Algo
	int motionType = 1;
	int trackerType = 0;
	int MLType = 10;
	float scale = 0.5;
	int waitKeyTime=1;
	int record = 0;
	int demoMode=0;
	// MOG2 params:
	int MHistory = 100;
	float MvarThreshold = 580.0;
	float MlearningRate = -1.;
	int useGPU = 1;
	std::vector <int> camROI = { 0,0,0,0 }; // RECT 
	cv::Rect  motionROI;
	};
