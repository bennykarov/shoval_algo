#pragma once


#define MAX_OBJECTS 30 

const bool YOLO_MNGR = true; // YOLO detectors limiter 


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
	int const DEFAULT_STEP_FRAMES_BGSEG = 1;
	//int const DEFAULT_SKIP_FRAMES_IN_MOTION_YOLO = 2;//  2; // Process yolo in case motion was detected 
	int const DEFAULT_STEP_FRAMES_YOLO = 1; // 10;   // Process yolo in constant intervals 

	int const DEFAULT_LOADBALANCER_RESOURCE = 4;

};


namespace  SIZES {
	const int minVehicleWidth = 55 * 2;

	const int minHumanWidth = 15 *2;
	const int maxHumanWidth = 50 * 2;
	const int minHumanHeight = 30 * 2;
	const int maxHumanHeight = 60 * 2;
};

namespace  TRACKER {
	const float lowScore = 0.8;
	const int maxHidden = 3;
	const int scoreHistoryLen = 10;
	const float scale = 0.4;
};


struct APIData {
	std::vector <int> camID;
	std::vector <int> ployID;
	std::vector <int> personHeight;
};


struct Config
{
	// Operational 
	std::string videoName;
	std::string roisName;

	std::string modelFolder = "../BauoSafeZone/config_files/";
	int showMotion = 0;
	int debugLevel = 0;
	int debugLevel_LB = 0;

	float displayScale = 1.;

	// OPtimization - skip (steps) for Motion (BGSEG), Detection 
	int stepMotionFrames = CONSTANTS::DEFAULT_STEP_FRAMES_BGSEG;
	int stepDetectionFrames = CONSTANTS::DEFAULT_STEP_FRAMES_YOLO; // YOLO steps 

	// Algo
	int motionDetectionType = 0;
	int trackerType = 0;
	int trackerStep = 1;
	int MLType = 1;
	float scale = 1.0; 
	int record = 0;

	int debugTraceCamID = -1;

	// MOG2 params:
	int MHistory = 50;
	float MvarThreshold = 2700.0; // 580.0;
	float MlearningRate = -1.;
	int useGPU = 1;
	int GPUBatchSize = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;
	int LB_scheme = 0;
	std::vector <int> camROI = { 0,0,0,0 }; // RECT 
	cv::Rect  motionROI;
	};
