#include <unordered_map>
#include <string>
#include <fstream> // debug file
#include <vector>

#include "opencv2/opencv.hpp"

#include "config.hpp"
#include "utils.hpp"
#include "queuing.hpp"
#include "AlgoApi.h"
#include "AlgoDetection.hpp"
#include "algoProcess.hpp"
#include "files.hpp"
#include "logger.hpp"

#include "loadBalancer.hpp"


typedef struct tagRGB32TRIPLE {
	BYTE    rgbtBlue;
	BYTE    rgbtGreen;
	BYTE    rgbtRed;
	BYTE    rgb0;
} RGB32TRIPLE;


const std::string VERSION_NUM = "1.3.0";

std::vector <uint32_t> g_activeVideos;


inline bool isVideoActive(uint32_t videoIndex)
{
	return std::find(g_activeVideos.begin(), g_activeVideos.end(), videoIndex) != g_activeVideos.end();
}

//--------------------------------------------------------------------------------------------------------
//  G L O B A L S   !
// Define a global critical section
CRITICAL_SECTION gCriticalSection;
bool m_initialize = false;

CameraRequestCallback gCameraRequestCallback = nullptr; // moved from header file
std::unordered_map<uint32_t, CameraAICallback> gAICllbacks;

CLoadBalaner  g_loadBalancer;

CAlgoProcess   g_algoProcess[MAX_VIDEOS];
TSBuffQueue g_bufQ[MAX_VIDEOS];
//--------------------------------------------------------------------------------------------------------

API_EXPORT void Crash()
{
	int* p = NULL;
	*p = 1;
}


API_EXPORT void AlgoSetTime(int hour, int min, int sec)
{

}
 
API_EXPORT int BauotechAlgoConnector_Init(bool runloadBalancer, int debugLevel)
{

	if (m_initialize) // DDEBUG avoid duplicate call !!!! 
		return 0;

	InitializeCriticalSection(&gCriticalSection);

	debugLevel = (int)DLEVEL::_SYSTEM_INFO; 
	LOGGER::init(FILES::OUTPUT_FOLDER_NAME, debugLevel);
	//LOGGER::log(DLEVEL::ERROR1, "BauotechAlgoConnector DLL version = " + VERSION_NUM);

	LOGGER::log(DLEVEL::_SYSTEM_INFO, "BauotechAlgoConnector_Init()");

	uint32_t batchSize = 8; // Eli changes - right now the 'batchSize' init inside g_loadBalancer.init() from config 

	LB_MODE loadbalanerMode;
	if (runloadBalancer)
	{
		LOGGER::log(DLEVEL::_SYSTEM_INFO, " RUNNIG  W I T H  LOAD BALANCER");
		//loadbalanerMode = LB_MODE::LIGHT; // currently a test  
		loadbalanerMode = LB_MODE::FULL;
	}
	else {
		LOGGER::log(DLEVEL::_SYSTEM_INFO, " RUNNIG  W I T H O U T  LOAD BALANCER  !!!!!");
		loadbalanerMode = LB_MODE::STATISTICS;
	}




	g_loadBalancer.init(&batchSize, loadbalanerMode);
	m_initialize = true;


	return (int)batchSize;
}

API_EXPORT void BauotechAlgoConnector_Release()
{
	if (m_initialize == true)
	{
		LOGGER::log(DLEVEL::_SYSTEM_INFO, "call to BauotechAlgoConnector_Release()");
		for (int i = 0; i < MAX_VIDEOS; i++) {
			g_algoProcess[i].terminate();
			g_algoProcess[i].~CAlgoProcess();
		}
		Sleep(1000); // wait for all thread to terminate

		DeleteCriticalSection(&gCriticalSection);
		m_initialize = false;
		LOGGER::close();
	}
}

API_EXPORT void BauotechAlgoConnector_Remove(uint32_t videoindex)
{
	if (m_initialize == true && isVideoActive(videoindex))
	{
		g_loadBalancer.remove(videoindex);
		g_algoProcess[videoindex].terminate();
		g_activeVideos.erase(std::remove(g_activeVideos.begin(), g_activeVideos.end(), videoindex), g_activeVideos.end());
		LOGGER::log(DLEVEL::_SYSTEM_INFO, "BauotechAlgoConnector_Rmove() for cam# = "+std::to_string(videoindex));
		
	}
}

/*-------------------------------------------------------------------------------------------------------------------
* Return number of object provided (1 or 0) 
* Caller reponsible to alloc * pObjects
* ------------------------------------------------------------------------------------------------------------------*/
API_EXPORT int BauotechAlgoConnector_GetAlgoObjectData(uint32_t videoIndex, int index, ALGO_DETECTION_OBJECT_DATA* pObjects)
{
	int frameNum;
	return g_algoProcess[videoIndex].getObjectData(videoIndex, index, pObjects, frameNum);

}


/*-------------------------------------------------------------------------------------------------------------------
*  runner use the TSBuffQeue buffering
 -------------------------------------------------------------------------------------------------------------------*/
API_EXPORT int BauotechAlgoConnector_Run3(uint32_t videoIndex, uint8_t* pData, uint64_t frameNumber, int64_t timestamp)
{
	bool forceList = true; // ignore input not in Loadbalncer cameraList

	// Load balancer Aquire - quit if not allowed (if allowOverflow = true)	
	auto error = g_loadBalancer.acquire(videoIndex);
	if (error != AQUIRE_ERROR::NOT_ACTIVE && error != AQUIRE_ERROR::OK)
	{
		LOGGER::log(DLEVEL::_SYSTEM_INFO, "LB Ignors cam #" + std::to_string(videoIndex));
		if (forceList)
			return false;	
	}

	// Push buffer 
	timestamp = 0; // time stamp is done in the push() function
	bool ok = g_bufQ[(int)videoIndex].push(CframeBuffer(frameNumber, timestamp, (char*)pData));
	if (!ok) {
		LOGGER::log(DLEVEL::_SYSTEM_INFO, "API: can't push frame to buffer Queue , cam = " + std::to_string(videoIndex));
		return ok;
	}

	g_algoProcess[videoIndex].WakeUp();
	return ok;
}

/*-------------------------------------------------------------------------------------------------------------------
*  Sync version of run3()
 -------------------------------------------------------------------------------------------------------------------*/
API_EXPORT int BauotechAlgoConnector_Run3_sync(uint32_t videoIndex, uint8_t* pData, ALGO_DETECTION_OBJECT_DATA* AIObjects, uint64_t frameNumber)
{
	return g_algoProcess[videoIndex].run_sync(pData, frameNumber, AIObjects);

}

API_EXPORT int BauotechAlgoConnector_AddPolygon(uint32_t videoIndex,
	int CamID,
	int polygonId,
	char* DetectionType,
	int MaxAllowed,
	int Polygon[],
	int polygonSize,
	int motionType,
	int timeLimit, // in seconds, Positive  for max limit, negative for  Minimum limit
	int personHeight)
{
	if (!g_algoProcess[videoIndex].isActive())
		return 0;

	std::string logMSG = "API..._AddPolygon(); polyID = " + std::to_string(polygonId) + " cam = " + std::to_string(videoIndex);
	LOGGER::log(DLEVEL::_SYSTEM_INFO, logMSG.c_str());

	g_algoProcess[videoIndex].addPolygon(CamID, polygonId, DetectionType, motionType, MaxAllowed, Polygon, polygonSize, timeLimit);
	return 1;
		
}

  
int BauotechAlgoConnector_PolygonClear(uint32_t videoIndex, uint32_t polygonId)
{
	std::string logMSG = "API..._AddPolygon(); polyID = " + std::to_string(polygonId) + " cam = " + std::to_string(videoIndex);
	LOGGER::log(DLEVEL::_SYSTEM_INFO, logMSG.c_str());
	g_algoProcess[videoIndex].polygonClear(polygonId);
	return 1;
}

API_EXPORT int BauotechAlgoConnector_Config(uint32_t videoIndex,
											BAUOTECH_AND_BENNY_KAROV_ALGO algo,
											uint32_t width,
											uint32_t height,
											uint32_t pixelWidth,
											uint32_t image_size,
											uint8_t youDraw,
											uint8_t invertImage,
											CameraAICallback callback)
											
{
	if (videoIndex >= MAX_VIDEOS) return 0;

	g_loadBalancer.initCamera(videoIndex); 

	// Init Queue 
	int bufSize = 3;
	g_bufQ[(int)videoIndex].set(width, height, pixelWidth, bufSize);

	// Init Algo thread
	if (!g_algoProcess[videoIndex].init(videoIndex, width, height, image_size, pixelWidth, (int)invertImage))
		return -1;

	// init callback function 
	g_algoProcess[videoIndex].setCallback(callback);
	g_algoProcess[videoIndex].setDrawFlag(int(youDraw));

	// Run Algo thread
	g_algoProcess[videoIndex].run(&g_bufQ[(int)videoIndex], &g_loadBalancer);


	std::string logMSG = "BauotechAlgoConnector_Config() cam=" + std::to_string(videoIndex);
	LOGGER::log(DLEVEL::_SYSTEM_INFO, logMSG.c_str());

	g_activeVideos.push_back(videoIndex);

	return 1;
}
 
API_EXPORT int BauotechAlgoConnector_Config_sync(uint32_t videoIndex,
	BAUOTECH_AND_BENNY_KAROV_ALGO algo,
	uint32_t width,
	uint32_t height,
	uint32_t pixelWidth,
	uint32_t image_size,
	uint8_t youDraw,
	uint8_t invertImage,
	CameraAICallback callback)	 
{
	if (videoIndex >= MAX_VIDEOS) return 0;


	if (!g_algoProcess[videoIndex].init(videoIndex, width, height, image_size, pixelWidth, invertImage))
		return -1;

	g_activeVideos.push_back(videoIndex);

	std::string logMSG = "BauotechAlgoConnector_Config_sync() cam=" + std::to_string(videoIndex);
	LOGGER::log(DLEVEL::_SYSTEM_INFO, logMSG.c_str());

	return 1;
}


API_EXPORT void BauotechAlgoConnector_SetCameraRequestCallback(CameraRequestCallback callback)
{
	LOGGER::log(DLEVEL::INFO1, "BauotechAlgoConnector_SetCameraRequestCallback()");
	g_loadBalancer.SetCameraRequestCallback(callback);
}

API_EXPORT void BauotechAlgoConnector_SetCameraType(uint32_t videoIndex, uint32_t type)
{
	if (videoIndex >= MAX_VIDEOS) return;

	LOGGER::log(DLEVEL::_SYSTEM_INFO, std::string("BauotechAlgoConnector_SetCameraType" + std::to_string(videoIndex)));
	g_loadBalancer.SetCameraType(videoIndex, type);
}

/*------------------------------------------------------------------------------------------------------------------
* OPtimization : remove mutex lock whiile console API 'GetbjectData()' is active 
------------------------------------------------------------------------------------------------------------------*/
API_EXPORT void BauotechAlgoConnector_setConsoleAPI(uint32_t videoIndex, uint8_t swichON)
{
	if (videoIndex >= MAX_VIDEOS) return;
	g_algoProcess[videoIndex].setConsoleAppAPI(swichON);
}

API_EXPORT void BauotechAlgoConnector_setMinPersonDim(uint32_t videoIndex, uint32_t  minheight)
{
	g_algoProcess[videoIndex].setMinPersonDim((int)minheight);
}


API_EXPORT int BauotechAlgoConnector_addFalseImg(uint32_t videoIndex, uint8_t* pData, uint32_t width, uint32_t height, uint32_t pixelWidth, uint32_t label)
{
	if (videoIndex >= MAX_VIDEOS) return 0;
	g_algoProcess[videoIndex].addFalseImg((char*)pData, width, height, pixelWidth, Labels(label));
	return 1;
}