//#include "AlgoApi.h"
#include <unordered_map>
#include <string>
#include <fstream> // debug file

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



//std::atomic<bool>  debugFirstCameraConfig = true;



//--------------------------------------------------------------------------------------------------------
//  G L O B A L S   !


// Define a global critical section
CRITICAL_SECTION gCriticalSection;
bool m_initialize = false;

CameraRequestCallback gCameraRequestCallback = nullptr; // moved from header file
std::unordered_map<uint32_t, CameraAICallback> gAICllbacks;

CLoadBalaner  g_loadBalancer;
//CSemaphore  g_ResourceSemaphore;
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
 
API_EXPORT void BauotechAlgoConnector_Init()
{

	if (m_initialize) // DDEBUG avoid duplicate call !!!! 
		return;

	InitializeCriticalSection(&gCriticalSection);

		


	bool runloadBalancer = true; // DDEBUG FLAG


	if (runloadBalancer)
		g_loadBalancer.init();
	else
		LOGGER::log(DLEVEL::WARNING1, " RUNNIG  W I T H O U T  LOAD BALANCER  !!!!!");

	int debugLevel = 1;
	if (debugLevel > 0)
		LOGGER::init(FILES::OUTPUT_FOLDER_NAME, DLEVEL::ALL);

	LOGGER::log(DLEVEL::WARNING2, "BauotechAlgoConnector_Init()");


	m_initialize = true;
}

API_EXPORT void BauotechAlgoConnector_Release()
{
	if (m_initialize == true)
	{
		LOGGER::log(DLEVEL::INFO1, "call to BauotechAlgoConnector_Release()");
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

API_EXPORT void BauotechAlgoConnector_Remove(int videoindex)
{
	if (m_initialize == true)
	{
		g_loadBalancer.remove(videoindex);
		g_algoProcess[videoindex].terminate();
		
		LOGGER::log(DLEVEL::INFO1, "BauotechAlgoConnector_Rmove() for cam# = "+std::to_string(videoindex));
		
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
API_EXPORT int BauotechAlgoConnector_Run3(uint32_t videoIndex, uint8_t* pData, uint64_t frameNumber)
{
	

	// Load balancer Aquire - quit if not allowed (if allowOverflow = true)
	bool allowOverflow = false;
	
	auto error = g_loadBalancer.acquire(videoIndex, allowOverflow);
	if (error != AQUIRE_ERROR::NOT_ACTIVE && error != AQUIRE_ERROR::OK)
	{
		std::string errorMsg = "LB Ignoring cam #" + std::to_string(videoIndex) + " error = " + std::to_string(error) + "\n";
		LOGGER::log(DLEVEL::WARNING2, errorMsg);
		//std::cout << "LB Ignoring cam #" << videoIndex << " error =" << error << "\n";
		if (!allowOverflow) 
			return false;	
	}

	// Push buffer 
	bool ok = g_bufQ[(int)videoIndex].push(CframeBuffer(frameNumber, (char*)pData));
	g_algoProcess[videoIndex].WakeUp();

	return ok ? 1 : 0;
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
	int polygonSize)
{
	std::string logMSG = "API..._AddPolygon() cam=" + std::to_string(videoIndex) + "polygonPoints = " + std::to_string(polygonSize);
	LOGGER::log(DLEVEL::INFO2, logMSG.c_str());

	g_algoProcess[videoIndex].addPolygon(CamID, polygonId, DetectionType, MaxAllowed, Polygon, polygonSize);
	return 1;
		
}

  
int BauotechAlgoConnector_PolygonClear(uint32_t videoIndex)
{
	g_algoProcess[videoIndex].polygonClear();
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
	LOGGER::log(DLEVEL::INFO1, logMSG.c_str());

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

	// Init Queue 
	if (!g_algoProcess[videoIndex].init(videoIndex, width, height, image_size, pixelWidth, invertImage))
		return -1;

	return 1;
}


API_EXPORT void BauotechAlgoConnector_SetCameraRequestCallback(CameraRequestCallback callback)
{
	LOGGER::log(DLEVEL::INFO1, "BauotechAlgoConnector_SetCameraRequestCallback()");

	//gCameraRequestCallback = callback;
	g_loadBalancer.SetCameraRequestCallback(callback);
}

API_EXPORT void BauotechAlgoConnector_SetCameraType(uint32_t videoIndex, uint32_t type)
{
	LOGGER::log(DLEVEL::INFO1, std::string("BauotechAlgoConnector_SetCameraType" + std::to_string(videoIndex)));
	
	g_loadBalancer.SetCameraType(videoIndex, type);
}


