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
//#include "logger.hpp"
#include "CPPLogger.h"


#include "loadBalancer.hpp"




typedef struct tagRGB32TRIPLE {
	BYTE    rgbtBlue;
	BYTE    rgbtGreen;
	BYTE    rgbtRed;
	BYTE    rgb0;
} RGB32TRIPLE;




//--------------------------------------------------------------------------------------------------------
//  G L O B A L S   !


// Define a global critical section
CRITICAL_SECTION gCriticalSection;
bool m_initialize = false;
//CameraRequestCallback gCameraRequestCallback = nullptr; // moved from header file

std::unordered_map<uint32_t, CameraAICallback> gAICllbacks;

CLoadBalaner  g_loadBalancer;
//CSemaphore  g_ResourceSemaphore;
CAlgoProcess   g_algoProcess[MAX_VIDEOS];
TSBuffQueue g_bufQ[MAX_VIDEOS];

AutoResetNumericEvent cameraRes[MAX_VIDEOS];

//--------------------------------------------------------------------------------------------------------




API_EXPORT void AlgoSetTime(int hour, int min, int sec)
{

}
 
API_EXPORT void BauotechAlgoConnector_Init(bool loadBalance)
{
	InitializeCriticalSection(&gCriticalSection);

	CPPLogger::getLog().setLogLevel(DEBUG);
	CPPLogger::getLog().setLogModeFile("/tmp/testlog.log");
	std::cout << "All Logs\n";
	LOG(DEBUG) << "Log 1";
	LOG(INFO) << "Log 2";

	
	if (0)   // DDEBUG TEST LOAD BALANCER	
		g_loadBalancer.test_async();

	if (loadBalance)
		g_loadBalancer.init();

	m_initialize = true;
}

API_EXPORT void BauotechAlgoConnector_Release()
{
	if (m_initialize == true)
	{
		for (int i = 0; i < MAX_VIDEOS; i++) {
			g_algoProcess[i].terminate();
		}

		DeleteCriticalSection(&gCriticalSection);
		m_initialize = false;
	}
}

API_EXPORT void BauotechAlgoConnector_Release(int videoindex)
{
	if (m_initialize == true)
	{
			g_algoProcess[videoindex].terminate();
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

	if (!g_loadBalancer.acquire(videoIndex))
		return 0;

	bool ok = g_bufQ[(int)videoIndex].push(CframeBuffer(frameNumber, (char*)pData));
	g_algoProcess[videoIndex].WakeUp();

	return ok ? 1 : 0;

}

/*-------------------------------------------------------------------------------------------------------------------
*  runner use the TSBuffQeue buffering
 -------------------------------------------------------------------------------------------------------------------*/
API_EXPORT int BauotechAlgoConnector_Run4(uint32_t videoIndex, uint8_t* pData, uint64_t frameNumber)
{

	if (!g_loadBalancer.acquire(videoIndex))
		return 0;

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
											CameraAICallback callback)
											
{

	// LOAD BALANCER
	//----------------
	//g_loadBalancer.set(videoIndex, 0); 
	g_loadBalancer.initCamera(videoIndex); 

	// Init Queue 
	int bufSize = 3;
	g_bufQ[(int)videoIndex].set(width, height, pixelWidth, bufSize);
	// Init Algo thread
	if (!g_algoProcess[videoIndex].init(videoIndex, width, height, image_size, pixelWidth))
			return -1;


	// init callback function 
	g_algoProcess[videoIndex].setCallback(callback);
	g_algoProcess[videoIndex].setDrawFlag(int(youDraw));

	// Run Algo thread
	g_algoProcess[videoIndex].run(&g_bufQ[(int)videoIndex], &g_loadBalancer);
	//g_algoProcess[videoIndex].run(&g_bufQ[(int)videoIndex], &cameraRes[(int)videoIndex]);
	return 1;
}
 
API_EXPORT int BauotechAlgoConnector_Config_sync(uint32_t videoIndex,
	BAUOTECH_AND_BENNY_KAROV_ALGO algo,
	uint32_t width,
	uint32_t height,
	uint32_t pixelWidth,
	uint32_t image_size,
	uint8_t youDraw,
	CameraAICallback callback)	 
{

	int bufSize = 10;

	// Init Queue 
	if (!g_algoProcess[videoIndex].init(videoIndex, width, height, image_size, pixelWidth))
		return -1;

	return 1;
}


API_EXPORT void BauotechAlgoConnector_SetCameraRequestCallback(CameraRequestCallback callback)
{
	//gCameraRequestCallback = callback;
	g_loadBalancer.SetCameraRequestCallback(callback);
}

API_EXPORT void BauotechAlgoConnector_SetCameraType(uint32_t videoIndex, uint32_t type)
{
	g_loadBalancer.SetCameraType(videoIndex, type);
}


