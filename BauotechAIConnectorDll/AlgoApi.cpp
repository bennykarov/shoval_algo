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


//const int MaxVideos = 10;
CDetector g_tracker;
TSBuffQueue g_bufQ;

// GLOBALS !
std::ofstream g_debugFile; 
bool printTime = false;
//boost::thread	g_algo_Th;
std::thread	g_algo_Th;


// Define a global critical section
CRITICAL_SECTION gCriticalSection;
bool m_initialize = false;
CameraRequestCallback gCameraRequestCallback = nullptr; // moved from header file

std::unordered_map<uint32_t, CameraAICallback> gAICllbacks;


typedef struct tagRGB32TRIPLE {
	BYTE    rgbtBlue;
	BYTE    rgbtGreen;
	BYTE    rgbtRed;
	BYTE    rgb0;
} RGB32TRIPLE;


//std::unordered_map<uint32_t, ALGO_DETECTION_OBJECT_DATA> gAlgoObjects;
//std::unordered_map <uint32_t , VideoInfo>  gVideoInfo;

// GLOBAL :
algoProcess   g_algoProcess[MAX_VIDEOS] ;



/*===========================================================================================*/
API_EXPORT void AlgoSetTime(int hour, int min, int sec)
{

}
 
API_EXPORT void BauotechAlgoConnector_Init()
{
	InitializeCriticalSection(&gCriticalSection);

	m_initialize = true;
}

API_EXPORT void BauotechAlgoConnector_Release()
{
	if (m_initialize == true)
	{
		for (int i = 0; i < MAX_VIDEOS; i++)
			g_algoProcess[i].terminate();

		DeleteCriticalSection(&gCriticalSection);
		m_initialize = false;
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

	//memcpy(pObjects, &gAlgoObjects[videoIndex], sizeof(ALGO_DETECTION_OBJECT_DATA));
}

API_EXPORT int BauotechAlgoConnector_GetAlgoObjectData(uint32_t videoIndex, ALGO_DETECTION_OBJECT_DATA* pObjects, uint32_t* objectCount)
{
	int frameNum;
	return g_algoProcess[videoIndex].getObjectData(videoIndex, -1, pObjects, frameNum);
	//memcpy(pObjects, &gAlgoObjects[videoIndex], sizeof(ALGO_DETECTION_OBJECT_DATA));
}



/*-------------------------------------------------------------------------------------------------------------------
*  runner use the TSBuffQeue buffering 
 -------------------------------------------------------------------------------------------------------------------*/
API_EXPORT int BauotechAlgoConnector_Run3(uint32_t videoIndex, uint8_t* pData, uint32_t frameNumber)
{
	// take the video data here and leave immidiatly 
	 
	// benny will take the pData and index to his buffers - and release function 
	// fifo_push(pdata, index);

	bool ok = g_bufQ.push(CframeBuffer(frameNumber, (char*)pData));

	return ok ? 1 : 0;

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

	int bufSize = 10;

	// if (1) pixelWidth = 4;// DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG DDEBUG for RGBA 
		
	// Init Queue 
	g_bufQ.set(width, height, pixelWidth, bufSize);
	// Init Algo thread
	if (!g_algoProcess[videoIndex].init(videoIndex, width, height, image_size, pixelWidth))
		return -1;
	// init callback function 
	g_algoProcess[videoIndex].setCallback(callback);
	g_algoProcess[videoIndex].setDrawFlag(int(youDraw));

	// Run Algo thread
	g_algoProcess[videoIndex].run(&g_bufQ);
	return 1;
}
 
API_EXPORT void BauotechAlgoConnector_SetCameraRequestCallback(CameraRequestCallback callback)
{
	gCameraRequestCallback = callback;
}

 





// AI Code

/*
thread{
     
	 processIndex1()
	 {
	    detection process:
	     // example:
		ALGO_DETECTION_OBJECT_DATA aidata[10];
		aidata[0].X = 200;
		aidata[0].Y = 200;
		aidata[0].Width = 500;
		aidata[0].Height = 400;
		//.....
		// CameraAICallback(uint32_t videoIndex, ALGO_DETECTION_OBJECT_DATA* pObjects, uint32_t objectCount, uint8_t* pData, int size);

		uint32_t videoIndex = 0;
		ALGO_DETECTION_OBJECT_DATA* aidata;
		uint32_t objectCount = aidata, uint8_t* pData, int size
		gAICllbacks[videoIndex](videoIndex, aidata, 3, nullptr, 0);
	 }


	 // For stage II - now ALGO gets only first camera by default
	 //---------------------------------------------------------------
	// example - call it from different place
	uint32_t cam[4] = { 1,0,0,4 };
	gCameraRequestCallback(cam, 4);

}

*/





