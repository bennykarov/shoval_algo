//#include "AlgoApi.h"
//#include <winver.h>
//#include <windows.h>
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


const std::string VERSION_NUM = "1.3.0";



std::string GetDllVersion(const std::string& dllPath) {
	// Convert the file path to a wide string
	std::wstring widePath(dllPath.begin(), dllPath.end());

	// Get the size of the version information
	DWORD dummy;
	DWORD versionInfoSize = GetFileVersionInfoSizeW(widePath.c_str(), &dummy);
	if (versionInfoSize == 0) {
		std::cerr << "Error getting version info size: " << GetLastError() << std::endl;
		return "";
	}

	// Allocate memory for version information
	std::vector<BYTE> versionInfo(versionInfoSize);
	if (!GetFileVersionInfoW(widePath.c_str(), 0, versionInfoSize, versionInfo.data())) {
		std::cerr << "Error getting version info: " << GetLastError() << std::endl;
		return "";
	}

	// Get the root block of the version information
	VS_FIXEDFILEINFO* fixedFileInfo;
	UINT size;
	if (!VerQueryValueW(versionInfo.data(), L"\\", reinterpret_cast<LPVOID*>(&fixedFileInfo), &size)) {
		std::cerr << "Error querying version info: " << GetLastError() << std::endl;
		return "";
	}

	// Extract version information
	DWORD major = HIWORD(fixedFileInfo->dwFileVersionMS);
	DWORD minor = LOWORD(fixedFileInfo->dwFileVersionMS);
	DWORD build = HIWORD(fixedFileInfo->dwFileVersionLS);
	DWORD revision = LOWORD(fixedFileInfo->dwFileVersionLS);

	// Format the version string
	return std::to_string(major) + "." + std::to_string(minor) + "." +
		std::to_string(build) + "." + std::to_string(revision);
}

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
 
API_EXPORT int BauotechAlgoConnector_Init(bool runloadBalancer)
{

	if (m_initialize) // DDEBUG avoid duplicate call !!!! 
		return 0;

	InitializeCriticalSection(&gCriticalSection);

		
	int debugLevel = 1;
	if (debugLevel > 0) {
		//LOGGER::init(FILES::OUTPUT_FOLDER_NAME, DLEVEL::INFO2);
		LOGGER::init(FILES::OUTPUT_FOLDER_NAME, DLEVEL::DEBUG_HIGH);
		std::string dllFilePath = "C:\\Program Files\\Bauotech\\dll\\algo\\BauotechAlgoConnector.dll";
		LOGGER::log(DLEVEL::DEBUG_HIGH, "BauotechAlgoConnector DLL version = " + GetDllVersion(dllFilePath));
		
	}
	LOGGER::log(DLEVEL::INFO1, "BauotechAlgoConnector_Init()");

	uint32_t batchSize; // Eli changes - right now the 'batchSize' init inside g_loadBalancer.init() from config 
	if (runloadBalancer == true)
		g_loadBalancer.init(&batchSize);
	else
		LOGGER::log(DLEVEL::DEBUG_HIGH, " RUNNIG  W I T H O U T  LOAD BALANCER  !!!!!");

	m_initialize = true;


	return (int)batchSize;
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
API_EXPORT int BauotechAlgoConnector_Run3(uint32_t videoIndex, uint8_t* pData, uint64_t frameNumber, int64_t timestamp)
{
	bool forceList = true; // ignore input not in Loadbalncer cameraList

	// Load balancer Aquire - quit if not allowed (if allowOverflow = true)	
	auto error = g_loadBalancer.acquire(videoIndex);
	if (error != AQUIRE_ERROR::NOT_ACTIVE && error != AQUIRE_ERROR::OK)
	{
		LOGGER::log(DLEVEL::DEBUG_HIGH, "LB Ignors cam #" + std::to_string(videoIndex));
		if (forceList)
			return false;	
	}

	// Push buffer 
	bool ok = g_bufQ[(int)videoIndex].push(CframeBuffer(frameNumber, timestamp, (char*)pData));
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
	int polygonSize,
	int motionType)
{
	std::string logMSG = "API..._AddPolygon() cam=" + std::to_string(videoIndex) + "polygonPoints = " + std::to_string(polygonSize);
	LOGGER::log(DLEVEL::INFO2, logMSG.c_str());

	g_algoProcess[videoIndex].addPolygon(CamID, polygonId, DetectionType, motionType, MaxAllowed, Polygon, polygonSize);
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

/*------------------------------------------------------------------------------------------------------------------
* OPtimization : remove mutex lock whiile console API 'GetbjectData()' is active 
------------------------------------------------------------------------------------------------------------------*/
API_EXPORT void BauotechAlgoConnector_setConsoleAPI(uint32_t videoIndex, uint8_t swichON)
{
	g_algoProcess[videoIndex].setConsoleAppAPI(swichON);

}

