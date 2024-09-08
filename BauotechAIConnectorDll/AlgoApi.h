#pragma once 
#ifndef ALGOAPI_HEADER
#define ALGOAPI_HEADER

#include <Windows.h>
#include <stdint.h>


#define MAX_VIDEOS     101

#pragma pack(push, 1) 

enum _MotionType {
	_OnlyMoving = 0,
	_MotionOrNot = 1,
	_OnlyStatics = 2,
};


typedef struct ALGO_DETECTION_OBJECT_DATA
{
	int X;
	int Y;
	int Width;
	int Height;
	int CountUpTime;
	int ObjectType; // optional	
	int DetectionPercentage;// optional
	int frameNum;
	int ID;
	int polygonID;
} ALGO_DETECTION_OBJECT_DATA;
#pragma pack(pop) 


// Define the callback function type
typedef void (*CameraRequestCallback)(const uint32_t camera[] , int size);
// CameraRequestCallback gCameraRequestCallback = nullptr;

typedef void (__cdecl *CameraAICallback)(uint32_t videoIndex, ALGO_DETECTION_OBJECT_DATA* pObjects, uint32_t objectCount, uint8_t* pData, int size);


typedef enum BAUOTECH_AND_BENNY_KAROV_ALGO
{
	NO_ALGO = 0,
	ALGO_SHOVAL = 1,
	ALGO_RAMI_LEVI = 2,
	ALGO_ZOSMAN = 3,
	ALGO_DEFAULT = 4,

} BAUOTECH_AND_BENNY_KAROV_ALGO;
typedef struct VideoInfo
{

	BAUOTECH_AND_BENNY_KAROV_ALGO algo;
	uint32_t width;
	uint32_t height;
	uint32_t pixelWidth;
	uint32_t image_size;
	uint8_t youDraw;

} VideoInfo;
 

 

#ifdef __cplusplus
extern "C" {


	 

#define API_EXPORT __declspec(dllexport)
#endif
 
	 
	API_EXPORT void Crash();

	API_EXPORT int BauotechAlgoConnector_Config(uint32_t videoIndex,
		BAUOTECH_AND_BENNY_KAROV_ALGO algo,
		uint32_t width,
		uint32_t height,
		uint32_t pixelWidth,
		uint32_t image_size,
		uint8_t youDraw,
		uint8_t invertImage,
		CameraAICallback callback);
	 
	API_EXPORT int BauotechAlgoConnector_AddPolygon(uint32_t videoIndex,	 
													int CamID,
													int polygonId,
													char* DetectionType,
													int MaxAllowed,
													int Polygon[],
													int polygonSize,
													int motionType=0);




	API_EXPORT int BauotechAlgoConnector_PolygonClear(uint32_t videoIndex);
	


	API_EXPORT int BauotechAlgoConnector_Config_sync(uint32_t videoIndex,
		BAUOTECH_AND_BENNY_KAROV_ALGO algo,
		uint32_t width,
		uint32_t height,
		uint32_t pixelWidth,
		uint32_t image_size,
		uint8_t youDraw,
		uint8_t invertImage,
		CameraAICallback callback);

	API_EXPORT int BauotechAlgoConnector_Run3(uint32_t videoIndex, uint8_t* pData, uint64_t frameNumber);
	API_EXPORT int BauotechAlgoConnector_Run3_sync(uint32_t videoIndex, uint8_t* pData, ALGO_DETECTION_OBJECT_DATA* AIObjects, uint64_t frameNumber);

	API_EXPORT int BauotechAlgoConnector_GetAlgoObjectData(uint32_t videoIndex, int index, ALGO_DETECTION_OBJECT_DATA* pObjects);


	API_EXPORT void BauotechAlgoConnector_Init();
	API_EXPORT void BauotechAlgoConnector_Release();
	API_EXPORT void BauotechAlgoConnector_Remove(int videoindex);

 
	API_EXPORT void BauotechAlgoConnector_SetCameraRequestCallback(CameraRequestCallback callback);
	API_EXPORT void BauotechAlgoConnector_SetCameraType(uint32_t videoIndex, uint32_t type);

	API_EXPORT void AlgoSetTime(int hour, int min, int sec);

	 


#ifdef __cplusplus
}
#endif

#endif // ALGOAPI_HEADER


