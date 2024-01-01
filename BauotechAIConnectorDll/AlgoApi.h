#pragma once 
#ifndef ALGOAPI_HEADER
#define ALGOAPI_HEADER

#include <Windows.h>
#include <stdint.h>


#define MAX_VIDEOS     10

#pragma pack(push, 1) 
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
	int reserved2;
} ALGO_DETECTION_OBJECT_DATA;
#pragma pack(pop) 


// Define the callback function type
typedef void (*CameraRequestCallback)(const uint32_t camera[] , int size);
// CameraRequestCallback gCameraRequestCallback = nullptr;

typedef void (__cdecl *CameraAICallback)(uint32_t videoIndex, ALGO_DETECTION_OBJECT_DATA* pObjects, uint32_t objectCount, uint8_t* pData, int size);


typedef enum BAUOTECH_AND_BENNY_KAROV_ALGO
{
	ALGO_SHOVAL = 2023,
	ALGO_RAMI_LEVI = 2001,
	ALGO_ZOSMAN = 2002,
	ALGO_DEFAULT = 2003,
	ALGO_EMBOSS = 2004,
	ALGO_GREY = 2005,
	ALGO_BLUR = 2006,
	ALGO_POSTERIZE = 2007,
	ALGO_XOR = 2008,
	ALGO_DARKEN = 2009,
	ALGO_BLUE = 2010,
	ALGO_GREEN = 2011,
	ALGO_RED = 2012,
	BLACK_WHITE = 2013

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


	API_EXPORT int BauotechAlgoConnector_Run(BAUOTECH_AND_BENNY_KAROV_ALGO algo,
		uint8_t *pData,
		uint32_t width,
		uint32_t height,
		uint32_t pixelWidth,
		uint32_t image_size,
		uint8_t youDraw,
		ALGO_DETECTION_OBJECT_DATA* pObjects,
		uint32_t *objectCount);

	//API_EXPORT void BauotechAlgoConnector_GetAlgoObjectData(uint32_t videoIndex, int index, ALGO_DETECTION_OBJECT_DATA* pObjects);
	API_EXPORT int BauotechAlgoConnector_GetAlgoObjectData(uint32_t videoIndex, int frameNum, ALGO_DETECTION_OBJECT_DATA* pObjects);

	API_EXPORT int BauotechAlgoConnector_Run2(BAUOTECH_AND_BENNY_KAROV_ALGO algo,
										     uint8_t* pData,
										     uint32_t width,
										     uint32_t height,
											 uint32_t pixelWidth,
  											 uint32_t image_size,
											 uint8_t youDraw,
											 uint32_t* objectCount);


 
	 
	API_EXPORT int BauotechAlgoConnector_Config(uint32_t videoIndex,
											    BAUOTECH_AND_BENNY_KAROV_ALGO algo,		
											    uint32_t width,
											    uint32_t height,
											    uint32_t pixelWidth,
											    uint32_t image_size,
											    uint8_t youDraw,
												CameraAICallback callback);
										  
	API_EXPORT int BauotechAlgoConnector_Run3(uint32_t videoIndex, uint8_t* pData, uint32_t frameNumber );


	API_EXPORT void BauotechAlgoConnector_Init();
	API_EXPORT void BauotechAlgoConnector_Release();

 

	API_EXPORT void AlgoSetTime(int hour, int min, int sec);

	 


#ifdef __cplusplus
}
#endif

#endif // ALGOAPI_HEADER


