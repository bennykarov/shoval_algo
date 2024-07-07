#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 
#include "AutoResetEvent.h"
#include "queuing.hpp"
#include "utils.hpp"
#include "timer.hpp"
#include "config.hpp"
#include "semaphore.hpp"
#include "loadBalancer.hpp"



/*===========================================================================================
* CAlgoProcess (thread) class
  ===========================================================================================*/
class CAlgoProcess {
public:
	~CAlgoProcess();
	bool init(int video_index, int width, int height, int image_size, int pixelWidth, int invertImage);
	//void setRousceSemaphore(CSemaphore *sema) {m_resourceSemaphore = sema;}
	void addPolygon(int CamID, int polygonId, char* DetectionType, int MaxAllowed, int Polygon[], int polygonSize);
	void polygonClear();
	void initPolygons();

	bool terminate();
	void setCallback(CameraAICallback callback);
	void setDrawFlag(int youDraw) { m_youDraw = youDraw; }

	int getObjectData(int videoIndex, int index, ALGO_DETECTION_OBJECT_DATA *pObjects, int &frameNum);


	int imageSize();
	/*---------------------------------------------------------------------
	* Process thread: Fetching frame from the queue , process frame and send to callback
	  ---------------------------------------------------------------------*/
	int run(TSBuffQueue* bufQ, CLoadBalaner* loader);
	//int run(TSBuffQueue* bufQ, AutoResetNumericEvent* camRes) { return 0; } // DDEBUG 
	int run_sync(void* pData, int frameNum, ALGO_DETECTION_OBJECT_DATA* AIobjects);
	int run_th2(TSBuffQueue* bufQ, CLoadBalaner* loader);

	/*
	int run(TSBuffQueue* bufQ);
	int run_th(TSBuffQueue* bufQ);
	//int run(TSBuffQueue* bufQ, );
	*/

	void WakeUp();

private:
	void makeVehicleInfo(std::vector<cv::Point> contour, int MaxAllowed, int polygonId);
	std::vector <int> addMultiPolygons(std::string DetectionTypeList);


private:
	bool m_active = false;
	std::thread m_thread;
	AutoResetEvent m_event;
	CameraAICallback m_callback;
	CDetector m_tracker;

	ALGO_DETECTION_OBJECT_DATA m_Objects[MAX_OBJECTS];
	std::vector <ALGO_DETECTION_OBJECT_DATA> m_pObjectsAPI; // for debug API

	std::mutex m_BufferMutex;

	
	std::atomic_bool m_terminate = false;
	int m_frameNum = -1;
	int m_videoIndex;

	int m_width = 0;
	int m_height = 0;
	int  m_imageSize, m_pixelWidth;
	float m_scale;


	int m_objectCount = 0;
	int m_youDraw = 0;

	void fakeCallBack(int m_videoIndex, ALGO_DETECTION_OBJECT_DATA* m_pObjects, int m_objectCount, void* ptr, int something);

	std::vector <CAlert> m_camerasInfo;
	CSemaphore *m_resourceSemaphore=nullptr;

	CTimer m_timer;

	std::queue<CCycle>         *m_loaderResQueuePtr; // loader queue for resource 
	CLoadBalaner* m_loader; // Load Balancer class pointer

};
