#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 
#include "AutoResetEvent.h"
#include "queuing.hpp"
#include "utils.hpp"
#include "timer.hpp"
#include "config.hpp"


/*===========================================================================================
* AlgoProcess (thread) class
  ===========================================================================================*/
class algoProcess {
public:
	bool init(int video_index, int width, int height, int image_size, int pixelWidth, char * cameraConfig);
	bool terminate();
	void setCallback(CameraAICallback callback);
	void setDrawFlag(int youDraw) { m_youDraw = youDraw; }

	int getObjectData(int videoIndex, int index, ALGO_DETECTION_OBJECT_DATA *pObjects, int &frameNum);


	int imageSize();
	/*---------------------------------------------------------------------
	* Process thread: Fetching frame from the queue , process frame and send to callback
	  ---------------------------------------------------------------------*/
	int run(TSBuffQueue* bufQ);
	int run_th(TSBuffQueue* bufQ);
	int run_sync(void* pData, int frameNum, ALGO_DETECTION_OBJECT_DATA* AIobjects);
	// getters

	void WakeUp();

private:
	std::thread m_thread;
	AutoResetEvent m_event;
	CameraAICallback m_callback;
	CDetector m_tracker;

	ALGO_DETECTION_OBJECT_DATA m_Objects[MAX_OBJECTS];
	std::vector <ALGO_DETECTION_OBJECT_DATA> m_pObjectsAPI; // for debug API

	std::mutex m_BufferMutex;

	bool m_terminate = false;
	int m_frameNum = -1;
	int m_videoIndex;

	int m_width, m_height, m_imageSize, m_pixelWidth;
	float m_scale;


	int m_objectCount = 0;
	int m_youDraw = 0;

	void fakeCallBack(int m_videoIndex, ALGO_DETECTION_OBJECT_DATA* m_pObjects, int m_objectCount, void* ptr, int something);

	CTimer m_timer;
};
