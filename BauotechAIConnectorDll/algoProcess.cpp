#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 

#include "opencv2/opencv.hpp"


#include "AlgoApi.h"
#include "config.hpp"

#include "AlgoDetection.hpp"

#include "algoProcess.hpp"


/*===========================================================================================
* AlgoProcess (thread) class
  ===========================================================================================*/
	bool algoProcess::init(int video_index, int width, int height, int image_size, int pixelWidth)
	{
		//gAICllbacks[videoIndex](videoIndex, aidata, 1, nullptr, 0);
		m_videoIndex = video_index;
		m_width = width;
		m_height = height;
		m_pixelWidth = pixelWidth;
		m_scale = 0.5;


		int badImageSize = imageSize();

		bool ok = m_tracker.init(m_width, m_height, image_size, m_pixelWidth, m_scale/*0.5*/);
		return ok;
	}

	void algoProcess::setCallback(CameraAICallback callback)
	{
		m_callback = callback;
	}

	int algoProcess::imageSize()
	{
		return m_width * m_height * m_pixelWidth; // / 8;
	}

	/*---------------------------------------------------------------------
	* Start the algo thread
	 ---------------------------------------------------------------------*/
	int algoProcess::run(TSBuffQueue* bufQ)
	{
		m_thread = std::thread(&algoProcess::run_th, this, bufQ);

		return m_thread.joinable();
	}
	/*----------------------------------------------------------------------------------------------------
	* Process thread (main loop) : Fetching frame from the queue , process frame and send to callback
	  ---------------------------------------------------------------------------------------------------*/
	int algoProcess::run_th(TSBuffQueue* bufQ)
	{
		CframeBuffer frameBuff;

		while (!m_terminate) {
			frameBuff = bufQ->back();
			m_frameNum = frameBuff.frameNum;
			if (frameBuff.ptr == nullptr) {
				Sleep(20);
				continue;
			}

			m_objectCount = m_tracker.process((void*)frameBuff.ptr, m_Objects);

			if (1) // DDEBUG : for getbjectData() API
			{
				std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);
				m_pObjectsAPI.clear();
				for (int i = 0; i < m_objectCount; i++)
					m_pObjectsAPI.push_back(m_Objects[i]); // for debug API
			}


			bufQ->pop(); // release buffer
			//objectCount = pObjects->reserved1;

			int i = 0;
			for (;i < m_objectCount; i++)
				m_Objects[i].frameNum = frameBuff.frameNum;
			for (;i < MAX_OBJECTS; i++)
				m_Objects[i].frameNum = -1;


			//if (m_objectCount > 0)  gAlgoObjects[m_videoIndex] = m_Objects[0]; // only first object (for now)

			// send the data to the callback
			if (m_callback != nullptr && m_objectCount > 0)
			{
				m_callback(m_videoIndex, &m_Objects[0], m_objectCount, nullptr, 0);  //gAICllbacks[m_videoIndex](m_videoIndex, m_pObjects, m_objectCount, nullptr, 0);
			}
			//else // DDEBUG DDEBUG DDEBUG DDEBUG 
				//fakeCallBack(m_videoIndex, m_Objects, m_objectCount, nullptr, 0);
		}


		// termination()...
		return m_frameNum > 0;

	}


	// DDEBUG DDEBUG functions:
	void algoProcess::fakeCallBack(int m_videoIndex, ALGO_DETECTION_OBJECT_DATA* pObjects, int m_objectCount, void* ptr, int something)
	{
		if (m_objectCount)
			std::cout << "algoProcess::fakeCallBack: detected objects = " << m_objectCount << "\n";

		for (int i = 0; i < m_objectCount; i++)
			std::cout << "			>>	type = " << pObjects[i].ObjectType << "\n";
	}

	/*---------------------------------------------------------------------------------------------------
	* API function: getObjectData():
	* return number of objects provided 
	 ---------------------------------------------------------------------------------------------------*/
	int algoProcess::getObjectData(int videoIndex, int index, ALGO_DETECTION_OBJECT_DATA* pObjects, int& frameNum)
	{
		if (index >= m_objectCount && m_objectCount == 0)
			return 0;

		std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);

		if (index < 0) { // return all objects
			memcpy(pObjects, &m_Objects[0], sizeof(ALGO_DETECTION_OBJECT_DATA) * m_objectCount);
			frameNum = m_Objects[0].frameNum;
			return m_objectCount;
		}
		else {
			memcpy(pObjects, &m_Objects[index], sizeof(ALGO_DETECTION_OBJECT_DATA));
			frameNum = m_Objects[0].frameNum;
			return 1;
		}

	}


	bool algoProcess::terminate() 
	{ 
		m_terminate = true;
		if (m_thread.joinable())
			m_thread.join();
		return true;
	}
