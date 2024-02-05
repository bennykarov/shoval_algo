#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 
#include "opencv2/opencv.hpp"


#include "AlgoApi.h"
#include "config.hpp"
#include "utils.hpp"
#include "AlgoDetection.hpp"

#include "algoProcess.hpp"

//typedef std::chrono::duration<float, std::milli> duration;

float 	g_elapsedSum = 0;
float   g_elapsedMin = 99999;
float   g_elapsedMax = 0;

/*===========================================================================================
* AlgoProcess (thread) class
  ===========================================================================================*/
	bool algoProcess::init(int video_index, int width, int height, int image_size, int pixelWidth, char *cameraConfig)
	{
		//gAICllbacks[videoIndex](videoIndex, aidata, 1, nullptr, 0);
		m_videoIndex = video_index;
		m_width = width;
		m_height = height;
		m_pixelWidth = pixelWidth;
		m_scale = 0.5;


		int badImageSize = imageSize();

		bool ok = m_tracker.init(video_index, m_width, m_height, image_size, m_pixelWidth, cameraConfig, m_scale/*0.5*/);

		m_timer.start();
		return ok;
	}

	void algoProcess::setCallback(CameraAICallback callback)
	{
		m_callback = callback;
	}

	void algoProcess::WakeUp()
	{
		m_event.Set();
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

		std::cout << "GPU warmup for one second... \n";
		Sleep(1000); // DDEBBUG let CUDA and GPU init

		return m_thread.joinable();
	}



	/*----------------------------------------------------------------------------------------------------
	* Process thread (main loop) : Fetching frame from the queue , process frame and send to callback
	* Run the m_tracker.process() and fills the 'm_Objects' objects 
	  ---------------------------------------------------------------------------------------------------*/
	int algoProcess::run_th(TSBuffQueue* bufQ)
	{
		CframeBuffer frameBuff;
		long frameCount = 0;
		float elapsedSum = 0;
		float elapsedMin = 999999;
		float elapsedMax = 0;

		while (m_terminate == false) 
		{
			if (bufQ->front(frameBuff) == false)
			{
				Sleep(10);
				continue;
			}
			break;				
		}

		while (!m_terminate) 
		{
			if (bufQ->IsEmpty())
			{
				//std::cout << "< before m_event.Wait()";
				m_event.Wait();
				//std::cout << "> after m_event.Wait()";
			}
			if (bufQ->pop(frameBuff) == false)
			{
				continue;
			}
			//std::cout << "algoProcess::run_th: Wakeup" << frameBuff.frameNum << "\n";
			
			if (1)
				if (m_frameNum == frameBuff.frameNum)
				{
					//>>std::cout << "m_frameNum == frameBuff.frameNum  \n";
					Beep(1200, 10); // Sleep(5);
					//bufQ->pop(); // release buffer
					continue;
				}

			m_frameNum = frameBuff.frameNum;


			m_timer.sample();

			// m_objectCount = m_tracker.process((void*)frameBuff.ptr, m_Objects);
			cv::Mat frameBGR = converPTR2MAT(frameBuff.ptr, m_height, m_width, m_pixelWidth);
			m_objectCount = m_tracker.process(frameBGR, m_Objects);

			float elapsed = m_timer.sample();


			if (m_frameNum % 60 == 0)
				std::cout << "Algo duration: " << elapsed << " milliseconds" << std::endl;

			


			// DDEBUG : for getbjectData() API  
			//---------------------------------------
			bool supportGetbjectData = true; // DDEBUG
			if (supportGetbjectData) {
 				std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);
				m_pObjectsAPI.clear();
				for (int i = 0; i < m_objectCount; i++)
					m_pObjectsAPI.push_back(m_Objects[i]); 
			}
			 

			int i = 0;
			for (;i < m_objectCount; i++)
				m_Objects[i].frameNum = m_frameNum;
			for (;i < MAX_OBJECTS; i++)
				m_Objects[i].frameNum = -1;


			// send the data to the callback
			if (m_callback != nullptr && m_objectCount > 0)
			{
				if (m_objectCount > 2)
					int debug = 10;
				//std::cout << "Find " << m_objectCount << " objects \n";

				m_callback(m_videoIndex, &m_Objects[0], m_objectCount, nullptr, 0);  //gAICllbacks[m_videoIndex](m_videoIndex, m_pObjects, m_objectCount, nullptr, 0);
			}

			frameCount++;
		} // while !terminate


		// termination()...
		//Sleep(20); // DDEBUG DDEBUG 

		return m_frameNum > 0;

	}

	int algoProcess::run_sync(void *pData, int frameNum, ALGO_DETECTION_OBJECT_DATA *AIobjects)
	{
		CframeBuffer frameBuff;
		frameBuff.ptr = (char*)pData;
		frameBuff.frameNum = frameNum;

		long frameCount = 0;


		g_elapsedSum = 0;
		g_elapsedMin = 10000;
		g_elapsedMax = 0;

		m_frameNum = frameBuff.frameNum;


		m_timer.start();

		//m_objectCount = m_tracker.process((void*)frameBuff.ptr, m_Objects);
		cv::Mat frameBGR = converPTR2MAT(frameBuff.ptr, m_height, m_width, m_pixelWidth);
		m_objectCount = m_tracker.process(frameBGR, m_Objects);

		//auto end = std::chrono::system_clock::now();
		m_timer.sample();

		for (int i=0;i< m_objectCount;i++)
			AIobjects[i] = m_Objects[i];


		frameCount++;

		if (frameCount % 30 == 0) {
			std::cout << " FPS = " << 1000. / (g_elapsedSum / 30.) << " ( min / max = " << (1000. / g_elapsedMin) << " , " << (1000. / g_elapsedMax) << "\n";
			g_elapsedSum = 0;
			g_elapsedMax = 0;
			g_elapsedMin = 99999;

			frameCount = 0;
		}

		return m_objectCount;

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
	* index = -1 means return all valid objects (m_objectCount)
	 ---------------------------------------------------------------------------------------------------*/
	int algoProcess::getObjectData(int videoIndex, int index, ALGO_DETECTION_OBJECT_DATA* pObjects, int& frameNum)
	{
		if (index >= m_objectCount || m_objectCount == 0)
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
		WakeUp(); // release wait() of queue
		if (m_thread.joinable())
			m_thread.join();
		return true;
	}
