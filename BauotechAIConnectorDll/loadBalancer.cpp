#include <thread> 
#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 
#include <stdlib.h>     /* srand, rand */

#include "opencv2/opencv.hpp"

#include "logger.hpp"
#include "loadBalancer.hpp"


const int MaxCameras = 20;
const int Maxthreads = 5;


//CSemaphore m_resourceBouncer(Maxthreads);

#ifdef USE_LOAD_BALANCER


#ifdef _DEBUG
const int beatTick = 99;
#else
const int beatTick = 33;
#endif 

void CLoadBalaner::beatTimer()
{
	while (!m_terminate) {
		LOGGER::log(" BEAT()              ----->");
		beat();
		Sleep(beatTick);
	}
}

void CLoadBalaner::init(int reoucesLen, int camerasLen)
{
	m_timestamps.set_capacity(TIMESTAMP_LEN);

	m_reoucesLen = reoucesLen;
	m_camerasLen = camerasLen;

	for (int i = 0; i < m_camerasLen; i++)
		m_priorityQueue.push(i, 0);


	//g_ResourceSemaphore.set(reoucesLen);
	/*
	m_resources.assign(reoucesLen, CResource());
	m_waitingCams.assign(camerasLen, CWaitingElement());
	*/
}
//------------------------------------------------------------
// Try to lzunch frame 
// detection Check if :
// (1) Camera top priority (top queue)
// (2) Resource available
//------------------------------------------------------------

bool CLoadBalaner::try_acquire(int camID)
{
	if (m_priorityQueue.empty()) {
		std::cout << "priority-Queue is EMPTY! \n";
		return false;
	}

	//--------------------------------------------------------------------------
	// Start beat() after first acuire request was sent (first run3() calling 
	//--------------------------------------------------------------------------
	if (!m_startProcess) {
		m_startProcess = true;
		m_beatthread = std::thread(&CLoadBalaner::beatTimer, this);
	}

	auto prior = m_priorityQueue.get_priority2(camID);

	//if (m_priorityQueue.get_priority2(camID) == 0) m_priorityQueue.update(camID, 1);   // First time use   

	if (!m_priorityQueue.inTop(camID))
		return false;

	if (!m_resourceBouncer->take(camID))
		return false;

	//--------------------------------------------
	// Resource free - ready  to launch thread 
	//--------------------------------------------
	m_timestamps.push_back(std::chrono::system_clock::now());
	if (m_timestamps.size() > 1) {
		std::chrono::duration<float, std::milli> elapsed = m_timestamps.back() - m_lastBeat;

		double cycles_elapsed = elapsed.count() / CYCLE_MS;

		std::cout << "cycles_elapsed = " << (int)cycles_elapsed << "\n";

	}
	m_camInProcess.push_back(camID);

	// set to lowest priority.   (algo can change this priority later after intruders was found )
	m_priorityQueue.set(camID, 0); 

	return true;
}

/*-----------------------------------------------------------------------------------------
* Release resoure and set the new priority acording to detection (and other conditions)
 -----------------------------------------------------------------------------------------*/
int CLoadBalaner::release(int camID, int status)
{
	// release semaphore
	//g_ResourceSemaphore.release(camID);
	auto res = m_priorityQueue.get_priority(camID);
	
	if (status == 0) // no detection
		m_priorityQueue.set(camID, int( res.second / 2));
	else 
		m_priorityQueue.set(camID, HIGH_PRIOR);

	// unregister from process list 
	m_camInProcess.erase(std::remove(m_camInProcess.begin(), m_camInProcess.end(), 8), m_camInProcess.end());

	return 1;
}

void CLoadBalaner::prior()
{


}
/*
	void run3(int camID, int duration_ms)
	{
		if (m_resourceBouncer.take(camID)) {
			//std::cout << "Run process for camera " << camID << " for " << duration_ms << " ms \n";
			std::cout << camID << "\n";
			Sleep(duration_ms);
			m_resourceBouncer.release(camID);
			g_balancer.set(i, 0);

		}
		else
			;// std::cout << "Request for cam = " << camID << " rejected \n";
	}



	bool CLoadBalaner::test()
	{
		std::thread th[MaxCameras];
		std::thread th_single;

		int reourceNum = 5;
		//m_resourceBouncer.set(reourceNum);

		for (int i = 0; i < MaxCameras; i++)
			m_priorityQueue.push(i, 0);

		for (int f = 0; f < 1000; f++) {
			for (int camID = 0; camID < MaxCameras; camID++) {
				if (th[camID].joinable())
					th[camID].join();
				th[camID] = std::thread(run3, camID, 25);
				//Sleep(50);
			}

			Sleep(1000);
			m_priorityQueue.beat();

		}


		return true;


	}


*/

#endif 