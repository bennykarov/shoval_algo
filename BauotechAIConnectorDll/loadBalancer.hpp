#ifndef LOAD_BALANCER_HEADER
#define LOAD_BALANCER_HEADER

#include <iostream> 
#include <condition_variable> 
#include <mutex> 
#include "timer.hpp"
#include "config.hpp"
#include <tuple>

#include <boost/circular_buffer.hpp>

#include "semaphore.hpp"
#include "updatable_priority_queue.h"

//#define USE_LOAD_BALANCER // DDEBUG DDEBUG FLAG 

#define CYCLE_MS 500.
/*===========================================================================================
* AlgoProcess (thread) class
  ===========================================================================================*/
enum {
	FREE = 0,
	BUSY = 1
};


class CResource {
	int status = FREE;
	int camID = -1;
	std::chrono::system_clock::time_point   startTime;
};

class CCycle {
public:
    CCycle() {}
    CCycle(int camID_, int priority_, int detections_, int activeCamera_)
    {
        camID = camID_;
        priority = priority_;
        detections = detections_;
        activeCamera = activeCamera_;
    }

    void reset()
    {
        camID = -1;
        priority = -1;
        detections = 0;
        activeCamera = 0;
        alert = 0;
        motion = 0;
    }

    int camID = -1;
    int priority = -1;
    int detections = 0;
    int activeCamera = 0;
    int alert = 0;
    int motion = 0;
    int timeStamp = 0;

};



#ifdef USE_LOAD_BALANCER
class CLoadBalaner  {
public:
    /*
    CLoadBalaner()
    {
        init();
    }
    */
    void init(CSemaphore* sem);
    int calcPCResource() { return CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;  }
    //void init(int reoucesLen, int camerasLen);
    //bool try_acquire(int camID);
    int release(int camID, int status);
    
    //void initCamera(int videoIndex);
    void set(int camID, int proir);
    
    int getPriority(int camID)
    {
        return m_priorityQueue.get_priority2(camID);
    }


    int priorityOerflow()
    {
        int overflow = m_topPriority - m_priorityQueue.top().priority;
        return max(0, overflow); // 'overflow' can be negative
    }

    int calcPriority(int motion, int detections, int alert, int observed);
    std::tuple <int, int> getNextCam();


    void updatePriorThresholds();
    void beat(int cycles = 1) 
    { m_priorityQueue.beat(cycles); }
    
    bool isActive() { return true; }


    void beatTimer();
    //void test(int reourceNum, int camerasLen); // DDEBUG function 
    void test(); // DDEBUG function 
    void test_async(); // DDEBUG function 
    std::tuple<int, int, int> test_run3_(int camID, int printPrior, int duration_ms);
    void test_run3(CCycle& info, int duration_ms);
    //void test_run4(CCycle info, int duration_ms, int &detections);
    //void test_run3_async(int camID, int printPrior, int duration_ms, CCycle& info);


public:
    CSemaphore *m_resourceBouncer;

private:
	void prior();
    int converToQueuePriority(int priority);
    void setRosourceSemaphore(CSemaphore* sem) { m_resourceBouncer = sem; }

private:
	std::vector <int>   m_camInProcess; // cameras that are in process
	better_priority_queue::updatable_priority_queue<int, int> m_priorityQueue;

	//int m_reoucesLen = 4;
	int m_camerasLen = 0;
    boost::circular_buffer<std::chrono::system_clock::time_point> m_timestamps;
    std::chrono::system_clock::time_point  m_lastBeat;
    const int TIMESTAMP_LEN = 1000;
    bool m_startProcess = false; // stream start =  run3() was called 
    bool m_terminate = false;

    int m_topPriority = 0;
    int m_thirdPriority = 0;
    int m_2thirdPriority = 0;
    int m_resourceNum = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;

    std::thread m_beatthread;
};
#else
// SUSPEND ALL LOAD_BALANCER ACTIVITIES 
class CLoadBalaner {
public:
    class CLoadBalaner(int reoucesLen = 4, int camerasLen = MAX_CAMERAS) {}
void setRosourceSemaphore(CSemaphore* sem) { }
void init(int reoucesLen, int camerasLen) {}
bool try_acquire(int camID) { return true; }
int release(int camID, int status) { return 1;}
void set(int camID, int proir){}
void beat(int cycles) { }
bool isActive() { return false; }
int getPriority(int camID) { return 0; }


};
#endif 

#endif 