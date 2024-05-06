#ifndef LOAD_BALANCER_HEADER
#define LOAD_BALANCER_HEADER

#include <iostream> 
#include <thread>
#include <mutex> 
#include <queue>
#include <condition_variable> 

#include "timer.hpp"
#include "config.hpp"
#include <tuple>

#include <boost/circular_buffer.hpp>

#include "semaphore.hpp"
#include "priority_queue.hpp"
#include "updatable_priority_queue.h"

//#define USE_LOAD_BALANCER // DDEBUG DDEBUG FLAG 

#define CYCLE_MS 500.
#define MaxCameras  20

typedef void (*CameraRequestCallback)(const uint32_t camera[], int size); // (was taking from algoApi.h)


/*===========================================================================================
* AlgoProcess (thread) class
  ===========================================================================================*/
enum {
	FREE = 0,
	BUSY = 1
};

enum CAMERA_TYPE {
    Normal = 0,
    Active,
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
    
    //bool isActive() { return true; }
    void init();
    bool acquire(int camID, bool allowOverflow=false);
    void remove(int camID);
    bool releaseDebug(int camID);

    void priorityTH(); // thread to consolidate cam detection results 
    bool priorityUpdate(); // thread to consolidate cam detection results 

    // Callback for camera threads 
    static std::queue<CCycle>     *getResQueuePtr(); 
    static std::condition_variable *getResCondVPtr();

    void initCamera(int videoIndex);
    void SetCameraRequestCallback(CameraRequestCallback callback) { m_bachCallback = callback; }

    void SetCameraType(int camID, int type);

    // DDEBUG tests:
    void test(); 
    void test_async(); 

private:
    //void test_run3(CCycle& info, int duration_ms);


    int calcPCResource() { return CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;  }
    void set(int camID, int proir);
    

    int priorityOerflow()
    {
        int overflow = m_topPriority - m_priorityQueue.top().priority;
        return max(0, overflow); // 'overflow' can be negative
    }

    int calcPriority(int motion, int detections, int alert, int observed);

    void setPrior(CCycle info);

    void nextBatch();
    void updatePriorThresholds();

    void beat(int cycles = 1)
    {
        m_priorityQueue.beat(cycles);
    }



    void beatTimer();
    void prior();

public:
    CSemaphore m_resourceBouncer;

private:
    int convertToQueuePriority(int priority);

private:
    std::vector <int>   m_cameraBatchList; // list sent to server 
    std::vector <int>   m_camInProcess; // cameras actually in process
    std::vector <int>   m_camInQueue; // cameras actually in process
    std::vector <int>   m_camType; // cameras type

#if 0
    better_priority_queue::updatable_priority_queue<int, int> m_priorityQueue;
#else
    CPriority_queue m_priorityQueue;
#endif 

    bool m_active = false;

	//int m_reoucesLen = 4;
	int m_camerasLen = 0;
    boost::circular_buffer<std::chrono::system_clock::time_point> m_timestamps;
    std::chrono::system_clock::time_point  m_lastBeat;
    const int TIMESTAMP_LEN = 1000;
    bool m_startProcess = false; // stream start =  run3() was called 
    bool m_terminate = false;

    int m_topPriority = 1;
    int m_thirdPriority = 0;
    int m_2thirdPriority = 0;
    int m_resourceNum = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;

    std::thread m_beatthread;


    std::thread  m_PrioirityTh;

    CameraRequestCallback m_bachCallback = nullptr;
};
#else
// SUSPEND ALL LOAD_BALANCER ACTIVITIES 
class CLoadBalaner {
public:
class CLoadBalaner(int reoucesLen = 4, int camerasLen = MAX_CAMERAS) {}
void setRosourceSemaphore(CSemaphore* sem) { }
//void init(int reoucesLen, int camerasLen) {}
void init() {}
void initCamera(int videoIndex) {}
bool acquire(int camID) { return true; }
int release(int camID, int status) { return 1;}
void set(int camID, int proir){}
void beat(int cycles) { }
bool isActive() { return false; }
int getPriority(int camID) { return 0; }
void SetCameraRequestCallback(CameraRequestCallback callback) { }
void SetCameratype(int camID, int type) {}
void remove(int camID) {}
bool releaseDebug(int camID) { return true; }

public:
    CSemaphore m_resourceBouncer;
    std::vector <int>   m_cameraBatchList; // list sent to server 
    std::vector <int>   m_camInProcess; // cameras actually in process
    std::vector <int>   m_camInQueue; // cameras actually in process


#if 0
better_priority_queue::updatable_priority_queue<int, int> m_priorityQueue;
#else
CPriority_queue m_priorityQueue;
#endif 




std::queue<CCycle>* getResQueuePtr()  {
    std::queue<CCycle> fake;
    return &fake;
}

std::condition_variable* getResCondVPtr()
{
    std::condition_variable fake;
    return &fake;
}


void test_async() {}
void test() {}

};
#endif 

#if 0
class CLaunchList {
private:
    std::mutex mtxList[MAX_CAMERAS];
    std::vector <int> camList;

public:
    void add(int camID)
    {
        std::lock_guard<std::mutex> bufferLockGuard(mtxList[camID]);
        camList.push_back(camID);
    }

    void release(int camID)
    {
        std::lock_guard<std::mutex> bufferLockGuard(mtxList[camID]);
        auto it = std::find(camList.begin(), camList.end(), camID);
        if (it != camList.end()) {
            camList.erase(it);
        }
    }

    std::vector <int> getCamList() { return camList; }

    int len() { return camList.size(); }

};
#endif 

#endif 