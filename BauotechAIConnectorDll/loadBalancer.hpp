#ifndef LOAD_BALANCER_HEADER
#define LOAD_BALANCER_HEADER

#include <iostream> 
#include <thread>
#include <mutex> 
#include <queue>
#include <vector>
#include <condition_variable> 

#include "timer.hpp"
#include "config.hpp"
#include <tuple>

//#include <boost/circular_buffer.hpp>

#include "semaphore.hpp"
#include "priority_queue.hpp"
//#include "updatable_priority_queue.h"

#define CYCLE_MS 500.
typedef void (*CameraRequestCallback)(const uint32_t camera[], int size); // (was taking from algoApi.h)


#define ROUND(a)   floor(a + 0.5);
/*===========================================================================================
* AlgoProcess (thread) class
  ===========================================================================================*/


enum LB_MODE {
    FULL = 1,
    LIGHT = 2,
    STATISTICS = 3
};


enum {
	FREE = 0,
	BUSY = 1
};

enum OBJECT_STATUS {
    READY = 0,      // Processed and Reprior , waiting in queue
    IN_PROCESS,     // place holder
    DONE            // Right after process, before repriorized 
};


static  std::vector<int> ValidSchemeNum = { 0,1,2,3, 20,21, 30,31, 200,201, 300,301,302 };

static bool isValidSchemeNum(int val) { return         std::find(ValidSchemeNum.begin(), ValidSchemeNum.end(), val) != ValidSchemeNum.end(); }

enum LB_SCHEME {
    V0 = 0,
    V1,
    V2,
    V3,
    // simple 
    V20=20,
    V21,
    V30=30,
    V31,
    // smooth 
    V200=200,
    V201,
    V300=300,
    V301,
    V302
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
        alerts = 0;
        newObjs = 0;
        motion = 0;
        status = OBJECT_STATUS::READY;
    }

    int camID = -1;
    int priority = -1;
    int detections = 0;
    int activeCamera = 0;
    int alerts = 0;
    int newObjs = 0; // New objects - provided by getNewObjects()
    int motion = 0;
    int timeStamp = 0;
    OBJECT_STATUS status = OBJECT_STATUS::READY;


};

enum AQUIRE_ERROR {
    OK = 0,
    RESOURCE_OVERFLOW = 1,
    NOT_IN_LIST,
    NOT_IN_TOP ,
    DUPLICATE_AQUIRED ,
    NOT_ACTIVE 
};

#define USE_LOAD_BALANCER
#ifdef USE_LOAD_BALANCER


class CPrioritiesLevles {
public:
    /*--------------------------------------------------------------
     * Set 4 levels:
     * High [3], 2nd [2], 3d [1], low [0]
    --------------------------------------------------------------*/
    void init(int camsNum, int resourcesNum, int topPriority)
    {
        float fPrior;

        if (camsNum <= 0 || resourcesNum <= 0)
            return;

        m_priorities.clear(); // for mulpile call to init()

        // [0] (lowest)
        m_priorities.push_back(0); 
        // [1] 1/3
        fPrior = (float)topPriority / 3.;
        fPrior = floor(fPrior + 0.5);
        m_priorities.push_back(int(fPrior));
        // [2] 2/3
        fPrior = (float)topPriority * 2. / 3.;
        fPrior = floor(fPrior + 0.5);
        m_priorities.push_back(int(fPrior));
        // [3] higest
        m_priorities.push_back(topPriority);

    }

    // Return the priority for queue
    // In range of 0..3 (see init()
    //---------------------------------
    int get(int prior)
    {
        if (prior < 0 || prior > 3)
            return 0;
            
        return m_priorities[prior];
    }


    // scheme V2
    std::vector <int> m_priorities;

    int m_highPrio = 30; // 100;
    int m_2ndPrio = int((m_highPrio * 3) / 4);
    int m_3dPrio = int(m_highPrio / 2);
    int m_lowPrio = 0;
    int m_MidPrio = 1;
};

class CResourceRange {
public:
    CResourceRange(int f, int t, int res) { from = f; to = t; resourcesNum = res; }
    void set(int f, int t) { from = f; to = t; }
    int from, to, resourcesNum;
    bool inRange(int camerasNum)
    {
        return (camerasNum >= from && camerasNum <= to);
    }
};


//----------------------------------------------------------------------------
//  L O A D     B A L A N C E R      C L A S S  
//----------------------------------------------------------------------------

class CLoadBalaner  {
public:
    
    bool isActive() { return m_active; }
    LB_MODE getMode() { return m_Mode; }
    int init(uint32_t* batchSize, LB_MODE lbMode = LB_MODE::FULL);
    //int init(LB_SCHEME scheme = LB_SCHEME::V301);
    void stop() 
    {
            m_terminate = true;
    }
    AQUIRE_ERROR acquire(int camID);
    void remove(int camID);
    bool releaseDebug(int camID);

    /*--------------------------------------------
     * Three balancers: 
     * (1) full balancer
     * (2) light balancer
     * (3) statistics only (for Eli "balancer")
     ---------------------------------------------*/
    void priorityTH(); // thread to consolidate cam detection results 
    void priorityTHLight();
    void statisticsTH(); 

    bool priorityUpdateTop(); // Handle top cam in queue

    // Callback for camera threads 
    void ResQueuePush(CCycle);
    static void ResQueueNotify();

    bool isCamInBatchList(int camID); 

    void cameraCounter(int cams); // { m_camerasLen += cams;  } // Add or remove cameras counter
    void initCamera(int videoIndex);
    void SetCameraRequestCallback(CameraRequestCallback callback); 
    void SetCameraType(int camID, int type);

    // DDEBUG tests:
    void test(); 
    void test_async(); 

private:

    //void updatePCResource(int camerasNum);// { return CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE; }
    void tuneQueueParams(int camerasNum);// { return CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE; }
    void set(int camID, int proir);
    
    int getDebugLevel() { return m_debugLevel;  } // for algoAPI 

    /*
    int priorityOverflow()
    {
        int overflow = m_topPriority - m_priorityQueue.top().priority;
        return max(0, overflow); // 'overflow' can be negative
    }
    */

    int calcPriority(int motion, int detections, int alerts, int observed);
    int calcPriority_V0(int motion, int detections, int alert, int observed);
    int calcPriority_V1(int motion, int detections, int alert, int observed);
    int calcPriority_V2(int motion, int detections, int alert, int observed);
    int calcPriority_V3(int motion, int detections, int alert, int observed);
    // New and simpler scheme 
    // Prefere with detection cams
    int calcPriority_V20(int motion, int detections, int alert, int observed);
    int calcPriority_V21(int motion, int detections, int alert, int observed);
    // Prefere without detection cams
    int calcPriority_V30(int motion, int detections, int alert, int observed);
    int calcPriority_V31(int motion, int detections, int alert, int observed);
    // smooth 
    int calcPriority_V200(int motion, int detections, int alert, int observed);
    int calcPriority_V201(int motion, int detections, int alert, int observed);
    int calcPriority_V300(int motion, int detections, int alert, int observed);
    int calcPriority_V301(int motion, int detections, int alert, int observed);
    int calcPriority_V302(int motion, int detections, int alert, int observed);

    //void setPrior(CCycle info);

    void prepareNextBatch(int mode);
    void prepareNextBatchFull();
    void prepareNextBatchLight();

    std::vector <int> getCamTopLight();
    void updatePriorThresholds();

    void printQueueInfo();


public:
    CSemaphore m_resourceBouncer;

private:
    int convertToQueuePriority_simple(int priority);
    int convertToQueuePriority(int priority);
    bool initYoloMngr(int resourceNum, std::string modelFolder);

private:

    LB_MODE m_Mode = LB_MODE::FULL;


    std::vector <int>   m_cameraBatchList; // list sent to server 
    std::vector <int>   m_cameraBatchListAquired; // list sent to server 

    std::vector <int>   m_cameraBatchList_prev; // list sent to server 
    std::vector <int>   m_camsProcessed; // cameras had been process (return in resQueue)
    std::vector <int>   m_camType; // cameras type
    std::vector <int>   m_camPriority_Debug; // cameras type
    std::vector <CCycle>   m_logBatchQueue;
    int m_debugLevel = 1; // DDEBUG CONST 
    int m_cycleCounter = 0; // batch counter 
    std::vector <CResourceRange>   m_resourcesRange;
    LB_SCHEME   m_scheme = LB_SCHEME::V0;

    int m_topPriority = 10; // queue scale , from 0 .. 10
    int m_beatStep;       // beat increase step for cams in queue 

    CPriority_queue m_priorityQueue;

    bool m_active = false;
    bool m_statisticsOnly = false;

	int m_camerasNum = 0;
    //boost::circular_buffer<std::chrono::system_clock::time_point> m_timestamps;
    std::chrono::system_clock::time_point  m_lastBeat;
    const int TIMESTAMP_LEN = 1000;
    bool m_startProcess = false; // stream start =  run3() was called 
    bool m_terminate = false;

    int m_actualTopPriority; // actual in curent queue 
    CPrioritiesLevles m_priorities;

    int m_maxResourceNum = CONSTANTS::DEFAULT_LOADBALANCER_RESOURCE;
    int m_resourceNum = m_maxResourceNum; // can be less tham max availble - in case num pf cameras is less

    std::thread m_beatthread;


    std::thread  m_PrioirityTh;

    int m_lastCamIndex = 0;

    bool m_printStatistics = true; // DDEBUG PRINTING 

    CameraRequestCallback m_bachCallback = nullptr;
};
#else
// SUSPEND ALL LOAD_BALANCER ACTIVITIES 
class CLoadBalaner {
public:
class CLoadBalaner(int reoucesLen = 4, int camerasLen = MAX_CAMERAS) {}
void setRosourceSemaphore(CSemaphore* sem) { }
//void init(int reoucesLen, int camerasLen) {}
int init() { return 0; }
void initCamera(int videoIndex) {}
bool acquire(int camID, bool allwedOverflow) { return true; }
int release(int camID, int status) { return 1;}
void set(int camID, int proir){}
void beat(int cycles) { }
bool isActive() { return false; }
int getPriority(int camID) { return 0; }
void SetCameraRequestCallback(CameraRequestCallback callback) { }
void SetCameratype(int camID, int type) {}
void remove(int camID) {}
bool releaseDebug(int camID) { return true; }
void SetCameraType(int camID, int type) { }
void ResQueuePush(CCycle info) {}
void ResQueueNotify() {}


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

#endif 