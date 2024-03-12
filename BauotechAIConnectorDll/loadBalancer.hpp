#ifndef LOAD_BALANCER_HEADER
#define LOAD_BALANCER_HEADER

#include <iostream> 
#include <condition_variable> 
#include <mutex> 
#include "timer.hpp"
#include "config.hpp"

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

enum {
    LOW_PRIOR = 0,
    MID_PRIOR = 5, 
    HIGH_PRIOR = 10
};

class CResource {
	int status = FREE;
	int camID = -1;
	std::chrono::system_clock::time_point   startTime;
};



#ifdef USE_LOAD_BALANCER
class CLoadBalaner  {
public:
    CLoadBalaner(int reoucesLen = 4, int camerasLen = MAX_CAMERAS)
    {
        init(reoucesLen, camerasLen);
    }
    void setRosourceSemaphore(CSemaphore *sem) { m_resourceBouncer = sem; }
    void init(int reoucesLen, int camerasLen);
    bool try_acquire(int camID);
    int release(int camID, int status);
    void set(int camID, int proir)
    {
        m_priorityQueue.set(camID, proir);
    }
    int getPriority(int camID)
    {
        return m_priorityQueue.get_priority2(camID);
        }

    void beat(int cycles = 1) 
    { m_priorityQueue.beat(cycles); }
    
    bool isActive() { return true; }


    void beatTimer();

public:
    CSemaphore *m_resourceBouncer;

private:
	void prior();

private:
	std::vector <int>   m_camInProcess; // cameras that are in process
	better_priority_queue::updatable_priority_queue<int, int> m_priorityQueue;

	int m_reoucesLen = 4;
	int m_camerasLen = 10;
    boost::circular_buffer<std::chrono::system_clock::time_point> m_timestamps;
    std::chrono::system_clock::time_point  m_lastBeat;
    const int TIMESTAMP_LEN = 1000;
    bool m_startProcess = false; // stream start =  run3() was called 
    bool m_terminate = false;

    int resourceNum;

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