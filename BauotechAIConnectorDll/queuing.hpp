#pragma once

#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 



#include <boost/circular_buffer.hpp>


class CframeBuffer {
public:
    CframeBuffer() {}
    CframeBuffer(int _frameNum, char* _ptr) { frameNum = _frameNum; ptr = _ptr; }

    char *ptr = nullptr;
    int frameNum = -1;

    void free() {
        if (ptr != nullptr) {
			delete[] ptr;
			ptr = nullptr;
		}
	}
};


// Thread-safe buffer queue
class TSBuffQueue {
private:
    std::mutex g_BufferMutex;
    //std::vector <char*>  m_buffers;
    std::vector <CframeBuffer>  m_buffers;
    int m_imgWidth;
    int m_imgHeight;
    int m_depth; // in bytes
    //-------------------------------------------------------------------------------------------------------------
    // bufferSize() : depth ==2 here means YV12, where depth is 1.5 bytes per pixel (where 4 pixels share data):
    //-------------------------------------------------------------------------------------------------------------
    int bufferSize() { return (m_depth == 2 ? bufferSize_YV12() : m_imgWidth * m_imgHeight * m_depth); }
    int bufferSize_YV12() { return int((float)(m_imgWidth * m_imgHeight) * 1.5); }
    int m_queueLen;

    int pushPtr = 0;    // index in queue
    int popPtr =  0;  // index in queue

    int ptrNext(int ptr);

    boost::circular_buffer<int> m_queueIndex;
    int getQueueSize();
     

public:
    TSBuffQueue() {}
    ~TSBuffQueue()
    {
        for (auto frame : m_buffers)
            frame.free();
    }
    TSBuffQueue(int imgWidth, int imgHeight, int depth, int queueLen);

    void set(int imgWidth, int imgHeight, int depth, int queueLen);
    // Pushes an element to the queue 
    bool push(CframeBuffer);
    bool IsEmpty();
    bool pop(CframeBuffer& poppedFrame);
    bool front(CframeBuffer& frontFrame);
};
