#pragma once

#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 


#include <boost/circular_buffer.hpp>
#include "timer.hpp"


class CframeBuffer {
public:
    char* ptr = nullptr;
    int64_t frameNum = -1;
    uint64_t ts = 0;
    int     bufSize = 0;
    //CTimer m_timer;

public:
    CframeBuffer() {  }
    CframeBuffer(int64_t  _frameNum, int64_t _ts, char* _ptr) 
    { 
        ts = _ts; 
        frameNum = _frameNum; 
        ptr = _ptr; 
    }


public:
    bool alloc(int size) {
		if (ptr != nullptr) {
			free();
		}
		ptr = new char[size];
        bufSize = size;
		return ptr != nullptr;
	}

    // clone the buffer with inner allocation
    CframeBuffer clone()
    {
		CframeBuffer newFrame;
        newFrame.frameNum = frameNum;
        newFrame.ts = ts;
        newFrame.bufSize = bufSize;

        newFrame.alloc(bufSize);
		memcpy(newFrame.ptr, ptr, bufSize);

        return newFrame;
	}

    // copy the buffer - assume ptr is already allocated
    CframeBuffer copy(CframeBuffer &newFrame)
    {
        newFrame.frameNum = frameNum;
        newFrame.ts = ts;
        newFrame.bufSize = bufSize;

        //newFrame.alloc(bufSize);
        memcpy(newFrame.ptr, ptr, bufSize);

        return newFrame;
    }

    void free() {
        if (ptr != nullptr) {
			delete[] ptr;
			ptr = nullptr;
		}
	}
};

//----------------------------
// Thread-safe buffer queue
//---------------------------
class TSBuffQueue {
private:
    std::mutex m_BufferMutex;
    std::vector <CframeBuffer>  m_buffers;
    int m_buffersCounter=0; // count buffer in queue
    int m_imgWidth;
    int m_imgHeight;
    int m_depth; // in bytes
    CTimer m_timer;

    //-------------------------------------------------------------------------------------------------------------
    // bufferSize() : depth ==2 here means YV12, where depth is 1.5 bytes per pixel (where 4 pixels share data):
    //-------------------------------------------------------------------------------------------------------------
    int bufferSize_YV12() { return int((float)(m_imgWidth * m_imgHeight) * 1.5); }
    int m_queueLen;

    int pushPtr = 0;    // index in queue
    int popPtr =  0;  // index in queue

    int ptrNext(int ptr);

    //int getQueueSize();

    bool queueOverflow() { return pushPtr == popPtr && m_buffersCounter == m_queueLen; }

     

public:
    int bufferSize() { return (m_depth == 2 ? bufferSize_YV12() : m_imgWidth * m_imgHeight * m_depth); }
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
    bool popCopy(CframeBuffer& poppedFrame);
    bool pop(CframeBuffer& poppedFrame);
    bool front(CframeBuffer& frontFrame);
};
