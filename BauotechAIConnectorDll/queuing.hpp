#pragma once

#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 



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
    int bufferSize() { return m_imgWidth * m_imgHeight * m_depth; }
    int bufferSize_() { return int((float)(m_imgWidth * m_imgHeight) * 1.5); }
    int m_queueLen;

    int pushPtr = 0;    // index in queue
    int popPtr = -1;  // index in queue

    int ptrNext(int ptr);

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
    CframeBuffer pop();
    CframeBuffer back();
};
