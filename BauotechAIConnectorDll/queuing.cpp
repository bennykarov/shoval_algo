#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 

#include "queuing.hpp"


//------------------------------------------------------------------------------------
// Thread-safe buffer queue
// this class alloc the buffer (accordoing to set() params) and manage the queue
//------------------------------------------------------------------------------------
    int TSBuffQueue::ptrNext(int ptr)
    {
        ptr++;
        if (ptr >= m_queueLen)
            return  0;
        
        return ptr;
    }

    TSBuffQueue::TSBuffQueue(int imgWidth, int imgHeight, int depth, int queueLen)
    {
        set(imgWidth, imgHeight, depth, queueLen);
    }

    void TSBuffQueue::set(int imgWidth, int imgHeight, int depth, int queueLen) 
    {
        this->m_imgWidth = imgWidth;
        this->m_imgHeight = imgHeight;
        this->m_depth = depth;
        this->m_queueLen = queueLen;
        for (int i = 0; i < queueLen; i++) {
            CframeBuffer frameBuf;
            frameBuf.ptr = new char[bufferSize()];
            m_buffers.push_back(frameBuf);
        }
    }

    // Pushes an element to the queue 
    bool TSBuffQueue::push(CframeBuffer frame)
    {
        std::lock_guard<std::mutex> bufferLockGuard(g_BufferMutex);

        if (pushPtr == popPtr) {
            std::cout << "overfloaw in buffer queueing \n";
            return false; // queue is full
        }

        memcpy(m_buffers[pushPtr].ptr, frame.ptr, bufferSize());
        m_buffers[pushPtr].frameNum = frame.frameNum;

        if (popPtr < 0)
			popPtr = pushPtr;

        pushPtr = ptrNext(pushPtr);
        return true;
    }

    // Pops an element off the queue 
    CframeBuffer TSBuffQueue::pop()
    {
        popPtr = ptrNext(popPtr);
        if (popPtr == pushPtr)
            return CframeBuffer();

        return this->m_buffers[popPtr];
    }
    // Get pop element off the queue 
    CframeBuffer TSBuffQueue::back()
    {
        if (popPtr < 0)
            return CframeBuffer();

        return this->m_buffers[popPtr];

    }

#if 0
// Driver code 
int test_queu()
{
    TSQueue<int> q;

    // Push some data 
    q.push(1);
    q.push(2);
    q.push(3);

    // Pop some data 
    std::cout << q.pop() << std::endl;
    std::cout << q.pop() << std::endl;
    std::cout << q.pop() << std::endl;

    return 0;
}
#endif

