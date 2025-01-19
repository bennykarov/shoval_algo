#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 

#include "queuing.hpp"


//------------------------------------------------------------------------------------
// Thread-safe buffer queue
// this class alloc the buffer (accordoing to set() params) and manage the queue
//------------------------------------------------------------------------------------

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
            //frameBuf.ptr = new char[bufferSize()];
            frameBuf.alloc(bufferSize());
            m_buffers.push_back(frameBuf);
        }
    }

    /*---------------------------------------------------------------------
     Pushes an element to the queue 
     'pushPtr' is the index of the next element to be pushed
     Push any case - even if queue is full (overwrite previous frames in queue)
     TBD: in case POP is in the same location - skip to next location 
     ---------------------------------------------------------------------*/

    bool TSBuffQueue::push(CframeBuffer frame)
    {
        std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex); // ????  DDEBUG ????

        memcpy(m_buffers[pushPtr].ptr, frame.ptr, bufferSize());
        m_buffers[pushPtr].frameNum = frame.frameNum;

        if (frame.ts != 0)
            m_buffers[pushPtr].ts = frame.ts;
        else
            m_buffers[pushPtr].ts = m_timer.sampleFromStart(); // use sampleFromStart() to get time in ms

        //-------------------------------------------------------------
        // In case queue is full - overwrite oldest frame in queue, 
        // and point popPtr to next (now oldest) location
        // In overwriting - dont increase 'm_buffersCounter
        //-------------------------------------------------------------
        if (queueOverflow()) {
            popPtr = (popPtr + 1) % m_queueLen;
            //std::cout << "====================  Buffers queue overflow in " << pushPtr << ">>>>>>>>>>>>>>>>>>>>>>>>>>> \n";
            //std::cout << "*";
        }
        else
            m_buffersCounter++;        

        pushPtr = (pushPtr + 1) % m_queueLen;
        return true;
    }

    bool TSBuffQueue::IsEmpty()
    {
        std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);
        return (m_buffersCounter <= 0);
    }

    // Pop with copy to a new buffer
    //--------------------------------- 
    bool TSBuffQueue::popCopy(CframeBuffer& poppedFrame)
    {
        std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);

        if (m_buffersCounter <= 0)
        {
            return false; // Queue is empty
        }

        //poppedFrame = m_buffers[popPtr];
        //poppedFrame = m_buffers[popPtr].clone();
        m_buffers[popPtr].copy(poppedFrame);


        popPtr = (popPtr + 1) % m_queueLen;
        m_buffersCounter--;
        return true;
    }

    // Pop w/o copy to a new buffer
    bool TSBuffQueue::pop(CframeBuffer& poppedFrame)
    {
        std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);

        if (m_buffersCounter <= 0)
        {
            return false; // Queue is empty
        }

        poppedFrame = m_buffers[popPtr];
        popPtr = (popPtr + 1) % m_queueLen;
        m_buffersCounter--;
        return true;
    }


    bool TSBuffQueue::front(CframeBuffer& frontFrame)
    {
        std::lock_guard<std::mutex> bufferLockGuard(m_BufferMutex);

        if (popPtr < 0)
            return false; // Queue is empty

        frontFrame = m_buffers[popPtr];
        return true;
    }

    int TSBuffQueue::ptrNext(int ptr)
    {
        ptr++;
        if (ptr >= m_queueLen)
            return  0;

        return ptr;
    }

