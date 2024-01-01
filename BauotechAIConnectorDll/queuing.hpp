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

#if 0
class TSBuffQueue {
private:
    std::mutex g_BufferMutex;
    std::vector <char*>  m_buffers;
    int m_imgWidth;
    int m_imgHeight;
    int m_depth; // in bytes
    int bufferSize() { return m_imgWidth * m_imgHeight * m_depth; }
    int m_queueLen;

    int pushPtr = 0;    // index in queue
    int popPtr = -1;  // index in queue
    
    int ptrNext(int ptr);

public:
    TSBuffQueue() {}
    ~TSBuffQueue()
    {
        for (auto ptr: m_buffers)
            free (ptr);

    }
    TSBuffQueue(int imgWidth, int imgHeight, int depth, int queueLen);

    void set(int imgWidth, int imgHeight, int depth, int queueLen);
    // Pushes an element to the queue 
    bool push(char* buf);
    char* pop();
    char* back();
};
#endif 

// Thread-safe queue 
#if 0
template <typename T>
class TSQueue {
private:
    // Underlying queue 
    std::queue<T> m_queue;

    // mutex for thread synchronization 
    std::mutex m_mutex;

    // Condition variable for signaling 
    std::condition_variable m_cond;

public:
    // Pushes an element to the queue 
    void push(T item)
    {

        // Acquire lock 
        std::unique_lock<std::mutex> lock(m_mutex);

        // Add item 
        m_queue.push(item);

        // Notify one thread that 
        // is waiting 
        m_cond.notify_one();
    }

    // Pops an element off the queue 
    T pop()
    {

        // acquire lock 
        std::unique_lock<std::mutex> lock(m_mutex);

        // wait until queue is not empty 
        m_cond.wait(lock,
            [this]() { return !m_queue.empty(); });

        // retrieve item 
        T item = m_queue.front();
        m_queue.pop();

        // return item 
        return item;
    }
};
#endif 
