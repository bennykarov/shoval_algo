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

    /*---------------------------------------------------------------------
     Pushes an element to the queue 
     'pushPtr' is the index of the next element to be pushed
     Push any case - even if queue is full (overwrite previous frames in queue)
     TBD: in case POP is in the same location - skip to next location 
     ---------------------------------------------------------------------*/

    int TSBuffQueue::getQueueSize()
    {
        if (pushPtr == popPtr)
        {
            return 0;
        }
        if (pushPtr > popPtr)
		{
			return pushPtr - popPtr;
		}
		else
		{
			return m_queueLen - popPtr + pushPtr;
		}   
    }

    bool TSBuffQueue::push(CframeBuffer frame)
    {
        std::lock_guard<std::mutex> bufferLockGuard(g_BufferMutex); // ????  DDEBUG ????

        //std::cout << ">> push = " << m_buffers[pushPtr].frameNum << std::endl;
        if (getQueueSize() >= m_queueLen)
        {
			std::cout << "overflow in buffer queueing \n";
			return false; // queue is full
		}
         

        memcpy(m_buffers[pushPtr].ptr, frame.ptr, bufferSize());
        m_buffers[pushPtr].frameNum = frame.frameNum;


        // Starter to popPtr
        //if (popPtr < 0)
			//popPtr = pushPtr;

         
        pushPtr = (pushPtr + 1) % m_queueLen;
        //pushPtr = ptrNext(pushPtr);
        return true;
    }

    bool TSBuffQueue::IsEmpty()
    {
        std::lock_guard<std::mutex> bufferLockGuard(g_BufferMutex);

        return (pushPtr == popPtr); 
    }

    bool TSBuffQueue::pop(CframeBuffer& poppedFrame)
    {
        std::lock_guard<std::mutex> bufferLockGuard(g_BufferMutex);

        if (getQueueSize() <= 0)
        {
            return false; // Queue is empty
        }
 
        poppedFrame = m_buffers[popPtr];
        popPtr  = (popPtr + 1) % m_queueLen;
        return true;
    }


    bool TSBuffQueue::front(CframeBuffer& frontFrame)
    {
        std::lock_guard<std::mutex> bufferLockGuard(g_BufferMutex);

        if (popPtr < 0)
            return false; // Queue is empty

        frontFrame = m_buffers[popPtr];
        return true;
    }

// NOTES \ FRAFTS 
#if 0
//------------------------------------------------------------------------------------
// Thread safe circular buffer
//------------------------------------------------------------------------------------
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

    // Thread safe circular buffer 
    template <typename T>
    class circ_buffer : private boost::noncopyable
    {
    public:
        typedef boost::mutex::scoped_lock lock;
        circ_buffer() {}
        circ_buffer(int n) { cb.set_capacity(n); }
        void send(T imdata) {
            lock lk(monitor);
            cb.push_back(imdata);
            buffer_not_empty.notify_one();
        }
        T receive() {
            lock lk(monitor);
            while (cb.empty())
                buffer_not_empty.wait(lk);
            T imdata = cb.front();
            cb.pop_front();
            return imdata;
        }
        void clear() {
            lock lk(monitor);
            cb.clear();
        }
        int size() {
            lock lk(monitor);
            return cb.size();
        }
        void set_capacity(int capacity) {
            lock lk(monitor);
            cb.set_capacity(capacity);
        }
    private:
        boost::condition buffer_not_empty;
        boost::mutex monitor;
        boost::circular_buffer<T> cb;
    };
#endif 



#if 0
    boost::circular_buffer<int> cb(3);
    // Insert three elements into the buffer.
    cb.push_back(1);
    cb.push_back(2);
    cb.push_back(3);

    int a = cb[0];  // a == 1
    int b = cb[1];  // b == 2
    int c = cb[2];  // c == 3

    // The buffer is full now, so pushing subsequent
    // elements will overwrite the front-most elements.

    cb.push_back(4);  // Overwrite 1 with 4.
    cb.push_back(5);  // Overwrite 2 with 5.

    // The buffer now contains 3, 4 and 5.
    a = cb[0];  // a == 3
    b = cb[1];  // b == 4
    c = cb[2];  // c == 5

    // Elements can be popped from either the front or the back.
    cb.pop_back();  // 5 is removed.
    cb.pop_front(); // 3 is removed.

    // Leaving only one element with value = 4.
    int d = cb[0];  // d == 4
#endif 