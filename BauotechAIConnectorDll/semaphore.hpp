#pragma once 
#include <algorithm>
#include <iostream>
#include <shared_mutex>


class CSemaphore {
public:

    inline void set(int maxCount_)
    {
        maxCount = maxCount_;
    }

    inline int get()
    {
        return maxCount - count;
    }

    void release();
    bool take();
    bool take(int maxWaitToResource);

private:
    std::condition_variable cv;
    int count = 0;
    int maxCount = 4; 
};


