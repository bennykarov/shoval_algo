#pragma once 
#include <algorithm>
#include <iostream>
#include <shared_mutex>


class CSemaphore {
public:
    /*CSemaphore(int maxCount_)
        : maxCount(maxCount_)
    {
    }
    */

    inline void set(int maxCount_)
    {
        maxCount = maxCount_;
    }

    inline int get()
    {
        return maxCount - count;
    }

    void release(int camID);
    bool take(int camID);

private:
    std::condition_variable cv;
    int count = 0;
    int maxCount = 4; 
};


