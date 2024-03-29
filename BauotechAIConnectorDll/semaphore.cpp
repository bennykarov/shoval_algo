#include <iostream>
#include "semaphore.hpp"

static std::shared_mutex mtx;


    void CSemaphore::release(int camID) {
        std::unique_lock<std::shared_mutex> lock(mtx);
        count--;
        if (count < 0)
            std::cout << "Error in semaphore counter !!!!\n";
        //count = std::max(--count, 0);
    }
    bool CSemaphore::take(int camID) {
        std::unique_lock<std::shared_mutex> lock(mtx);
        if (count >= maxCount)
            return false;
        count++;

        //std::cout << "semaphore count after take = " << count << "\n";
        return true;
    }


