#include <Windows.h> // sleep
#include <iostream>
#include "semaphore.hpp"

static std::shared_mutex mtx;


    void CSemaphore::release() 
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        count--;
        if (count < 0)
            std::cout << "Error in semaphore counter !!!!\n";
    }

    bool CSemaphore::take() 
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        if (count >= maxCount)
            return false;
        count++;

        //std::cout << "semaphore count after take = " << count << "\n";
        return true;
    }

    bool CSemaphore::take(int maxWaitToResourceSec) 
    {
        int maxWaitToResource = maxWaitToResourceSec / 5;
        int waitedToResource = 0;
        while (!take() && waitedToResource < maxWaitToResource ) {
            Sleep(5);
            waitedToResource++;
        }

        if (waitedToResource < maxWaitToResource )
            return true;
        else
            return false;
    }

