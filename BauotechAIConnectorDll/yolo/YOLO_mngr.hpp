#pragma once 

//#include <Windows.h> // Beep()
#include <vector>
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "../utils.hpp"

#include "yolo.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;



#define MAX_DETECTORS 100



class ST_yoloDetectors {
private:

    static std::map<int, std::shared_ptr<ST_yoloDetectors>> instances;  // Map of instances
    static std::mutex st_mtx;
    //static std::vector <int >freeDetectors;
    static ThreadSafeVector <int> freeDetectors;


    // Private constructor
    ST_yoloDetectors(int id) : m_id(id) {
        std::cout << "ST_yoloDetectors instance with ID: " << id << " created!" << std::endl;
    }

    int m_id;  // Unique identifier for each instance
    std::vector <CYolo8> m_yolos; // Unique YOLO Detector for each instance

public:
    static int m_maxDetectors;
    // Multiple level
    //------------------
    // Deleted copy constructor and assignment operator to prevent copying
    ST_yoloDetectors(const ST_yoloDetectors&) = delete;
    ST_yoloDetectors& operator=(const ST_yoloDetectors&) = delete;

    // Find free detector using the 'freeDetectors' vecotr 
    static int getFreeInstenceInd() 
    {
        return freeDetectors.findFree();
    }
    
    //---------------------------------------------------------------
    // Return a free instance if availble (using freeDetectors vec)
    // return NULL if none is available 
    //--------------------------------------------------------------
    static std::shared_ptr<ST_yoloDetectors> getInstance()
    {
        std::lock_guard<std::mutex> lock(st_mtx);

        int freeID = freeDetectors.findFree();

        if (freeID < 0 || instances.find(freeID) == instances.end())
             return nullptr;

        return instances[freeID];
    }

    static void setMaxdetectors(int max)
    {
        m_maxDetectors = max;
    }


    // Static method to get or create an instance
    static std::shared_ptr<ST_yoloDetectors> getInstance(int id)
    {
        std::lock_guard<std::mutex> lock(st_mtx);


        if (instances.find(id) == instances.end()) {
            if (freeDetectors.size() >= m_maxDetectors) // exceed limit of YOLO detectors allowed 
                return nullptr;

            instances[id] = std::shared_ptr<ST_yoloDetectors>(new ST_yoloDetectors(id));
            freeDetectors.push_back(1); // set as free
        }
        return instances[id];
    }

    static void terminate()
    {
        instances.clear();
        m_maxDetectors = 0;
    }

    // Singleton level
    //------------------
    bool init(std::string modelFName, bool is_cuda = true);

    void showMessage(std::string msg) const
    {
        std::cout << msg << "ST detector# " << m_id << std::endl;
    }

    void detect(cv::Mat& frame, std::vector<YDetection>& output, std::vector <int> lables_to_detect = std::vector <int>()); // instence detect   



    cv::Rect optimizeRect(cv::Rect r, cv::Size dim);




};
