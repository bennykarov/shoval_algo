//#include <Windows.h> // Beep()
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "YOLO_mngr.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;



//static CYolo8 m_yolos[MAX_DETECTORS];
static std::vector <int> m_freedetector;


/*------------------------------------------------------------------------------------------
* find a free Detecotr Instence 
 ------------------------------------------------------------------------------------------*/

// instence detectoer
void ST_yoloDetectors::detect(cv::Mat& frame, std::vector<YDetection>& output)
{
    freeDetectors.set(id, 0);
    m_yolo.detect(frame, output);
    freeDetectors.set(id, 1);
}

cv::Rect ST_yoloDetectors::optimizeRect(cv::Rect r, cv::Size dim)
{
    return m_yolo.optimizeRect(r, dim);
    //return m_yolos[id].optimizeRect(r, dim);
}

 