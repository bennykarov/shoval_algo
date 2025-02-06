//#include <Windows.h> // Beep()
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "../logger.hpp"
#include "YOLO_mngr.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;



//static CYolo8 m_yolos[MAX_DETECTORS];
static std::vector <int> m_freedetector;


/*------------------------------------------------------------------------------------------
* Init detectors (currently 2 YOLO models =  Original and Forklift model)
------------------------------------------------------------------------------------------*/


bool ST_yoloDetectors::init(std::string modelFolder, bool is_cuda)
{
    bool ret;
    std::vector  <std::string> yoloMedelNames = { YOLO_MODEL_NAME , YOLO_MODEL_EXPANDED_NAME };

    m_yolos.resize(yoloMedelNames.size());

    for (int i = 0; i < yoloMedelNames.size(); i++) {
        std::string modelFullpath = modelFolder + "/" + yoloMedelNames[i];
        if (!m_yolos[i].init(modelFullpath, is_cuda)) {
            LOGGER::log(DLEVEL::ERROR2, "Error: Failed to load YOLO model: " + modelFullpath);
            return false;
        }
    }

    return true;
}

// instence detectoer
void ST_yoloDetectors::detect(cv::Mat& frame, std::vector<YDetection>& output, std::vector <int> lables_to_detect)
{
    freeDetectors.set(m_id, 0);
    m_yolos[0].detect(frame, output); // Original YOLO model

    // Special Modles for extended classes (currently Forklift dmodel):
    if (std::find(lables_to_detect.begin(), lables_to_detect.end(), FORKLIFT_CLASS_BASE+0) != lables_to_detect.end())
    {
		std::vector<YDetection> output_expanded;
        m_yolos[1].detect(frame, output_expanded);

        for (auto &obj : output_expanded) 
			obj.class_id += FORKLIFT_CLASS_BASE; // change label 0 to FORKLIFT ID 

        output.insert(output.end(), output_expanded.begin(), output_expanded.end());
	}

    freeDetectors.set(m_id, 1);
}

cv::Rect ST_yoloDetectors::optimizeRect(cv::Rect r, cv::Size dim)
{
    return m_yolos[0].optimizeRect(r, dim);
    //return m_yolos[id].optimizeRect(r, dim);
}

 