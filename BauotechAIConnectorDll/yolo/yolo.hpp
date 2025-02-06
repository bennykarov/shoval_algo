#pragma once


// LearnOpencv model builtin 
#if 0 // DDEBUG 
const std::string YOLO_MODEL_NAME = "yolov8s.onnx";
const float YOLO_INPUT_WIDTH = 640.0;
const float YOLO_INPUT_HEIGHT = 480.0;
#else 
//const std::string YOLO_MODEL_NAME = "yolov8m_640.onnx";
const std::string YOLO_MODEL_NAME = "yolov8l_640.onnx";
const std::string YOLO_MODEL_EXPANDED_NAME = "forklift_v8s_100e_MERGE.onnx"; // "forklift_v8m_100e_MERGE.onnx";
//const std::string YOLO_MODEL_EXPANDED_NAME = "forklift_v8s_100e_MERGE.onnx"; 

//const std::string YOLO_MODEL_NAME = "yolov8s_640.onnx";
const float YOLO_INPUT_WIDTH = 640.0;
const float YOLO_INPUT_HEIGHT = 640.0;
#endif 
const int VEHICLE_CLASS_ID = 999;
const int FORKLIFT_CLASS_BASE = 500;

// YOLO Thresholds 

const float YOLO_MIN_CONFIDENCE = 0.1; // Must be the max of MIN_CENFIDENCE in concluder.cpp !!!

const float YOLO_HIGH_CONFIDENCE = 0.7; // small boxe must have stron confidence 
//const float YOLO_CONFIDENCE_MIN_THRESHOLD = 0.4;
const float NMS_THRESHOLD = 0.5;



// DDEBUG function - should read from classes file!!!
std::string getYoloClassStr(int i);
int getYoloClassIndex(std::string classStr);

struct YDetection
{
    int class_id;
    float confidence;
    cv::Rect box;
};




class CYolo5 {
public:
    bool init(std::string modelFolder, bool is_cuda);
    void detect(cv::Mat& image, std::vector<YDetection>& output);
    void detect(cv::Mat& image, std::vector<YDetection>& output, std::vector <cv::Rect>  ROIs);
    std::string  getClassStr(int i) { return (m_class_list.size() > i ? m_class_list[i] : "None"); }

private:
    std::vector<std::string> load_class_list();
    cv::Mat format_yolov5(const cv::Mat& source);
    bool load_net(bool is_cuda);
    int getVersion() { return 5; }

private:

    std::vector<std::string> m_class_list;
    std::string m_modelFolder;
    cv::dnn::Net m_net;


};

class CYolo8 {
public:
    bool init(std::string modelFolder, bool is_cuda);
    void detect(cv::Mat& image, std::vector<YDetection>& output);
    bool load_net(std::string modelMame, bool is_cuda);
    //void detect(cv::Mat &image, std::vector<YDetection> &output, std::vector <cv::Rect>  ROIs);
    std::string  getClassStr(int i) { return (m_class_list.size() > i ? m_class_list[i] : "None"); }
    int getVersion() { return 8; }

    cv::Rect optimizeRect(cv::Rect r, cv::Size dim);

private:

    std::vector<std::string> m_class_list;
    std::string m_modelFolder;
    cv::dnn::Net m_net;


};

// UTILS:
static std::vector <std::string> lablesStr = { "person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat", 
"traffic light",
"fire hydrant",
"stop sign",
"parking meter",
"bench",
"bird",
"cat",
"dog",
"horse",
"sheep",
"cow",
"elephant",
"bear",
"zebra",
"giraffe",
"backpack",
"umbrella",
"handbag",
"tie",
"suitcase"
};
static std::string getYoloClassStr(int i)
{
    return lablesStr[i];
}
static int getYoloClassIndex(std::string classStr)
{
    if (classStr == "vehicle" || classStr == "VEHICLE")
        return VEHICLE_CLASS_ID;
    else if (classStr == "forklift" || classStr == "FORKLIFT")
        return FORKLIFT_CLASS_BASE+0;
    
    auto it = std::find(lablesStr.begin(), lablesStr.end(), classStr);
    if (it != lablesStr.end())
        return std::distance(lablesStr.begin(), it);
    else
        return -1;
}

static   char* labelToStr(int labelInd)
{
    char errorStr[] = "";

    if (labelInd == VEHICLE_CLASS_ID)
        return (char*)"vehicle";

    if (labelInd == FORKLIFT_CLASS_BASE+0)
        return (char*)"forklift";

    if (labelInd >= lablesStr.size())
        return errorStr;

    return ((char *)lablesStr[labelInd].data());
}


