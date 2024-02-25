#pragma once


const float YOLO_INPUT_WIDTH = 640.0;
const float YOLO_INPUT_HEIGHT = 480.0;

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

private:

    std::vector<std::string> m_class_list;
    std::string m_modelFolder;
    cv::dnn::Net m_net;


};

class CYolo8 {
public:
    bool init(std::string modelFolder, bool is_cuda);
    void detect(cv::Mat& image, std::vector<YDetection>& output);
    bool load_net(bool is_cuda);
    //void detect(cv::Mat &image, std::vector<YDetection> &output, std::vector <cv::Rect>  ROIs);
    std::string  getClassStr(int i) { return (m_class_list.size() > i ? m_class_list[i] : "None"); }

private:

    std::vector<std::string> m_class_list;
    std::string m_modelFolder;
    cv::dnn::Net m_net;


};


