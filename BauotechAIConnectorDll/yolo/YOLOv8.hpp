#pragma once

// DDEBUG function - should read from classes file!!!


class CYolo8 {
public:
    bool init(std::string modelFolder,   bool is_cuda);
    void detect(cv::Mat &image, std::vector<YDetection> &output);
    //void detect(cv::Mat &image, std::vector<YDetection> &output, std::vector <cv::Rect>  ROIs);
    std::string  getClassStr(int i) { return (m_class_list.size() > i ? m_class_list[i] : "None");}

private:
    std::vector<std::string> m_class_list;
	std::string m_modelFolder;
    cv::dnn::Net m_net;


};

