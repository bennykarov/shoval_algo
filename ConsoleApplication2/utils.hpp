#ifndef UTILS_CONSOLE2_HPP
#define UTILS_CONSOLE2_HPP
#pragma once

#include <vector>

class CAlert_ { // duplicate structure of CAlert (alert.hpp)

public:
    int m_camID;
    std::vector<cv::Point > m_polyPoints;
    //std::tuple <int,int> label_allowed;
    int m_label = -1;
    int m_maxAllowed = -1;
    cv::Rect  m_bbox;
};


int readCamerasJson(std::string fname, int camID, std::vector <CAlert_>& cameras);
void drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale);


#endif 