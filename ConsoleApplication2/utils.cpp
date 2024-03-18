#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"


#include "utils.hpp"
 
// Draw ROI polygon 
/*
void drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale)
{
    if (contour.empty())
        return;

    cv::Scalar color(0, 255, 0);
    //if (cv::iscorrect(contour)) {
    cv::drawContours(img, std::vector<std::vector<cv::Point> >(1, contour), -1, color, 1, 8);
}
*/

std::string toUpper(std::string str)
{
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    return str;
}