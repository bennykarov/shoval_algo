#include <iostream>
#include <vector>
#include <map>
#include <thread>
#include <shared_mutex>  // C++17


#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"


#include "utils.hpp"
 

using namespace UTILS_CONSOLE2;

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

std::string UTILS_CONSOLE2::toUpper(std::string str)
{
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    return str;
}

static std::shared_mutex camIDMapMutex;  // Read-Write lock

int UTILS_CONSOLE2::camID2Ind_(int camID)
{
    static int indexCounter = 0;
    static std::map <int, int> cam2Ind;

    std::unique_lock<std::shared_mutex> lock(camIDMapMutex);  // Exclusive lock for writing
    if (cam2Ind.find(camID) == cam2Ind.end())
        cam2Ind[camID] = indexCounter++;

    return (cam2Ind[camID]);
}


std::vector<cv::Point> UTILS_CONSOLE2::roiToPolygon(cv::Rect roi)
{
    std::vector<cv::Point > polygon;
	polygon.push_back(roi.tl());
	polygon.push_back(cv::Point(roi.x + roi.width, roi.y));
	polygon.push_back(roi.br());
	polygon.push_back(cv::Point(roi.x, roi.y + roi.height));

	return polygon;

}