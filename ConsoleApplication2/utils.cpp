#include <iostream>
#include <vector>
#include <map>
#include <thread>
#include <shared_mutex>  // C++17
#include <filesystem>



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


int readFalseList(std::string folderName, std::vector <cv::Mat>& r_imgs, std::vector <int>& r_camIDs, std::vector <int>& r_labels)
{
	const std::filesystem::path FalseFolder{ folderName };
	/*
	const std::filesystem::path FalseFolder{R"(C:\Program Files\Bauotech\dll\algo\data\)"};

	if (!UTILS::isFolderExist(R"(C:\Program Files\Bauotech\dll\algo\data\)"))
		return 0;
	*/

	for (auto const& dir_entry : std::filesystem::directory_iterator{ FalseFolder }) {
		int camID = -1;
		std::string sPath = dir_entry.path().generic_string(); // convert to string 
		// get cam ID:
		//----------------------------------------------------------------
		std::string sFName = dir_entry.path().filename().generic_string();
		std::string sExt = dir_entry.path().extension().generic_string();

		auto ptr = sFName.find("camid_");
		if (ptr != std::string::npos) {
			auto dot = sFName.find(".");

			int IDStrLen = dot - ptr - 6;
			std::string sCamid = sFName.substr(ptr + 6, IDStrLen);
			camID = atoi(sCamid.c_str());
		}
		//----------------------------------------------------------------

		// Add false-image to the list
		//if (camID == r_camID) {
		cv::Mat falseImg = cv::imread(sPath);
		int alertLabel = 0; // DDEBUG DDEBUG DDEBUG CONST 
		r_imgs.push_back(falseImg);
		r_camIDs.push_back(camID);
		r_labels.push_back(alertLabel);
		// }
	}

	return r_imgs.size();
}
