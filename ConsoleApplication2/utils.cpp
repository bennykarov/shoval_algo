#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "rapidjson/document.h" 
#include "rapidjson/filereadstream.h" 
#include "rapidjson/error/en.h"

#include "utils.hpp"
/*--------------------------------------------------------------------------------------------
*  Read cameras ROI ( as vector (polygon))
* Format:
* camID <>
* detection-type <>
* max-allowed <>
* polygon <point1_X, point1_Y, point2_X, point2_Y, ...>
 --------------------------------------------------------------------------------------------*/


int readCamerasJson(std::string fname, int camID, std::vector <CAlert_>& cameras)
{
    FILE* fp;
    fopen_s(&fp, fname.c_str(), "rb");

    // Check if the file was opened successfully 
    if (!fp) {
        std::cerr << "Error: unable to open file"
            << std::endl;
        return 1;
    }

    // Read the file into a buffer 
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer,
        sizeof(readBuffer));

    // Parse the JSON document 
    rapidjson::Document doc;
    doc.ParseStream(is);

    // Check if the document is valid 
    if (doc.HasParseError()) {
        std::cerr << "Error: failed to parse JSON document"
            << std::endl;
        fprintf(stderr, "\nError(offset %u): %s\n",
            (unsigned)doc.GetErrorOffset(),
            GetParseError_En(doc.GetParseError()));
        fclose(fp);
        return 0;
    }
    fclose(fp);


    // Parse file
    rapidjson::Value::ConstValueIterator itr;

	try {
		for (itr = doc.Begin(); itr != doc.End(); ++itr) {
			CAlert_ camInfo;
			// Access the data in the object
			camInfo.m_camID = itr->GetObject_()["camID"].GetInt();
			//std::cout << "camID: " << camID << std::endl;
			std::string typeStr = itr->GetObject_()["detection-type"].GetString();
			camInfo.m_label = 0;// DDEBUG - must convert str to LABEL !!!!!
			camInfo.m_maxAllowed = itr->GetObject_()["max-allowed"].GetInt();

            if (itr->HasMember("polygon") && itr->GetObject_()["polygon"].IsArray()) {
                std::vector <int> points;
                rapidjson::Value::ConstValueIterator itrP;
                for (itrP = itr->GetObject_()["polygon"].Begin(); itrP != itr->GetObject_()["polygon"].End(); ++itrP) 
                    points.push_back(itrP->GetInt());

                CHECK_exception(true, "Error in Polygon points list - list has OD elements");


                for (int p = 0; p < points.size(); p += 2)
                    camInfo.m_polyPoints.push_back(cv::Point(points[p], points[p + 1]));
            }

            cameras.push_back(camInfo);
		}
	}
	catch (const std::exception& err) {
		std::cout << "Error in parsing file : " << fname << " : " << err.what() << "\n";
	}
    return 1;


}

// Draw ROI polygon 
void drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale)
{
    if (contour.empty())
        return;

    cv::Scalar color(0, 255, 0);
    //if (cv::iscorrect(contour)) {
    cv::drawContours(img, std::vector<std::vector<cv::Point> >(1, contour), -1, color, 1, 8);
}
