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
    //FILE* fp = fopen(fname.c_str(), "rb");
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

    // Close the file 
    fclose(fp);


    // Parse file

    for (int rec = 0; rec < 1; rec++) {
        // Get the "age" member 
        if (doc.HasMember("camID") && doc["camID"].IsInt()) {
            int camID = doc["camID"].GetInt();
            cameras.push_back(CAlert_());
            cameras.back().m_camID = camID;
            std::cout << "camID: " << camID << std::endl;
        }

        // Get the "detection-type" member 
        if (doc.HasMember("detection-type") && doc["detection-type"].IsString()) {
            std::string typeStr = doc["detection-type"].GetString();

            int label = 0;// DDEBUG - must convert str to LABEL !!!!!

            cameras.back().m_label = label;
            std::cout << "label: " << label << std::endl;
        }

        // Get the "max-allowed" member 
        if (doc.HasMember("max-allowed") && doc["max-allowed"].IsInt()) {
            int threshold = doc["max-allowed"].GetInt();
            cameras.back().m_maxAllowed = threshold;
            std::cout << "camID: " << threshold << std::endl;
        }


        // Get the "polygon" (points) array 
        if (doc.HasMember("polygon")
            && doc["polygon"].IsArray()) {
            const rapidjson::Value& Points = doc["polygon"];
            std::cout << "polygon points: ";

            if (Points.Size() % 2 != 0) {
                std::cout << "Error in Polygon points list - list has OD elements \n";
                continue;
            }

            for (rapidjson::SizeType i = 0; i < Points.Size(); i += 2) {
                if (Points[i].IsInt() && Points[i + 1].IsInt()) {
                    cameras.back().m_polyPoints.push_back(cv::Point(Points[i].GetInt(), Points[i + 1].GetInt()));
                }
            }
        }
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
