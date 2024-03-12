#include <stdio.h>
#include <fstream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"

// ptree for read cameras file
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp> 
#include <boost/property_tree/json_parser.hpp>

#include "rapidjson/document.h" 
#include "rapidjson/filereadstream.h" 
#include "rapidjson/error/en.h"

#include "database.hpp"


#include "CObject.hpp"
#include "yolo/yolo.hpp"

#include "utils.hpp"
#include "database.hpp"
#include "CObject.hpp"
#include "alert.hpp"


/*------------------------------------------------------------------------------------------------------------
* Alert for Anomaly (exception) according to Alert setting (the conditions to exception, type, count, area)
-------------------------------------------------------------------------------------------------------------*/
int CAlert::Siren(std::vector <CObject> objects)
{
	int hits = 0;


	for (auto obj : objects) {
		if (m_label == obj.m_label && cv::pointPolygonTest(m_polyPoints, obj.center(), false) > 0)
			hits++;
	}

	if (hits > m_maxAllowed)
		return hits;
	else 
		return 0;

}



std::vector <CObject> CAlert::selectObjects(std::vector <CObject> objects)
{
    std::vector <CObject> intruders;

    if (m_polyPoints.empty())
        return intruders;

	for (auto obj : objects) {
        if (obj.m_label == m_label && cv::pointPolygonTest(m_polyPoints, obj.center(), false) > 0)
            intruders.push_back(obj);
        else
            bool debug = cv::pointPolygonTest(m_polyPoints, obj.center(), false);
	}

    return intruders;
}


/*------------------------------------------------------------------------------------------------------------
* Set alert params : 
    * ROI = polygon
    * label type 
    * max allowed
------------------------------------------------------------------------------------------------------------*/
void CAlert::set(std::vector<cv::Point > polyPoints, int label, int max_allowed, int polyID)
{
    m_label = label; 
    m_maxAllowed = max_allowed;
    m_polyPoints = polyPoints;
    m_ployID = polyID;

	m_bbox = cv::boundingRect(m_polyPoints);

}


using boost::property_tree::ptree;

template <typename T>
std::vector<T> as_vector(ptree const& pt, ptree::key_type const& key)
{
	std::vector<T> r;
	for (auto& item : pt.get_child(key))
		r.push_back(item.second.get_value<T>());
	return r;
}

/*--------------------------------------------------------------------------------------------
*  Read cameras ROI ( as vector (polygon))
* Format:
* camID <>
* detection-type <>
* max-allowed <>
* polygon <point1_X, point1_Y, point2_X, point2_Y, ...>
 --------------------------------------------------------------------------------------------*/
int readCamerasJson(std::string fname, std::vector <CAlert>& cameras, int cameraIndex)
{
    FILE* fp;
    fopen_s(&fp, fname.c_str(), "rb");

    // Check if the file was opened successfully 
    if (!fp) {
        std::cerr << "Error: unable to open file"
            << std::endl;
        return 0;
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

    rapidjson::Value::ConstValueIterator itr;

    try {
        for (itr = doc.Begin(); itr != doc.End(); ++itr) {
            CAlert camInfo;
            // Access the data in the object
            int camID = itr->GetObject_()["camID"].GetInt();
            if (camID != cameraIndex)
                continue;
            camInfo.m_camID = camID;
            //std::cout << "camID: " << camID << std::endl;
            std::string typeStr = itr->GetObject_()["detection-type"].GetString();
            camInfo.m_label = getYoloClassIndex(typeStr);

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

    return cameras.size();
}




cv::Rect setcamerasROI(std::vector <CAlert>& cameras)
{
    cv::Rect camROI = cameras[0].m_bbox;
    int maxX = cameras[0].m_bbox.br().x;
    int maxY = cameras[0].m_bbox.br().y;

    for (auto& camInf : cameras) {
        camROI.x = min(camROI.x, camInf.m_bbox.x);
        camROI.y = min(camROI.y, camInf.m_bbox.y);
        maxX = max(maxX, camInf.m_bbox.br().x);
        maxY = max(maxY, camInf.m_bbox.br().y);
    }
    camROI.width = maxX - camROI.x;
    camROI.height = maxY - camROI.y;


    return camROI;

}
