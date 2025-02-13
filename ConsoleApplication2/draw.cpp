//============================================================================
// ConsoleApplication2.cpp : Shoval_sc test app
//============================================================================
#include <iostream>
#include <thread>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "utils.hpp"

#include "../BauotechAIConnectorDll/AlgoApi.h"
#include "../BauotechAIConnectorDll/CObject.hpp"  // required by alert.hpp
#include "../BauotechAIConnectorDll/alert.hpp"

#include "draw.hpp"
#include <format>

std::vector <cv::Scalar> g_colors = { cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255), cv::Scalar(255,255,0), cv::Scalar(255,0,255), cv::Scalar(0,255,255) , cv::Scalar(155,55,0), cv::Scalar(25,0,55), cv::Scalar(0,0,0) , cv::Scalar(110,155,255) };

void CDISPLAY::drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale)
{
	if (contour.empty())
		return;

	cv::Scalar color(0, 255, 0);
	//if (cv::iscorrect(contour)) {
	cv::drawContours(img, std::vector<std::vector<cv::Point> >(1, contour), -1, color, 3, 8);
}

void CDISPLAY::drawInfo(cv::Mat& img, CAlert camInfo)
{
	cv::Scalar color(255, 0, 0);
	// Draw ROI

	// Draw alert-polygon 
	drawPolygon(img, camInfo.m_polyPoints, 1.); // DDEBUG draw camera[0]

}

void CDISPLAY::drawInfo(cv::Mat& img, std::vector <CAlert> camsInfo)
{
	cv::Scalar color(255, 0, 0);
	// Draw ROI

	// Draw alert-polygon
	for (auto camInfo : camsInfo)
		drawPolygon(img, camInfo.m_polyPoints, 1.); // DDEBUG draw camera[0]

}




/* return cv::waitKey() */
int CDISPLAY::draw(int height, int width, char *pData, std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjects, std::vector <CAlert> g_cameraInfos, int frameNum, bool invertImg)
{
	static int wait = 1;
	int videoTodisplayInd = 0;
	int key;
	float scale = width < 1400 ? 1.0 : 1400. / (float)width;


	cv::Mat frameAfter = cv::Mat(height, width, CV_8UC3, pData);
	if (invertImg)
		cv::flip(frameAfter, frameAfter, 0);

	//--------------------
	// -1- Draw Polygons 
	//--------------------
	if (!g_cameraInfos.empty()) {
		std::vector<CAlert>::iterator iter = g_cameraInfos.begin();

		std::vector <CAlert> curCamPolys;
		while ((iter = std::find_if(iter, g_cameraInfos.end(), [&videoTodisplayInd](const CAlert& alert) { return alert.m_camID == videoTodisplayInd; })) != g_cameraInfos.end()) {
			curCamPolys.push_back(*iter);
			iter++;
		} 

		drawInfo(frameAfter, curCamPolys); 
	}

	//----------------------
	// -2- Draw detections 
	//----------------------
	bool displayPerID = true; // DDEBUG flag


	if (displayPerID)
		for (auto obj : AIObjects) {
			int colorInd = obj.ObjectType % g_colors.size();
			//if (frameNum > obj.frameNum)   colorInd = 8; // BLACK

			int thickness = 2;
			if (obj.DetectionPercentage < 0) { // TRICK TO REVEAL MOTION TYPE (static = negative)
				//thickness *= 3;
				cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], thickness);
				cv::rectangle(frameAfter, cv::Rect(obj.X - thickness, obj.Y - thickness, obj.Width + thickness*2, obj.Height + thickness * 2), g_colors[colorInd+1], thickness);
			}
			else
				cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], thickness);
			bool draw_confidence = true; // confidence OR objID
			if (draw_confidence)
			{	// draw Object confidence (ubless its trick for Static obj 9999...)
				//cv::putText(frameAfter, std::format("{:.2f}", obj.DetectionPercentage), cv::Point(obj.X, obj.Y - 5), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 255));
				cv::putText(frameAfter, std::to_string(obj.DetectionPercentage)+"%", cv::Point(obj.X, obj.Y - 5), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 255));
			}
			else
				// draw Object ID
				cv::putText(frameAfter, std::to_string(obj.ID), cv::Point(obj.X, obj.Y - 5), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 255));

			cv::putText(frameAfter, std::to_string(obj.ObjectType), cv::Point(obj.X, obj.Y + obj.Width - 2), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(0,0,0));
		}
	else 
		for (auto obj : AIObjects) {
			/*
			int colorInd = obj.ObjectType % g_colors.size();
			cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], 2);
			*/
			int colorInd = g_colors.size() - 1; //  DDEBUG
			int thickness = 6;
			if (obj.DetectionPercentage == 9999)
				thickness *= 2;
			//if (obj.)
			cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], thickness);

			if (obj.ObjectType != 2)
				int debug = 10;


		}



	////------------------
	cv::Mat display;
	cv::resize(frameAfter, display, cv::Size(0, 0), scale, scale);


	cv::putText(display, std::to_string(frameNum), cv::Point(display.cols - 170, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));

	if (!AIObjects.empty()) {
		cv::putText(display, "D"+std::to_string(AIObjects.size()), cv::Point(20, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));
		//std::cout << "Frame " << frameNum << " : " << AIObjects[0].x << " , " << AIObjects[0].y << "\n";
	}


	if (display.cols > 1200) {
		float scale = 1200. / (float)display.cols;
		cv::resize(display, display, cv::Size(0, 0), scale, scale);
	}
	cv::imshow("processed-image", display);
	key = cv::waitKey(wait);
	if (key == 'p')
		wait = -1;
	else
		wait = 1;

	return key;
}


int CDISPLAY::draw(cv::Mat frameAfter, std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjects, std::vector <CAlert> g_cameraInfos, int frameNum, bool invertImg)
{

	static int wait = 1;
	int videoTodisplayInd = 0;
	int key;

	float scale = frameAfter.cols < 1400 ? 1.0 : 1400. / (float)frameAfter.cols;


	if (invertImg)
		cv::flip(frameAfter, frameAfter, 0);

	//--------------------
	// -1- Draw Polygons 
	//--------------------
	if (!g_cameraInfos.empty()) {
		std::vector<CAlert>::iterator iter = g_cameraInfos.begin();
		//iter = std::find_if(g_cameraInfos.begin(), g_cameraInfos.end(),[&videoTodisplayInd](const CAlert& alert) { return alert.m_camID == videoTodisplayInd; });

		std::vector <CAlert> curCamPolys;
		while ((iter = std::find_if(iter, g_cameraInfos.end(), [&videoTodisplayInd](const CAlert& alert) { return alert.m_camID == videoTodisplayInd; })) != g_cameraInfos.end()) {
			curCamPolys.push_back(*iter);
			iter++;
		}

		drawInfo(frameAfter, curCamPolys);
	}

	//----------------------
	// -2- Draw detections 
	//----------------------


	bool displayPerID = true; // DDEBUG flag

	if (displayPerID)
		for (auto obj : AIObjects) {
			int colorInd = obj.ObjectType % g_colors.size();
			int thickness = 2;
			if (obj.DetectionPercentage == 9999)
				thickness *= 3;

			cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], thickness);
			//cv::putText(frameAfter, std::to_string(obj.ID), cv::Point(obj.X, obj.Y - 5), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 255));
			cv::putText(frameAfter, std::to_string(obj.DetectionPercentage), cv::Point(obj.X, obj.Y - 5), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 0, 255));
			cv::putText(frameAfter, std::to_string(obj.ObjectType), cv::Point(obj.X, obj.Y + obj.Width - 2), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(255,255, 0));
		}
	else
		for (auto obj : AIObjects) {
			/*
			int colorInd = obj.ObjectType % g_colors.size();
			cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], 2);
			*/
			int colorInd = g_colors.size() - 1; //  DDEBUG
			int thickness = 6;
			if (obj.DetectionPercentage == 9999)
				thickness *= 2;
			//if (obj.)
			cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], thickness);

			if (obj.ObjectType != 2)
				int debug = 10;


		}



	////------------------
	cv::Mat display;
	//float scale = 0.7;
	cv::resize(frameAfter, display, cv::Size(0, 0), scale, scale);


	cv::putText(display, std::to_string(frameNum), cv::Point(display.cols - 170, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));

	if (!AIObjects.empty()) {
		cv::putText(display, "D" + std::to_string(AIObjects.size()), cv::Point(20, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));
		//std::cout << "Frame " << frameNum << " : " << AIObjects[0].x << " , " << AIObjects[0].y << "\n";
	}

	cv::imshow("processed-image", display);
	key = cv::waitKey(wait);
	if (key == 'p')
		wait = -1;
	else
		wait = 1;

	return key;
}






