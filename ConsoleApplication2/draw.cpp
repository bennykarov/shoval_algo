//============================================================================
// ConsoleApplication2.cpp : Shoval_sc test app
//============================================================================
#include <iostream>
#include <thread>        
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "utils.hpp"

#include "../BauotechAIConnectorDll/AlgoApi.h"
#include "../BauotechAIConnectorDll/CObject.hpp"  // required by alert.hpp
#include "../BauotechAIConnectorDll/alert.hpp"

#include "draw.hpp"


std::vector <cv::Scalar> g_colors = { cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255), cv::Scalar(255,255,0), cv::Scalar(255,0,255), cv::Scalar(0,255,255) , cv::Scalar(155,55,0), cv::Scalar(25,0,55), cv::Scalar(110,155,255), cv::Scalar(0,0,0) };

void drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale)
{
	if (contour.empty())
		return;

	cv::Scalar color(0, 255, 0);
	//if (cv::iscorrect(contour)) {
	cv::drawContours(img, std::vector<std::vector<cv::Point> >(1, contour), -1, color, 1, 8);
}

void drawInfo(cv::Mat& img, CAlert camInfo)
{
	cv::Scalar color(255, 0, 0);
	// Draw ROI

	// Draw alert-polygon 
	drawPolygon(img, camInfo.m_polyPoints, 1.); // DDEBUG draw camera[0]

}

void drawInfo(cv::Mat& img, std::vector <CAlert> camsInfo)
{
	cv::Scalar color(255, 0, 0);
	// Draw ROI

	// Draw alert-polygon
	for (auto camInfo : camsInfo)
		drawPolygon(img, camInfo.m_polyPoints, 1.); // DDEBUG draw camera[0]

}




/* return cv::waitKey() */
int draw(int height, int width, char *pData, std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjects, std::vector <CAlert> g_cameraInfos, int framenum, float scale , bool invertImg)
{

	static int wait = 1;
	int videoTodisplayInd = 0;
	int key;

	if (width < 500)
		scale = 1.0;

	cv::Mat frameAfter = cv::Mat(height, width, CV_8UC3, pData);
	if (invertImg)
		cv::flip(frameAfter, frameAfter, 0);

	// -1- Draw ROI 	
	if (!g_cameraInfos.empty()) {
		//int camID = videoTodisplayInd;
		auto displayCamInfo = std::find_if(g_cameraInfos.begin(), g_cameraInfos.end(),
			[&videoTodisplayInd](const CAlert& alert) { return alert.m_camID == videoTodisplayInd; });
		
		drawInfo(frameAfter, *displayCamInfo); // DDEBUG draw camera[0]
		//drawInfo(frameAfter, g_cameraInfos[videoTodisplayInd]); // DDEBUG draw camera[0]
	}

	bool displayPerID = true; // DDEBUG flag

	if (displayPerID)
		for (auto obj : AIObjects) {
			//int colorInd = obj.ID % g_colors.size();
			int colorInd = obj.ObjectType % g_colors.size();
			int thickness = 2;
			if (obj.DetectionPercentage == 9999)
				thickness *= 3;

			cv::rectangle(frameAfter, cv::Rect(obj.X, obj.Y, obj.Width, obj.Height), g_colors[colorInd], thickness);
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
	//float scale = 0.7;
	cv::resize(frameAfter, display, cv::Size(0, 0), scale, scale);


	cv::putText(display, std::to_string(framenum), cv::Point(display.cols - 170, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));

	if (!AIObjects.empty()) {
		cv::putText(display, "D", cv::Point(20, display.rows - 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(0, 0, 255));
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






