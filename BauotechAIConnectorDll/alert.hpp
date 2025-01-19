#ifndef ALERT_HPP
#define ALERT_HPP
#pragma once

#include <vector>
#include<tuple>
/*--------------------------------------------------------------------------------------------------------------
* 
 --------------------------------------------------------------------------------------------------------------*/
enum MotionType {
	OnlyMoving	= 0,
	MotionOrNot = 1,
	OnlyStatics	= 2
};

class CAlert {
public:
	CAlert() {}
	CAlert(std::vector<cv::Point > contour, int label, int motionType, int max_allowed, int ployID, int timeLimit=0, int camID = -1) { set(contour, label, motionType, max_allowed, ployID, timeLimit, camID); }
	void	set(std::vector<cv::Point > contour, int label, int motionType, int max_allowed, int ployID, int timeLimit, int camID);

	//int		Siren(std::vector <CObject> objects);
	std::vector <CObject> selectObjects(std::vector <CObject> objects );

public:
	bool checkPolygon(int width, int height)
	{
		for (auto& point : m_polyPoints) {
			if (point.x < 0 || point.x >= width  )
				return false;
			//point.x = width - 1;
			if (point.y < 0 || point.y >= height)
				return false;
			//point.y = height - 1;
		}
		return true;
	}

public:
	int m_camID;
	int m_ployID;
	std::vector<cv::Point > m_polyPoints;
	int m_label;
	int m_motionType; 
	int m_maxAllowed = -1;
	cv::Rect  m_bbox;
	int m_timeLimit=0;
};


int readCamerasJson(std::string fname, std::vector <CAlert>& cameras, int cameraIndex);
cv::Rect setcamerasROI(std::vector <CAlert>& cameras);


#endif 