#ifndef ALERT_HPP
#define ALERT_HPP
#pragma once

#include <vector>
#include<tuple>
/*--------------------------------------------------------------------------------------------------------------
* 
 --------------------------------------------------------------------------------------------------------------*/

/* From 24: 
_OnlyMoving = 0,
_MotionOrNot = 1,
_OnlyStatics = 2,
*/

enum MotionType {
	OnlyMoving	= 0,
	MotionOrNot = 1,
	OnlyStatics	= 2
};

class CAlert {
public:
	CAlert() {}
	CAlert(std::vector<cv::Point > contour, int label, int motionType, int max_allowed, int ployID, int camID = -1) { set(contour, label, motionType, max_allowed, ployID, camID); }
	void	set(std::vector<cv::Point > contour, int label, int motionType, int max_allowed, int ployID, int camID);

	int		Siren(std::vector <CObject> objects);
	std::vector <CObject> selectObjects(std::vector <CObject> objects);

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
	//std::tuple <int,int> label_allowed;
	int m_label;
	int m_motionType; 
	int m_maxAllowed = -1;
	cv::Rect  m_bbox;

};


int readCamerasJson(std::string fname, std::vector <CAlert>& cameras, int cameraIndex);
cv::Rect setcamerasROI(std::vector <CAlert>& cameras);


#endif 