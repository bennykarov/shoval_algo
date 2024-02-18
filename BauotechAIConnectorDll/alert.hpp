#ifndef ALERT_HPP
#define ALERT_HPP
#pragma once

#include <vector>
#include<tuple>
/*--------------------------------------------------------------------------------------------------------------
* 
 --------------------------------------------------------------------------------------------------------------*/


class CAlert {
public:
	CAlert() {}
	CAlert(std::vector<cv::Point > contour, int label, int max_allowed) { set(contour, label, max_allowed); }
	void	set(std::vector<cv::Point > contour, int label, int max_allowed);

	int		Siren(std::vector <CObject> objects);
	std::vector <CObject> selectObjects(std::vector <CObject> objects);

public:
	bool checkPolygon(int width, int height)
	{
		for (auto& point : m_polyPoints) {
			if (point.x >= width)
				return false;
			//point.x = width - 1;
			if (point.y >= height)
				return false;
			//point.y = height - 1;
		}
		return true;
	}

public:
	int m_camID;
	std::vector<cv::Point > m_polyPoints;
	//std::tuple <int,int> label_allowed;
	int m_label;
	int m_maxAllowed = -1;
	cv::Rect  m_bbox;

};


int readCamerasJson(std::string fname, std::vector <CAlert>& cameras, int cameraIndex);


#endif 