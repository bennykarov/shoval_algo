#pragma once

#include "../BauotechAIConnectorDll/AlgoApi.h" // for borh consoleApp and DLL (algoProcess debug draw)


class CAlert_ { // duplicate structure of CAlert (alert.hpp)

public:
	int m_camID;
	std::vector<cv::Point > m_polyPoints;
	//std::tuple <int,int> label_allowed;
	int m_label = -1;
	int m_maxAllowed = -1;
	cv::Rect  m_bbox;
public:
	CAlert_() {}
	CAlert_(std::vector<cv::Point > polyPoints, int label, int maxAllowed)
	{
		m_polyPoints = polyPoints;
		m_label = label;
		m_maxAllowed = maxAllowed;
	}

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
};


int draw(int height, int width, char* pData, std::vector <ALGO_DETECTION_OBJECT_DATA> AIObjects, std::vector <CAlert> g_cameraInfos, int framenum, float scale_, bool invertImg);
void drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale);

