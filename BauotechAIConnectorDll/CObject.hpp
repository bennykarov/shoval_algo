#pragma once 

#ifndef COBJECT_HEADER
#define COBJECT_HEADER
#include "config.hpp"


/* OLD OBJECT CLASS
class CObject2
{
public:

	int class_id;
	float confidence;
	cv::Rect box;
	std::vector <cv::Point2f> centers;
};
*/


enum Labels
{
	nonLabled = -1,
	person,		//	 0
	bicycle,    //   1
	car,        //   2
	motorbike,  //   3
	aeroplane,  //   4
	bus,        //   5
	train,      //   6  
	truck,       //   7
	boat,
	traffic_light,
	fire_hydrant,
	stop_sign,
	parking_meter,
	bench,
	bird,
	cat,
	dog,
	unknown = 9999
};


enum DETECT_TYPE {
	DETECT_NA = 0,
	BGSeg,
	ML,
	Tracking,
	Hidden,   // prediction on blind 
	Prediction 
	};


class CObject {
public:
	CObject() :m_bbox(0,0,0,0) { }
	CObject(cv::Rect  r, int frameNum, unsigned int id, DETECT_TYPE  detectionType, int label)
	{
		m_label = label;
		m_detectionType = detectionType;
		m_frameNum = frameNum;
		m_bbox = r;
		m_moving = 1; // used more as STATIC flag - object that doesnt move for sure.
	}

	bool empty() { return m_bbox.width == 0; }

	inline int getHiddenLen(int frameNum) {
		return m_frameNum - frameNum;
	}

	inline bool isHinheritLabel() {
		return m_label != m_finalLabel;
	}

	cv::Point center() {
		return cv::Point(m_bbox.x + int((float)m_bbox.width / 2.), m_bbox.y + int((float)m_bbox.height / 2.));
	}


public:
	unsigned int	m_ID = 0;  //unique id
	int				m_label = Labels::nonLabled; // current detection label
	int				m_finalLabel = Labels::nonLabled;  // Conclusion of all detection labels
	cv::Rect		m_bbox; // DDEBUG for debug
	DETECT_TYPE		m_detectionType;
	int				m_frameNum;
	int				m_moving; // pixel distance
	int				m_age = 0;
	float			m_confidance = 0;

private:
};

#endif 