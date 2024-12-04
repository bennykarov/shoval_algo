/*---------------------------------------------------------------------------------------------
 * Best RT tracker : CSRT (in opencv) or
 * the actual winner of VOT cha;;ange is : is SiamFC
 *---------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
//#include <opencv2/tracking/tracker.hpp>
#include  "opencv2/tracking/tracking.hpp"
#include  "opencv2/tracking/tracking_legacy.hpp"
//tracking_legacy.hpp
#include <opencv2/core/ocl.hpp>

#include "config.hpp"
#include "utils.hpp"
#include "trackerBasic.hpp"


//#define TRACKER_NO_LEGACY 

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( ( std::ostringstream() << std::dec << x ) ).str()

//========================================================================
// M U L T I       T R A C K E R    C L A S S 
//========================================================================

bool CMTracker::init()
{
    /*
    if (m_trackerType < 0) {
        std::cout << "Missing first Tracker init, use default : " << m_trackerTypes_str[m_trackerType];
    }

    if (!m_tracker.empty())
        m_tracker.release();

    m_tracker = createTrackerByName(m_trackerTypes_str[m_trackerType]);
    */
    return true;
}

bool CMTracker::init(int type, int debugLevel, int badFramesToReset)
{

    m_trackerType = type;
    m_badFramesToReset = badFramesToReset;
    m_debugLevel = debugLevel;


    if (type >= m_trackerTypes_str.size()) {
        std::cout << "Max tracker index = " << m_trackerTypes_str.size() << "\n";
        return false;
    }

    std::cout << "Tracker type = " << m_trackerTypes_str[m_trackerType] << "\n";

    return true;

}

void CMTracker::clear()
{ 
    for (auto trk : m_algorithms)
        trk->~Tracker();

    m_algorithms.clear();
    m_bboxs.clear();
    m_objID.clear();
    m_labels.clear();



}

void CMTracker::clear(int ind)
{

    m_algorithms[ind]->~Tracker();

    m_algorithms.erase(m_algorithms.begin() + ind);
    m_bboxs.erase(m_bboxs.begin() + ind);
    m_objID.erase(m_objID.begin() + ind);

}



void CMTracker::clear(std::vector <int> indices)
{

    std::sort(indices.begin(), indices.end(), greater<int>());

    for (auto ind : indices) {
        m_algorithms[ind]->~Tracker();
        m_algorithms.erase(m_algorithms.begin() + ind);
        m_bboxs.erase(m_bboxs.begin() + ind);
        m_objID.erase(m_objID.begin() + ind);
    }
}

void CMTracker::setROIs(std::vector <cv::Rect> bboxes, std::vector <int> objID, std::vector <int> labels, cv::Mat frame, bool clearHistory)
{
    if (m_trackerType < 0)
        return;

    for (int i = 0; i < MIN(bboxes.size(), MAX_TRACKERS_ALLOWED); i++) {
        m_bboxs.push_back(bboxes[i]);
#ifndef TRACKER_NO_LEGACY
        m_algorithms.push_back(createTrackerByName_legacy(m_trackerTypes_str[m_trackerType]));
#else 
        m_algorithms.push_back(createTrackerByName(m_trackerTypes_str[m_trackerType]));
#endif 
        m_objID.push_back(objID[i]);
        m_labels.push_back(labels[i]);

        if (m_trackerType == 8)
            m_algorithms.back()->init(frame, bboxes[i]);
        else 
            m_algorithms.back()->init(frame, Rect2d(bboxes[i]));
        //m_multiTracker.add(m_algorithms.back(), frame, Rect2d(bboxes[i]));
    }
}




int CMTracker::track(cv::Mat frame, std::vector<CObject>& trackerOutput, int frameNum)
{
    //for (auto trk : m_algorithms)
    for (int i = 0; i < m_algorithms.size(); i++) {
#ifndef TRACKER_NO_LEGACY
        bool ok = m_algorithms[i]->update(frame, m_bboxs[i]);
#else 
        cv::Rect bbox = cv::Rect(m_bboxs[i].x, m_bboxs[i].y, m_bboxs[i].width, m_bboxs[i].height);
        bool ok = m_algorithms[i]->update(frame, bbox);
#endif 

        if (ok) {
            UTILS::checkBounderies(m_bboxs[i], frame.size());
            int label = m_labels[i]; // DDEBUG 
            CObject trackObj(m_bboxs[i], frameNum, 0, DETECT_TYPE::Tracking, m_labels[i]);  

            trackerOutput.push_back(trackObj);
        }
    }

    return trackerOutput.size();
}

#if 1
//========================================================================
//  S I N G L E     T R A C K E R    C L A S S 
//========================================================================
bool CTracker::init(int type, int debugLevel, int badFramesToReset)
{

	m_trackerType = type;
	m_badFramesToReset = badFramesToReset;
	m_debugLevel = debugLevel;


	if (type >= m_trackerTypes_str.size()) {
		std::cout << "Max tracker index = " << m_trackerTypes_str.size() << "\n";
		return false;
	}

	std::cout <<  "Tracker type = " << m_trackerTypes_str[m_trackerType] << "\n";

	m_tracker =  createTrackerByName(m_trackerTypes_str[m_trackerType]);
	//cv::Ptr<cv::legacy::Tracker> m_trackerLegacy = createTrackerByName_legacy(const std::string& name)

    return true;

 }



bool CTracker::track(cv::Mat frame)
{

	if (m_bbox.width == 0 || m_bbox.height == 0) {
		//reset();
		return false;
	}

	// Update the tracking result
	bool ok = m_tracker->update(frame, m_bbox);

	// Manage end-of-life of tracker
	if (ok)
		falseDetectionLen = 0;
	else {
		falseDetectionLen++;
        std::cout << "Tracker failed (" << falseDetectionLen << ")\n";
    }

	if (m_debugLevel > 0) { // DDEBUG SHOW
		if (ok)
			rectangle(frame, m_bbox, Scalar( 255, 0, 0 ), 2, 1 ); // Tracking success : Draw the tracked object
		else
			putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2); // Tracking failure detected
		cv::Mat display = frame.clone();
		// Display tracker type on frame
		putText(display, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
		cv::putText(display, std::to_string(m_frameNum), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);


		imshow("Tracking", display);
	}


	// Check if exceeds bad detection len limits - then cancel tracker
	if (falseDetectionLen >= m_badFramesToReset)
		reset(); // set NOT active
	else
		m_frameNum++;

	return ok;

} 


void CTracker::setROI(const cv::Mat &img, cv::Rect bbox)
{
	UTILS::checkBounderies(bbox, img.size());
	m_bbox = bbox;

    if (m_bbox.x < 0)
        m_bbox.x = 0;
    if (m_bbox.y < 0)
        m_bbox.y = 0;
    if (m_bbox.x + m_bbox.width >= img.cols )
        m_bbox.width = img.cols - m_bbox.x - 1;
    if (m_bbox.y + m_bbox.height >= img.rows )
        m_bbox.height = img.rows - m_bbox.y - 1;
	m_tracker->init(img, m_bbox);
	m_frameNum++;
}


cv::Rect CTracker::setROI_GUI(cv::Mat frame)
{
        	// Define initial boundibg box
            cv::Rect bbox;

        	// Uncomment the line below to select a different bounding box
        	bbox = selectROI(frame, false);

        	// Display bounding box.
        	rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        	imshow("Tracking", frame);

        	//m_tracker->init(frame, m_bbox);

        	return bbox;
}


int CTracker::track_main(std::string videoFName, int trackerTypeInd, int skip)
{
    // List of tracker types in OpenCV 3.2
 

	m_trackerType = trackerTypeInd;
    Ptr<Tracker> tracker = createTrackerByName(m_trackerTypes_str[m_trackerType]);

	
	VideoCapture video(videoFName);
    
    // Exit if video is not opened
    if(!video.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
        
    }
    
	video.set(cv::CAP_PROP_POS_FRAMES, skip);
	
	// Read first frame
    Mat frame;
    video.read(frame);
    


	Rect bbox;


    // Run idle for  selection
    while (video.read(frame)) {
        imshow("Idle", frame);
        int k = waitKey(1);
        if(k == 's')
        	break;
        else if(k == 27)
            return 0;
    }

    video.read(frame);
    
    // Uncomment the line below to select a different bounding box
    bbox = selectROI("Select ROI", frame, false);

    // Display bounding box.
    rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
    Sleep(1000);
    cv::destroyAllWindows();

    imshow("Tracking", frame);

    tracker->init(frame, bbox);

    int minX = 999999;
    int maxX = 0;
    int minY = 999999;
    int maxY = 0;



    int frameNum = 0;
	bool ok =	false;
    while(video.read(frame))
    {

     
        // Start timer
        //double timer = (double)getTickCount();
        frameNum++;
        if (frameNum % 2 == 0)   // DDEBUG 
            continue;
        // Update the tracking result
		if (!bbox.empty())
			ok = tracker->update(frame, bbox);
                
        if (ok)
            // Tracking success : Draw the tracked object
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        else
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        
        // Display tracker type on frame
        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
        
        // Display FPS on frame
        //putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
        cv::putText(frame, std::to_string(frameNum), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
        cv::Point center = cv::Point(bbox.tl().x + bbox.br().x / 2, bbox.tl().y + bbox.br().y / 2);
        std::string text = "( " + std::to_string(center.x) + " , " + std::to_string(center.y) + " )";
        cv::putText(frame, text, Point(100, 100), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);

        minX = min(minX, center.x);
        maxX = max(maxX, center.x);
        minY = min(minY, center.y);
        maxY = max(maxY, center.y);

        int diffX = maxX - minX;
        int diffY = maxY - minY;

        // Display frame.
        imshow("Tracking", frame);
        // Exit if ESC pressed.
        int k = waitKey(1);
		if (k == 's') {
			//bool firstTime = bbox.empty();
			bbox = selectROI("selector", frame, false);
			//if (firstTime)
			tracker.release();
			tracker = createTrackerByName(m_trackerTypes_str[m_trackerType]);
			tracker->init(frame, bbox);

            cv::destroyAllWindows();
        }
        //else if(k == 27)    break;

    }
    
    return 0;

}




/*-----------------------------------------------------------------------------------------
	 Reset tracker - init for the tracker after first initializing
------------------------------------------------------------------------------------------*/

bool CTracker::init()
{
	if (m_trackerType < 0) {
		std::cout << "Missing first Tracker init, use default : " << m_trackerTypes_str[m_trackerType];
	}

	if (!m_tracker.empty())
		m_tracker.release();

	m_tracker =  createTrackerByName(m_trackerTypes_str[m_trackerType]);

	return true;
}
#endif