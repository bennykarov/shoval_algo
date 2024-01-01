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

#include "utils.hpp"
#include "config.hpp"
#include "trackerBasic.hpp"


using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( ( std::ostringstream() << std::dec << x ) ).str()


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


#if 1
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

#if 0
    // Run idle for  selection
    while(video.read(frame)) {
        imshow("Idle", frame);
        int k = waitKey(1);
        if(k == 's')
        	break;
        else if(k == 27)
            return 0;
    }

    
    // Uncomment the line below to select a different bounding box
    bbox = selectROI(frame, false);

    // Display bounding box.
    rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
    imshow("Tracking", frame);

    tracker->init(frame, bbox);

#endif 

    int frameNum = 0;
	bool ok =	false;
    while(video.read(frame))
    {
     
        // Start timer
        //double timer = (double)getTickCount();
        frameNum++;
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
        cv::putText(frame, std::to_string(frameNum), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        // Display frame.
        imshow("Tracking", frame);
        // Exit if ESC pressed.
        int k = waitKey(1);
		if (k == 's') {
			bool firstTime = bbox.empty();
			bbox = selectROI("selector", frame, false);
			if (firstTime)
				tracker->init(frame, bbox);
		}
        else if(k == 27)
            break;

    }
    
    return 0;

}
#endif



/*-----------------------------------------------------------------------------------------
	 Reset tracker - init for the tracker after first initializing
------------------------------------------------------------------------------------------*/

bool CTracker::init()
{
	if (m_trackerType < 0) {
		std::cout << "Missing first Tracker init \n";
		return false;
	}

	if (!m_tracker.empty())
		m_tracker.release();

	m_tracker =  createTrackerByName(m_trackerTypes_str[m_trackerType]);

	return true;
}
