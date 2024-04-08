#pragma once 

#include <iostream>
#include <cmath>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <boost/circular_buffer.hpp>

#include "config.hpp"
#include "trackerBasic.hpp"

using namespace cv;
using namespace cv::dnn;


#define MAX_TRACKERS  10


class CSiamTracker {
public:
    bool init(bool GPU, float scale, int debugLevel, int badFramesToreset = MAX_BAD_DETECTION_SEC * CONSTANTS::FPS);
    bool init(bool GPU, float scale = 1.);

    void setModelsFolder(std::string pathToModel)
    {
        m_pathToModel = pathToModel;
    }

    void setNewFrame(cv::Mat frame);

    //int setROI(cv::Mat image, CObject newObj);
    int setROI(CObject newObjs);
    int setROIs(std::vector <CObject> newObjs);
    //int updateROI(std::vector <CObject> newObjs, cv::Mat image);
    //int updateROIs(std::vector <CObject> newObjs, cv::Mat image);
    void prune(std::vector <int> ids);

    bool removeROI(int trkInd);
    bool removeROIbyID(int objInd);


    //int track(cv::Mat frame, std::vector <Rect>& bboxes, int frameNum);
    int track(std::vector <CObject>& objects, int frameNum);
    int track(cv::Mat frame, std::vector <CObject>& objects, int frameNum);

    int updateROIs(std::vector <cv::Rect> yoloBoxes);

    float getScore(int ind) { return (ind < m_objectsScores[ind].size() ? m_objectsScores[ind].back() : -1); }
    int getActiveTrackerNum() { return m_activesInd.size(); }
    std::vector <int> getActiveTrackersInd() { return m_activesInd; }
    void clear()
    {
        for (auto ind : m_activesInd)
            removeROI(ind);
    }


private:
    int getFreeTracker();
    int pruneTrackers();
    // Return sequebtial bad score 
    int badScoresLen(int ind)
    {
        int badScoreLen = 0;
        for (auto it = m_objectsScores[ind].rbegin(); it != m_objectsScores[ind].rend(); ++it) {
            if (*it < TRACKER::lowScore)
                badScoreLen++;
            else
                break;
        }

        return badScoreLen;
    }


private:
    std::vector <Ptr<TrackerDaSiamRPN>> m_tracker;
    std::vector <bool> m_oks;
    //std::vector <float> m_scores;
    std::vector <CObject> m_objects;
    //std::vector < std::vector<int>> m_objectsScores;
    std::vector < boost::circular_buffer<float>> m_objectsScores;
    cv::Rect m_scaledBbox;

    std::vector <int> m_activesInd; // Inices of active trackers 

    std::string m_pathToModel = R"(c:\data\models)";
    int m_MaxTrackers = MAX_TRACKERS;
    int m_debugLevel = 0;

    float m_scale = 1.;
    cv::Mat m_frame;
    int m_frameNum = 0;
    float m_enlargeBBox = 1.2;
    bool m_trackeractive = false;

};

