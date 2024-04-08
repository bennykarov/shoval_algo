// DaSiamRPN tracker.
// Original paper: https://arxiv.org/abs/1808.06048
// Link to original repo: https://github.com/foolwood/DaSiamRPN
// Links to onnx models:
// - network:     https://www.dropbox.com/s/rr1lk9355vzolqv/dasiamrpn_model.onnx?dl=0
// - kernel_r1:   https://www.dropbox.com/s/999cqx5zrfi7w4p/dasiamrpn_kernel_r1.onnx?dl=0
// - kernel_cls1: https://www.dropbox.com/s/qvmtszx5h339a0w/dasiamrpn_kernel_cls1.onnx?dl=0

#include <iostream>
#include <cmath>

#include "opencv2/opencv.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>



#include <stdarg.h>
#include <string.h>
#include <vector>
#include <codecvt>

#include <algorithm>

#include "config.hpp"
#include "utils.hpp"

#include "dasiamrpn_tracker.hpp"

using namespace cv;
using namespace cv::dnn;

const char *keys =
        "{ help     h  |   | Print help message }"
        "{ input    i  |   | Full path to input video folder, the specific camera index. (empty for camera 0) }"
        "{ net         | dasiamrpn_model.onnx | Path to onnx model of net}"
        "{ kernel_cls1 | dasiamrpn_kernel_cls1.onnx | Path to onnx model of kernel_r1 }"
        "{ kernel_r1   | dasiamrpn_kernel_r1.onnx | Path to onnx model of kernel_cls1 }"
        "{ backend     | 0 | Choose one of computation backends: "
                            "0: automatically (by default), "
                            "1: Halide language (http://halide-lang.org/), "
                            "2: Intel's Deep Learning Inference Engine (https://software.intel.com/openvino-toolkit), "
                            "3: OpenCV implementation, "
                            "4: VKCOM, "
                            "5: CUDA },"
        "{ target      | 0 | Choose one of target computation devices: "
                            "0: CPU target (by default), "
                            "1: OpenCL, "
                            "2: OpenCL fp16 (half-float precision), "
                            "3: VPU, "
                            "4: Vulkan, "
                            "6: CUDA, "
                            "7: CUDA fp16 (half-float preprocess) }"
;


/*-------------------------------------------------------------------------------------------------------
 *                   U  T  I  L  S  
 * -------------------------------------------------------------------------------------------------------*/
cv::Rect scaleBBox_(cv::Rect rect, float scale)
{
    cv::Rect sBBox;

    sBBox.width = int((float)rect.width * scale);
    sBBox.height = int((float)rect.height * scale);
    sBBox.x = int((float)rect.x * scale);
    sBBox.y = int((float)rect.y * scale);

    return sBBox;
}

cv::Rect2f resizeBBox_(cv::Rect2f rect, float scale)
{
    cv::Rect2f sBBox;

    sBBox = rect;
    float  wDiff = rect.width * (1. - scale);
    float hDiff = rect.height * (1. - scale);
    sBBox.width -= wDiff;
    sBBox.height -= hDiff;
    sBBox.x += wDiff / 2.;
    sBBox.y += hDiff / 2.;

    return sBBox;

}

//----------------------------------------------------------------------------------------------
// Find Rects1 (trackerBoxes) that is similar (overlapping) to Rects2 (yoloBoxes)
// return thier Indices list
//----------------------------------------------------------------------------------------------
std::vector <tuple<int, int>>  findDuplicated(std::vector <cv::Rect>  trackerBoxes, std::vector <cv::Rect> yoloBoxes)
{
    float nearDistance = 20;
    float nearRelativeDist = 0.3;
    float similarDimRatio = 0.6;
    float similarOverlappingRatio = 0.5;

    std::vector <int> trackRemoveIndices, yoloRemoveIndices;
    std::vector <tuple<int, int>> matchedIndices;


    for (int i = 0; i < trackerBoxes.size(); i++)
        for (int j = 0; j < yoloBoxes.size(); j++) {
            cv::Rect box1 = trackerBoxes[i];
            cv::Rect box2 = yoloBoxes[j];


            if (OverlappingRatio2(box1, box2) > similarOverlappingRatio) {
                trackRemoveIndices.push_back(i);
                yoloRemoveIndices.push_back(j);
                matchedIndices.push_back(std::make_tuple(i, j));
                j = 1000; continue;// out of yoloBoxes loop

            }


            float dimRatrio = 0.6; 

            if (dimRatrio < similarDimRatio)
                j = 1000; continue;// out of yoloBoxes loop

            float dist = distance(box1, box2);
            float dim = (float(box1.width + box2.width) / 2. + float(box1.height + box2.height) / 2.) / 2.;
            float relativeDist = dist / dim;
            if (relativeDist > nearRelativeDist)
                j = 1000; continue;// out of yoloBoxes loop

            // At his point hboxes defined as overlapped
            trackRemoveIndices.push_back(i);
            yoloRemoveIndices.push_back(j);
        }

    return matchedIndices;

}

/*-------------------------------------------------------------------------------------------------------
 *                   CSiamTracker C A L S S  
 * -------------------------------------------------------------------------------------------------------*/

bool CSiamTracker::init(bool GPU, float scale)
{

    m_scale = scale;
        //std::string inputName;// = parser.get<String>("input");
    std::string net;// = parser.get<String>("net");
    std::string kernel_cls1;// = parser.get<String>("kernel_cls1");
    std::string kernel_r1;// = parser.get<String>("kernel_r1");
    int backend;// = parser.get<int>("backend");
    int target;// = parser.get<int>("target");


    //inputName = R"(C:\Data\office\office_multiCars.ts)";
    net = m_pathToModel + "\\dasiamrpn_model.onnx"; // R"(C:\Data\models\dasiamrpn_model.onnx)";
    kernel_cls1 = m_pathToModel + "\\dasiamrpn_kernel_cls1.onnx";
    kernel_r1 = m_pathToModel + "\\dasiamrpn_kernel_r1.onnx";
    backend = cv::dnn::DNN_BACKEND_CUDA; // 5; //  CUDA
    target = cv::dnn::DNN_TARGET_CUDA_FP16; // 7;  //  6=CUDA OR 7=CUDA 16fp 

    try
    {
        TrackerDaSiamRPN::Params params;
        params.model = samples::findFile(net);
        params.kernel_cls1 = samples::findFile(kernel_cls1);
        params.kernel_r1 = samples::findFile(kernel_r1);
        params.backend = backend;
        params.target = target;
        //m_tracker = TrackerDaSiamRPN::create(params);

        for (int i = 0; i < m_MaxTrackers; i++) {
            m_tracker.push_back(TrackerDaSiamRPN::create(params));
            CObject trackObj(cv::Rect(), m_frameNum, 0, DETECT_TYPE::Tracking, -1);  
            m_oks.push_back(false);
            m_objects.push_back(CObject());
            m_objectsScores.push_back(boost::circular_buffer<float>(TRACKER::scoreHistoryLen));
        }
    }
    catch (const cv::Exception& ee)
    {
        std::cerr << "Exception: " << ee.what() << std::endl;
        std::cout << "Can't load the network by using the following files:" << std::endl;
        std::cout << "siamRPN : " << net << std::endl;
        std::cout << "siamKernelCL1 : " << kernel_cls1 << std::endl;
        std::cout << "siamKernelR1 : " << kernel_r1 << std::endl;
        return 2;
    }

    /*
    const std::string winName = "DaSiamRPN";
    namedWindow(winName, WINDOW_AUTOSIZE);
    */

    return true;

}

bool CSiamTracker::init(bool GPU, float scale, int debugLevel, int badFramesToreset)
{
    m_scale = scale;
    m_debugLevel = debugLevel;
    init(scale);

    m_trackeractive = true;

    return true;
}


/*-------------------------------------------------------------------------
* Muist come BEFORE all tracking processes (setRIO(), Track()) 
* Due to performance issues (resize only once 
 -------------------------------------------------------------------------*/
void CSiamTracker::setNewFrame(cv::Mat frame)
{
    if (!m_trackeractive)
        return;

    cv::resize(frame, m_frame, cv::Size(0, 0), m_scale, m_scale);

}

int CSiamTracker::track(cv::Mat frame_, std::vector <CObject>& objects, int frameNum)
{
    if (!m_trackeractive)
        return 0;

    cv::resize(frame_, m_frame, cv::Size(0, 0), m_scale, m_scale);
    return track(objects, frameNum);
}

int CSiamTracker::track(std::vector <CObject>& objects, int frameNum)
{
    std::fill(m_oks.begin(), m_oks.end(), false);

    m_frameNum = frameNum;

    //for (int i = 0; i < m_tracker.size(); i++) {
    for (auto i : m_activesInd) {
        m_scaledBbox = scaleBBox_(m_objects[i].m_bbox, m_scale);
        m_oks[i] = m_tracker[i]->update(m_frame, m_scaledBbox);
                if (0) // DDEBUG CHECKS:
                {
                    if (!m_oks[i])   int debug = 10;
                    cv::Rect debugBox = m_scaledBbox;
                    bool badRect = UTILS::checkBounderies(debugBox, m_frame.size());
                    if (badRect) int debug = 11;
                }
        m_objects[i].m_bbox = scaleBBox_(m_scaledBbox, 1. / m_scale);
        m_objects[i].m_confidance = m_oks[i] ? m_tracker[i]->getTrackingScore() : 0;
        m_objects[i].m_detectionType = DETECT_TYPE::Tracking;
        m_objects[i].m_frameNum = frameNum;

        m_objectsScores[i].push_back(m_objects[i].m_confidance); // ??? is needed ?
    }

    objects.clear();


#if 0
    // Remove untracked objects or with a bad score (low score 'TRACKER::maxHidden' times sequentially)
    for (auto i : m_activesInd) {
        //if (!m_oks[i] ||  badScoresLen(i) >= TRACKER::maxHidden) {
        float scoreDebug = m_tracker[i]->getTrackingScore();
        if (!m_oks[i] || m_objectsScores[i].back() < TRACKER::lowScore) {
            m_oks[i] = false;
			removeROI(i);
            std::cout << "remove objID " << m_objects[i].m_ID << "\n";
            }
    }
#endif 

    // Ad to return objects
    for (auto i : m_activesInd)
        if (m_oks[i])
            objects.push_back(m_objects[i]);


    return objects.size();
}


/*--------------------------------------------------------------------------------
* set ROI to tracker.
* If newObj.m_ID exist (=tracker is active), update its ROI
* If not - find free Tracker and init with new ROI
*   Note that in that case , it is possible that there is no free tracker be set
--------------------------------------------------------------------------------*/
int CSiamTracker::setROIs(std::vector <CObject> newObjs)
{
    int roiSet = 0;


    for (auto& newObj : newObjs) {
        int trkID = setROI(newObj);
        if (trkID >= 0)
            roiSet++;
        else
            int debug = 10;
    }
#if 0
    for (auto& obj : newObjs) {
        int trkInd = getFreeTracker();
        if (trkInd >= 0) {
            m_objects[trkInd] = obj;
            m_scaledBbox = scaleBBox_(obj.m_bbox, m_scale);

            m_tracker[trkInd]->init(m_frame, m_scaledBbox);
            obj.m_bbox = scaleBBox_(m_scaledBbox, 1. / m_scale);
            m_activesInd.push_back(trkInd);
            roiCreated++;
        }
        else
            std::cout << "Cant init tracker (too many) \n";
    }

#endif 

    return roiSet;
}

/*--------------------------------------------------------------------------------
* set ROI to tracker.
* If newObj.m_ID exist(= tracker is active), update its ROI
* If not - find free Tracker and init with new ROI
* Note that in that case, it is possible that there is no free tracker be set
-------------------------------------------------------------------------------- */

int CSiamTracker::setROI(CObject obj)
{
    int trkInd = -1;

    int id = obj.m_ID;
    //  Search if ID is among the active trackers :
    //auto it = std::find_if(m_objects.begin(), m_objects.end(),    [&id](const CObject& obj) { return obj.m_ID == id; });
    for (auto ind : m_activesInd)
        if (m_objects[ind].m_ID == obj.m_ID) {
            trkInd = ind;
            break;
        }

    if (trkInd < 0) {// ID not tracked - create new (=find free one):
        trkInd = getFreeTracker();  // find free tracker ("new" ROI)
        m_activesInd.push_back(trkInd);
        std::cout << "SIAM TRACKER: total=" << m_activesInd.size() << "(Add ID = " << m_objects[trkInd].m_ID << ")\n";

        if (m_activesInd.size() > m_MaxTrackers)
            int debbug = 10; // ERROR
    }


    if (trkInd < 0) {
        std::cout << "Cant init tracker (too many) \n";
        return  trkInd;
    }

	m_objects[trkInd] = obj;
	m_scaledBbox = scaleBBox_(obj.m_bbox, m_scale);

    if (m_enlargeBBox != 1.) 
        m_scaledBbox = resizeBBox_(m_scaledBbox, m_enlargeBBox);// enlarge to better tracking 

	m_tracker[trkInd]->init(m_frame, m_scaledBbox);
	obj.m_bbox = scaleBBox_(m_scaledBbox, 1. / m_scale);

	return true;
}



#if 0
int CSiamTracker::setROI(cv::Mat image_, CObject newObj)
{
    cv::resize(image_, m_frame, cv::Size(0, 0), m_scale, m_scale);

    int ind = getFreeTracker();
    if (ind >= 0) {
        m_tracker[ind]->init(m_frame, newObj.m_bbox);
        m_activesInd.push_back(ind);
    }
    else
        std::cout << "Cant init tracker (too many) \n";

    return ind;
}
int CSiamTracker::updateROIs(std::vector <CObject> toUpdateObjs, cv::Mat image_)
{
    int roiUpdated= 0;

    if (toUpdateObjs.size() == 0)
        return 0;

    cv::resize(image_, m_frame, cv::Size(0, 0), m_scale, m_scale);

    for (auto& obj : toUpdateObjs) {
        int id = obj.m_ID;
        //  Find coresponded tracker object by ID
        auto it = std::find_if(m_objects.begin(), m_objects.end(),
            [&id](const CObject& obj) { return obj.m_ID == id; });

        if (it == m_objects.end())
            continue;

        int ind = std::distance(m_objects.begin(), it);

        removeROI(ind);

        // Update tracker (=init())
        //m_tracker[ind]->init(m_frame, obj.m_bbox);

        roiUpdated++;
    }

    setROIs(toUpdateObjs);

    return roiUpdated;
}
#endif 

/*------------------------------------------------------------
* Prune using obj.m_ID (recieved from Concluder())
 ------------------------------------------------------------*/
void CSiamTracker::prune(std::vector <int> objIDs)
{
    for (auto id : objIDs)
        removeROIbyID(id);
}


/*----------------------------------------------
* This function just halt tracker 
* Remove the ROI info and remove from Active tracker list
 ----------------------------------------------*/
bool CSiamTracker::removeROI(int trkInd)
{
    // erase the i element
    auto it = std::find(m_activesInd.begin(), m_activesInd.end(), trkInd);
    if (it != m_activesInd.end()) {
        m_activesInd.erase(it);
        m_objectsScores[trkInd].clear();

        std::cout << "SIAM Tracker: total=" << m_activesInd.size() << "(remove ID = " << m_objects[trkInd].m_ID <<  ")\n";

        return true;
    }

    return false;
}

bool CSiamTracker::removeROIbyID(int objID)
{
    if (objID == 3)
        int debug = 10;
    // erase the i element
    for (auto ind : m_activesInd) 
        if (m_objects[ind].m_ID == objID) {
            removeROI(ind);
            return true;
    }

    return false;
}



//-----------------------------------------------------
// Return the index of the first free (halt) tracker 
// Check that ind i is not occupied
//-----------------------------------------------------
int CSiamTracker::getFreeTracker()
{
    for (int i = 0; i < m_tracker.size(); i++) {
        auto it = std::find(m_activesInd.begin(), m_activesInd.end(), i);
        if (it == m_activesInd.end())
            return i;
    }

    std::cout << "CSiamTracker  : Cant find free tracker, : Tracker ecceeds it limit !!!\n";
    return -1;

    //CHECK_exception(false, "CSiamTracker : Tracker ecceeds it limit !!!" );
}

int CSiamTracker::pruneTrackers()
{
    int prunerd = 0;

    return  prunerd;
}

//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================
//===========================================================================================================================



int run(std::string inputName)
{
    CSiamTracker   m_tracker;
    int actualTrackers = 4;
    int frameNum = 0;
    std::vector <cv::Rect> bboxs;


    //int TrackerType = 8;
    float scale = 0.4;
    int debugLevel = 1;
    m_tracker.init(scale, debugLevel);

    const std::string winName = "DaSiamRPN";
    namedWindow(winName, WINDOW_AUTOSIZE);

    // Open a video file or an image file or a camera stream.
    VideoCapture cap;

    if (inputName.empty() || (isdigit(inputName[0]) && inputName.size() == 1))
    {
        int c = inputName.empty() ? 0 : inputName[0] - '0';
        std::cout << "Trying to open camera #" << c << " ..." << std::endl;
        if (!cap.open(c))
        {
            std::cout << "Capture from camera #" << c << " didn't work. Specify -i=<video> parameter to read from video file" << std::endl;
            return 2;
        }
    }
    else if (inputName.size())
    {
        inputName = samples::findFileOrKeep(inputName);
        if (!cap.open(inputName))
        {
            std::cout << "Could not open: " << inputName << std::endl;
            return 2;
        }
    }

    // Read the first image.
    Mat image, image_;
    cap >> image_;
    if (0) 
        cv::resize(image_, image, cv::Size(0, 0), scale, scale);
    else 
        image = image_;

    if (image.empty())
    {
        std::cerr << "Can't capture frame!" << std::endl;
        return 2;
    }

    Mat image_select = image.clone();
    cv::putText(image_select, "Select initial bounding box you want to track.", Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
    cv::putText(image_select, "And Press the ENTER key.", Point(0, 35), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));

    // Get ROIs from user
    std::vector <CObject> objectList;
    for (int i = 0; i < actualTrackers; i++) {
        Rect selectRect = selectROI(winName, image_select);
        std::cout << "ROI=" << selectRect << std::endl;
        CObject newObj;
        newObj.m_bbox = selectRect;
        objectList.push_back(newObj);
        //int trkInd = m_tracker.setROI(image, newObj);
    }
    // multi ROIs setting
    m_tracker.setROIs(objectList);

    TickMeter tickMeter;
    Mat render_image;
    std::vector <CObject> trackedObjects;

    //------------------------
    //------------------------
    // VIdeo frames loop:
    //------------------------
    //------------------------
    for (int count = 0; ; ++count)
    {
        cap >> image_;
        frameNum++;
        if (1)
            if (frameNum % 2 == 0)
                continue; // DDEBUG TEST 
        if (0)
            cv::resize(image_, image, cv::Size(0, 0), scale, scale);
        else
            image = image_;


        if (image.empty())
        {
            std::cerr << "Can't capture frame " << count << ". End of video stream?" << std::endl;
            break;
        }


        tickMeter.start();


        render_image = image.clone();

        //int objectsDetected = m_tracker.track(image, bboxs, frameNum);

        trackedObjects.clear();
        bboxs.clear();
        int objectsDetected = m_tracker.track(image, trackedObjects, frameNum);
        for (auto obj : trackedObjects)
            bboxs.push_back(obj.m_bbox);

        tickMeter.stop();

        for (int i = 0; i < bboxs.size(); i++) {
            //if (m_oks[i])
            {
                rectangle(render_image, bboxs[i], Scalar(0, 255, 0), 2);

                std::string timeLabel = format("Inference time: %.2f ms", tickMeter.getTimeMilli());
                float score = m_tracker.getScore(i);
                //if (score < 0.9)  int debug = 10;
                std::string scoreLabel = format("Score: %f", score ); //  m_scores[i]);
                std::cout << "(" << i << ")" << scoreLabel << "\n";
                int leftSide = render_image.cols * 3 / 4;
                /*
                cv::putText(render_image, timeLabel, Point(leftSide, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
                cv::putText(render_image, scoreLabel, Point(leftSide, 35), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
                */
            }
        }

        std::cout << "frame " << count <<
            //": predicted score=" << score <<
            //"  rect=" << rect <<
            "  time=" << tickMeter.getTimeMilli() << "ms" <<
            std::endl;



        imshow(winName, render_image);

        tickMeter.reset();

        int c = waitKey(1);
        if (c == 27 /*ESC*/)
            break;
    }

    std::cout << "Exit" << std::endl;
    return 0;
}
int main_siamRPN(int argc, char **argv)
{
    try
    {
        std::string  videoFName(R"(C:\Data\office\office_multiCars.ts)");
        if (argc > 1)
            videoFName = argv[1];
        return run(videoFName);
        //return siam_tracker.run(R"(C:\Data\office\office_multiCars.ts)");
    }
    catch (const std::exception& e)
    {
        std::cerr << "FATAL: C++ exception: " << e.what() << std::endl;
        return 1;
    }
}
