//#include <Windows.h> // Beep()
#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "yolo.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;


bool m_isCuda = true;

const std::string classesFName = "classes.txt";

const std::vector<cv::Scalar> colors = { cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0) };

// Text parameters.
const float FONT_SCALE = 0.7;
const int FONT_FACE = FONT_HERSHEY_SIMPLEX;
const int THICKNESS = 1;

// Colors.
Scalar BLACK = Scalar(0, 0, 0);
Scalar BLUE = Scalar(255, 178, 50);
Scalar YELLOW = Scalar(0, 255, 255);
Scalar RED = Scalar(0, 0, 255);

void draw_label(Mat& input_image, string label, int left, int top)
{
    // Display the label at the top of the bounding box.
    int baseLine;
    Size label_size = getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
    top = max(top, label_size.height);
    // Top left corner.
    Point tlc = Point(left, top);
    // Bottom right corner.
    Point brc = Point(left + label_size.width, top + label_size.height + baseLine);
    // Draw white rectangle.
    rectangle(input_image, tlc, brc, BLACK, FILLED);
    // Put the label on the black rectangle.
    putText(input_image, label, Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, YELLOW, THICKNESS);
}



std::vector<std::string> load_class_list(std::string classesFName)
{
    std::vector<std::string> class_list;
    std::ifstream ifs(classesFName);

    if (!ifs.is_open())
        return class_list; // empty

    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    return class_list;
}





vector<Mat> pre_process(Mat& input_image, Net& net)
{

    // DDEBUG - enhance image :   -------------------------------------
    cv::Mat gray,imgEnhanced;
    if (1) {
        if (input_image.channels() > 1)
            cv::cvtColor(input_image, gray, cv::COLOR_BGR2GRAY);
        else
            gray = input_image;
        
        

		// if (0)  cv::adaptiveThreshold(gray, input_image, 255, /*cv::ADAPTIVE_THRESH_MEAN_C*/ cv::ADAPTIVE_THRESH_GAUSSIAN_C ,  cv::THRESH_BINARY, /*cv::THRESH_BINARY_INV */ 51, 0);
        /*
        if (0)
			cv::equalizeHist(gray, input_image); //DDEBUG enhance
		
		if (0) // DDEBUG
			cv::GaussianBlur(gray, imgEnhanced, cv::Size(5, 5), 3.0, 3.0);
		*/
    }
    else
        imgEnhanced = input_image;
    //----------------------------------------------------------------- 
    // Convert to blob.
    Mat blob;
    bool doCrop = false;
    blobFromImage(input_image, blob, 1. / 255., Size(YOLO_INPUT_WIDTH, YOLO_INPUT_HEIGHT), Scalar(), true, doCrop);
    //blobFromImage(imgEnhanced, blob, 1. / 255., Size(YOLO_INPUT_WIDTH, YOLO_INPUT_HEIGHT), Scalar(), true, doCrop, CV_8U);
    
    net.setInput(blob);
    vector<Mat> outputs;
    if (!blob.empty()) {
        net.forward(outputs, net.getUnconnectedOutLayersNames());
    }

    return outputs;
}





/*--------------------------------------------------------------------------------------------------------------------------
        Main YDetection function:
  --------------------------------------------------------------------------------------------------------------------------*/

bool CYolo8::load_net(bool is_cuda)
{
    //auto result = cv::dnn::readNetFromONNX(m_modelFolder + YOLO_MODEL_NAME); // DDEBUG DDEBUG 
    auto result = cv::dnn::readNet(m_modelFolder + YOLO_MODEL_NAME);

    if (result.empty())
        return false;
    if (is_cuda) {
        //Beep(1100, 200);
        std::cout << "Attempt to use CUDA\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);// Note the FP16 !!!
        //result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);// Note the FP16 !!!
    }
    else {
        std::cout << "Running on CPU\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    m_net = result;

    return true;
}


bool CYolo8::init(std::string modelFolder, bool is_cuda)
{
#ifdef USE_CUDA
    m_isCuda = is_cuda;
#else
    m_isCuda = false;
#endif 
    m_modelFolder = modelFolder;
    m_modelFolder.append("/");
    m_class_list = load_class_list(m_modelFolder + classesFName);
    if (m_class_list.empty()) {
        std::cout << "Error : Empty class list - ML detection won't work! \n";
        return false;
    }

    return load_net(m_isCuda);
    //return load_net_(m_net, m_isCuda);
}


void CYolo8::detect(cv::Mat& frame, std::vector<YDetection>& output)
{
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;
    vector<Mat> detections;

    detections = pre_process(frame, m_net);

    if (detections.size() > 0)
        int debug = 10;

    // TEMP CONVERT PARAMS
    cv::Mat input_image = frame;
    std::vector<cv::Mat>  outputs = detections;

    // Initialize vectors to hold respective outputs while unwrapping     detections.
    int rows = outputs[0].size[2];
    int dimensions = outputs[0].size[1];

    outputs[0] = outputs[0].reshape(1, dimensions);
    cv::transpose(outputs[0], outputs[0]);

    float* data = (float*)outputs[0].data;

    // Resizing factor.
    float x_factor = input_image.cols / YOLO_INPUT_WIDTH;
    float y_factor = input_image.rows / YOLO_INPUT_HEIGHT;

    // Iterate through  detections.
    //cout << "num detections  : " << rows << " " << dimensions << endl;
    for (int i = 0; i < rows; ++i)
    {
        float* classes_scores = data + 4;

        cv::Mat scores(1, m_class_list.size(), CV_32FC1, classes_scores);
        cv::Point class_id;
        double maxClassScore;

        minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);


        float scoreThreshold = (class_id.x == 0) ? YOLO_PERSON_CONFIDENCE_THRESHOLD : YOLO_CONFIDENCE_THRESHOLD;
        if (maxClassScore > scoreThreshold)
        {
            confidences.push_back(maxClassScore);
            class_ids.push_back(class_id.x);

            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];

            int left = int((x - 0.5 * w) * x_factor);
            int top = int((y - 0.5 * h) * y_factor);

            int width = int(w * x_factor);
            int height = int(h * y_factor);

            boxes.push_back(cv::Rect(left, top, width, height));
        }
        else
        {
            // DDEBUG DDEBUG DDEBUG 
            if (class_id.x == 0 && maxClassScore > 0.1)
                std::cout << "DEBUG INFO : Low Person score " << maxClassScore << "\n";
        }

        data += dimensions; // 85 in 5?
    }
    // Perform Non-Maximum Suppression .
    /*
    vector<int> indices;
    NMSBoxes(boxes, confidences, YOLO_CONFIDENCE_THRESHOLD, NMS_THRESHOLD, indices);
    */

    std::vector<int> nms_result;
    float scoreThreshold = YOLO_PERSON_CONFIDENCE_THRESHOLD; // YOLO_PERSON_CONFIDENCE_THRESHOLD;
    cv::dnn::NMSBoxes(boxes, confidences, scoreThreshold, NMS_THRESHOLD, nms_result);
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        YDetection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);

    }
}

/*-------------------------------------------------------------------------
* Optimize rect to have at least 640 * 480 size
* Missing : Force this dimension ratio 
-------------------------------------------------------------------------*/
cv::Rect CYolo8::optimizeRect(cv::Rect r, cv::Size dim)
{

    if (dim.width <= 640 && dim.height <= 480) 
        return (cv::Rect(0, 0, dim.width, dim.height));
    else
        return r;

#if 0
    cv::Rect finalRect = r;
    // Here frame dim > 640 * 480
    if (r.width < 640) {
            finalRect.x -= 640 / 2;
            finalRect.width += 640 / 2;
            // Increase width
            if (finalRect.x < 0) {
                finalRect.width += -finalRect.x;
                finalRect.x = 0;
            }
            else if (r.width > 640) {
                finalRect.width = 640;
                finalRect.x -= r.width - 640;
            }

            // Increase height
            if (finalRect.y < 0) {
                finalRect.height += -finalRect.y;
                finalRect.y = 0;
            }
            else if (r.height > 480) {
                finalRect.height = 480;
                finalRect.y -= r.height - 640;
            }

        return finalRect;

    }
#endif 

}

/*------------------------------------------------------------------------------

int test_yolo(cv::Mat frame)
{
    CYolo8 _yolo;
    bool _isCuda = true;
    std::string modelFolder = "";
    std::vector<YDetection> _Yolotput;


    if (!_yolo.init(modelFolder, _isCuda)) {
        std::cout << "Cant init YOLO net , quit \n";
        return -1;
    }
    _Yolotput.clear();
    _yolo.detect(frame, _Yolotput);



}
*/
//-----------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------
#if 0
Mat post_process(Mat& input_image, vector<Mat>& outputs, const vector<string>& class_name)
{
    // Initialize vectors to hold respective outputs while unwrapping     detections.
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;

    int rows = outputs[0].size[2];
    int dimensions = outputs[0].size[1];

    outputs[0] = outputs[0].reshape(1, dimensions);
    cv::transpose(outputs[0], outputs[0]);

    float* data = (float*)outputs[0].data;

    // Resizing factor.
    float x_factor = input_image.cols / YOLO_INPUT_WIDTH;
    float y_factor = input_image.rows / YOLO_INPUT_HEIGHT;

    // Iterate through  detections.
    //cout << "num detections  : " << rows << " " << dimensions << endl;
    for (int i = 0; i < rows; ++i)
    {
        float* classes_scores = data + 4;

        cv::Mat scores(1, class_name.size(), CV_32FC1, classes_scores);
        cv::Point class_id;
        double maxClassScore;

        minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);


        //if (class_id.x == 0)  int debug = 10;

        float scoreThreshold = (class_id.x == 0) ? YOLO_PERSON_CONFIDENCE_THRESHOLD : YOLO_CONFIDENCE_THRESHOLD;
        if (maxClassScore > scoreThreshold)
        {
            confidences.push_back(maxClassScore);
            class_ids.push_back(class_id.x);

            float x = data[0];
            float y = data[1];
            float w = data[2];
            float h = data[3];

            int left = int((x - 0.5 * w) * x_factor);
            int top = int((y - 0.5 * h) * y_factor);

            int width = int(w * x_factor);
            int height = int(h * y_factor);

            boxes.push_back(cv::Rect(left, top, width, height));
        }
        else
        {
            // DDEBUG DDEBUG DDEBUG 
            if (class_id.x == 0 && maxClassScore > 0.1)
                std::cout << "DEBUG INFO : Low Person score " << maxClassScore << "\n";
        }

        data += dimensions;
    }
    // Perform Non-Maximum Suppression and draw predictions.
    vector<int> indices;
    NMSBoxes(boxes, confidences, YOLO_CONFIDENCE_THRESHOLD, NMS_THRESHOLD, indices);
    //cout << "num detections finally : "<< indices.size() <<endl;
    for (int i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        int left = box.x;
        int top = box.y;
        int width = box.width;
        int height = box.height;
        // Draw bounding box.
        rectangle(input_image, Point(left, top), Point(left + width, top + height), BLUE, 3 * THICKNESS);
        // Get the label for the class name and its confidence.
        string label = format("%.2f", confidences[idx]);
        label = class_name[class_ids[idx]] + ":" + label;
        // Draw class labels.
        draw_label(input_image, label, left, top);
    }
    return input_image;
}
#endif 
