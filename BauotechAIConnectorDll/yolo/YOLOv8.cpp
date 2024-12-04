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
    //auto result = cv::dnn::readNetFromONNX(m_modelFolder + YOLO_MODEL_NAME);  
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

                


        //float scoreThreshold = (class_id.x == 0) ? YOLO_PERSON_CONFIDENCE_THRESHOLD : YOLO_CONFIDENCE_THRESHOLD;
        if (maxClassScore >= YOLO_MIN_CONFIDENCE)
        {
            if (class_id.x == 0)
                int debug = 10;


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

        data += dimensions; // 85 in 5?
    }
    // Perform Non-Maximum Suppression .
    /*
    vector<int> indices;
    NMSBoxes(boxes, confidences, YOLO_CONFIDENCE_THRESHOLD, NMS_THRESHOLD, indices);
    */

    std::vector<int> nms_result;
    //float scoreThreshold = YOLO_PERSON_CONFIDENCE_THRESHOLD; // YOLO_PERSON_CONFIDENCE_THRESHOLD;
    float scoreThreshold = YOLO_MIN_CONFIDENCE;
    cv::dnn::NMSBoxes(boxes, confidences, scoreThreshold, NMS_THRESHOLD, nms_result);
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        YDetection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
        if (result.class_id == 0)   std::cout << "DDEBUG DDEBUG : Person x = " << result.box.br() << "detected with confidence " << result.confidence << "\n";
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

