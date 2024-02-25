#include <fstream>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "yolo.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;


bool m_isCuda = true;

#if 0
const std::string modelFName = R"(C:\Program Files\Bauotech\dll\algo\models\yolov8s.onnx)";
//const std::string modelFName = R"(C:\Program Files\Bauotech\dll\algo\models\yolov8n.onnx)";
const std::string classesFName = R"(C:\Program Files\Bauotech\dll\algo\models\classes.txt)";
#endif 
const std::string modelFName = "yolov8s.onnx";
//const std::string modelFName = "yolov8n.onnx";
const std::string classesFName = "classes.txt";

const std::vector<cv::Scalar> colors = { cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0) };


//const float SCORE_THRESHOLD = 0.45;
const float SCORE_THRESHOLD = 0.55;
const float PERSON_SCORE_THRESHOLD = 0.15;
const float NMS_THRESHOLD = 0.5;
const float CONFIDENCE_THRESHOLD = 0.25;

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




bool load_net_(cv::dnn::Net& net, bool is_cuda)
{
    auto result = cv::dnn::readNet(modelFName);
    //cv::dnn::readNetFromONNX

    if (result.empty())
        return false;

    if (is_cuda)
    {
        std::cout << "Attempty to use CUDA\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        std::cout << "Running on CPU\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    net = result;

    return true;
}


vector<Mat> pre_process(Mat& input_image, Net& net)
{
    // Convert to blob.
    Mat blob;
    blobFromImage(input_image, blob, 1. / 255., Size(YOLO_INPUT_WIDTH, YOLO_INPUT_HEIGHT), Scalar(), true, false);
    //blobFromImage(input_image, blob, 1. / 255., Size(YOLO_INPUT_WIDTH, YOLO_INPUT_HEIGHT), cv::Scalar(), true, true);

    net.setInput(blob);
    vector<Mat> outputs;
    if (!blob.empty()) {
        net.forward(outputs, net.getUnconnectedOutLayersNames());
    }

    return outputs;
}


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


        if (class_id.x == 0)
            int debug = 10;

        float scoreThreshold = (class_id.x == 0) ? PERSON_SCORE_THRESHOLD : SCORE_THRESHOLD;
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

        data += dimensions;
    }
    // Perform Non-Maximum Suppression and draw predictions.
    vector<int> indices;
    NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
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



/*--------------------------------------------------------------------------------------------------------------------------
        Main YDetection function:
  --------------------------------------------------------------------------------------------------------------------------*/

bool CYolo8::load_net(bool is_cuda)
{
    //std::filesystem::path cwd = std::filesystem::current_path();

    //auto result = cv::dnn::readNet(m_modelFolder + "/yolov5s.onnx");
    auto result = cv::dnn::readNet(m_modelFolder + modelFName);

    if (result.empty())
        return false;

    //auto result = cv::dnn::readNet("config_files/yolov5n.onnx");
    if (is_cuda) {
        //Beep(1100, 200);
        std::cout << "Attempty to use CUDA\n";
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


        if (class_id.x == 0 && maxClassScore < PERSON_SCORE_THRESHOLD && maxClassScore && PERSON_SCORE_THRESHOLD > PERSON_SCORE_THRESHOLD)
            int debug = 10;

        float scoreThreshold = (class_id.x == 0) ? PERSON_SCORE_THRESHOLD : SCORE_THRESHOLD;
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
        else {
            if (maxClassScore > 0.2)
                std::cout << " drop low score " << maxClassScore << "\n";
        }
        data += dimensions; // 85 in 5?
    }
    // Perform Non-Maximum Suppression and draw predictions.
    vector<int> indices;
    NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    for (int i = 0; i < nms_result.size(); i++) {
        int idx = nms_result[i];
        YDetection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }

}