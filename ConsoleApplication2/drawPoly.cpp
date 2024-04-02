//============================================================================
// ConsoleApplication2.cpp : Shoval_sc test app
//============================================================================

#ifdef _DEBUG
#pragma comment(lib, "opencv_core470d.lib")
#pragma comment(lib, "opencv_highgui470d.lib")
#pragma comment(lib, "opencv_video470d.lib")
#pragma comment(lib, "opencv_videoio470d.lib")
#pragma comment(lib, "opencv_imgcodecs470d.lib")
#pragma comment(lib, "opencv_imgproc470d.lib")
#else
#pragma comment(lib, "opencv_core470.lib")
#pragma comment(lib, "opencv_highgui470.lib")
#pragma comment(lib, "opencv_video470.lib")
#pragma comment(lib, "opencv_videoio470.lib")
#pragma comment(lib, "opencv_imgcodecs470.lib")
#pragma comment(lib, "opencv_imgproc470.lib")
#endif

/*

#include <iostream>
#include <thread>        
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "utils.hpp"
#include "../BauotechAIConnectorDll/files.hpp"
#include "../BauotechAIConnectorDll/AlgoApi.h"
#include "../BauotechAIConnectorDll/config.hpp"
#include "../BauotechAIConnectorDll/timer.hpp"
*/






#include <opencv2/opencv.hpp>
#include <vector>
//#include <filesystem>
#include <iostream>
#include <fstream>

    // Global variables
    std::vector<cv::Point> vertices;
    cv::Mat image;

    // Mouse callback function
    void onMouse(int event, int x, int y, int flags, void* userdata) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            // Left mouse button click - add point to the vector
            vertices.push_back(cv::Point(x, y));

            // Draw a circle at the clicked point
            cv::circle(image, cv::Point(x, y), 8, cv::Scalar(255, 0, 0), -1);

            // Display the updated image
            cv::imshow("Image", image);
        }
        else if (event == cv::EVENT_RBUTTONDOWN) {
            // Right mouse button click - close the polygon and fill it
            if (vertices.size() >= 3) {
                // Convert the vector of points to a vector of vectors (required by fillPoly)
                std::vector<std::vector<cv::Point>> polygons = { vertices };

                // Fill the polygon on the image
                //cv::fillPoly(image, polygons, cv::Scalar(0, 255, 0)); // Green color
                cv::Scalar color = cv::Scalar(0, 255, 0);
                cv::drawContours(image, polygons, -1, color, 4, 8);


                // Display the image with the filled polygon
                cv::imshow("Image with Polygon", image);

                // Clear the vector for the next polygon
                //vertices.clear();
            }
        }
    }

    /*-----------------------------------------------------------------------
    [
	    {
		    "camID": 0,
		    "detection-type": "car",
		    "max-allowed": 0,
		    "polygon": [ 460, 500, 460, 1000, 1160, 1000, 1160, 3500 ],
    	}
    ]
    -----------------------------------------------------------------------*/
    bool writeCameraConfig(std::string fname, std::vector<cv::Point> vertices)
    {
        std::ofstream camfile(fname);
        //ifstream simple("C:\\src\\ConsoleApplication1\\test.json");

        if (vertices.size() < 3)
            return false;

        if (!camfile.is_open())
            return false;

        camfile << "[\n";
        camfile << "\t{\n";
        camfile << "\t\t\"camID\": 0,\n";
        camfile << "\t\t\"detection-type\": \"car\",\n";
        camfile << "\t\t\"max-allowed\" : 0,\n";
        camfile << "\t\t\"polygon\" : ["; 
        int i = 0;
        for (; i < vertices.size() - 1; i++)
            camfile << vertices[i].x << "," << vertices[i].y << ",";
        // last point:
        camfile << vertices[i].x << ", " << vertices[i].y << "]\n";
        camfile << "\t}\n";
        camfile << "]\n";


        camfile.close();
        return true;    
    }
    /*------------------------------------------------------------------------------------------------------
    * Write a new CAMERA.JSON file:
    * input params:
    * (1) image to load
    * (2) output camera.json file  name
     ------------------------------------------------------------------------------------------------------*/
    int main_camFileGen(int argc, char* argv[])
    {
        // Load the image
        std::string imageNname; //  = R"(C:\Users\scsho\Pictures\vlcsnap-2024-01-28-17h09m55s442.png)";
        std::string cameraNname = R"(C:\tmp\camera_draft.json)";

        if (argc < 2) {
            std::cout << "ERROR: Missing input image : \n";
            std::cout << "usage : cameraCreate.exe <image full path> [<outputfilename> = c:\\tmp\\camera_draft.json]  \n";
            exit(0);
        }

        imageNname = argv[1];
        if (argc > 2)
            cameraNname = argv[2];


        //
        if (1) {
            cv::VideoCapture cap;

            if (!cap.open(imageNname)) {
                std::cout << "Can't open file  " << imageNname << ")\n";
                return -1;
            }

            cap >> image;
        }
        else
            image = cv::imread(imageNname);

        // Check if the image was loaded successfully
        if (image.empty()) {
            std::cerr << "Error: Could not load the image." << std::endl;
            return -1;
        }

        // Create a window to display the image
        cv::namedWindow("Image", cv::WINDOW_NORMAL);

        // Set the mouse callback function
        cv::setMouseCallback("Image", onMouse, nullptr);

        // Display the image
        cv::imshow("Image", image);

        // Wait for a key press and close the window
        cv::waitKey(0);

        writeCameraConfig(cameraNname, vertices);
        cv::destroyAllWindows();

        return 0;
    }