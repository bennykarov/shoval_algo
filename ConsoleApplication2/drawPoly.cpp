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



#include <opencv2/opencv.hpp>
#include <vector>
//#include <filesystem>
#include <iostream>
#include <fstream>
#include <conio.h>

    // Global variables
    std::vector<cv::Point> vertices;
    cv::Mat image;

    std::string WINDOW_NAME = "draw-polygon";


    std::string change_extension(const std::string& filename, const std::string& extension)
    {
        char path[_MAX_PATH];
        char drive[_MAX_DRIVE];
        char dir[_MAX_DIR];
        char fname[_MAX_FNAME];
        char ext[_MAX_EXT];

        _splitpath_s(filename.c_str(), drive, dir, fname, ext);
        _makepath_s(path, drive, dir, fname, extension.c_str());

        return path;
    }


    // Mouse callback function
    void onMouse(int event, int x, int y, int flags, void* userdata) {
        if (event == cv::EVENT_LBUTTONDOWN) {
            // Left mouse button click - add point to the vector
            vertices.push_back(cv::Point(x, y));

            // Draw a circle at the clicked point
            cv::circle(image, cv::Point(x, y), 8, cv::Scalar(255, 0, 0), -1);

            // Display the updated image
            cv::imshow(WINDOW_NAME, image);
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
            "motionType":1
		    "polygon": [ 460, 500, 460, 1000, 1160, 1000, 1160, 3500 ],
    	}
    ]
    -----------------------------------------------------------------------*/
    bool writeCameraConfig(std::string fname, std::vector<cv::Point> vertices)
    {
        std::ofstream camfile(fname);

        if (vertices.size() < 3)
            return false;

        if (!camfile.is_open())
            return false;

        camfile << "[\n";
        camfile << "\t{\n";
        camfile << "\t\t\"camID\": 0,\n";
        camfile << "\t\t\"detection-type\": \"car\",\n";
        camfile << "\t\t\"max-allowed\" : 0,\n";
        camfile << "\t\t\"motionType\" : 1,\n";        
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
    int main_drawPoly(int argc, char* argv[])
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

            cameraNname = change_extension(imageNname, "json");

            cap >> image;

            if (1) // many records has empty frames at start
                for (int i=0;i<100;i++)
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
        cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);

        // Set the mouse callback function
        cv::setMouseCallback(WINDOW_NAME, onMouse, nullptr);

        // Display the image
        cv::imshow(WINDOW_NAME, image);

        // Wait for a key press and close the window
        cv::waitKey(0);

        if (writeCameraConfig(cameraNname, vertices))
            std::cout << " Json file successfully save to " << cameraNname << "\n";
        else
            std::cout << " Error in saving Json file " << cameraNname << "\n";
        _kbhit();

        cv::destroyAllWindows();

        return 0;
    }