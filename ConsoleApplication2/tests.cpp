#include <iostream>
#include <vector>
#include <map>
#include <thread>
#include <shared_mutex>  // C++17

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "utils.hpp"



using namespace UTILS_CONSOLE2;


cv::Mat drawEdges(cv::Mat img)
{
	cv::Mat gray, edgeFrame;

	if (img.channels() > 1)
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	else
		gray = img;

	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::Canny(gray, edgeFrame, 100, 200);
	return edgeFrame;

}

void TEST_drawEdges(std::string videoName)
{
	cv::VideoCapture cap;
	cv::Mat frame, edgeFrame, edgeFrameColor;

	if (!cap.open(videoName)) {
		std::cerr << "ERROR ERROR : Can't open file  " << videoName << ")\n";
		return;
	}

	int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);


	std::string edgeVideoName = videoName;
	auto dotLoc = edgeVideoName.find_last_of(".");
	edgeVideoName[dotLoc] = '0';

	edgeVideoName.append("_edges.avi");


	//std::string edgeVideoName = change_extension(videoName, "avi");

	cv::VideoWriter video;
	
	//video.open(edgeVideoName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, cv::Size(width, height));
	video.open(edgeVideoName, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 25, cv::Size(width, height));

	if (!video.isOpened()) {
		std::cerr << "Could not open the output video file for write : " << edgeVideoName << "\n";
		return;
	}

	cap >> frame;
	int frameNum = 0;
	int key = 0;
	while (!frame.empty() && key != 27)
	{
		std::vector<cv::Point> poly = { cv::Point(100, 100), cv::Point(200, 100), cv::Point(200, 200), cv::Point(100, 200) };
		edgeFrame = drawEdges(frame);

		//cv::putText(edgeFrame, std::to_string(frameNum), cv::Point(30, height-20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 1);


		cv::imshow("Frame", edgeFrame);
		if (frameNum++ == 0)
			cv::waitKey(-1);
		// write video frame with edges	
		// Default resolutions of the frame are obtained.The default resolutions are system dependent.

		// Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file.
		if (1)
		{
			cv::cvtColor(edgeFrame, edgeFrameColor, cv::COLOR_GRAY2BGR);
			video.write(edgeFrameColor);
		}
		else
			video.write(edgeFrame);


		key = cv::waitKey(5);

		if (key == 'p')
			key = cv::waitKey(-1);

		cap >> frame;
	}

	cap.release();
	video.release();


}


void pixelization(cv::Mat &img)
{
	cv::Mat tmpImg;
	float scale = 0.1;
	cv::resize(img, tmpImg, cv::Size(5, 5), 0, 0, cv::INTER_LINEAR);
	cv::resize(tmpImg, img, cv::Size(img.cols, img.rows), cv::INTER_NEAREST);

}

/*----------------------------------------------------------------------------------------------------
* Find project polygon (template) on video frames 
-----------------------------------------------------------------------------------------------------*/
std::vector<cv::Point > test_correrlation(std::string videoFName, std::string templateFname /*, std::vector<cv::Point > templatePolygon*/)
{
	// Read video file
	cv::VideoCapture cap(videoFName);
	if (!cap.isOpened()) {
		std::cout << "Error opening video stream or file" << std::endl;
		return std::vector<cv::Point >();
	}

	
	// read template file
	cv::Mat templ = cv::imread(templateFname);
	if (templ.empty()) {
		std::cout << "Error opening template file" << std::endl;
		return std::vector<cv::Point >();
	}


	std::vector <cv::Mat> templPieces;
	std::vector <double> maxVals(3);
	std::vector <double> minVals(3);
	std::vector <cv::Point> minLocs(3);
	std::vector <cv::Point> maxLocs(3);
	
	//double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;

	int peiceWidth = templ.size().width / 3;
	int peiceHeight = templ.size().height;
	for (int i=0;i<3;i++)
		templPieces.push_back(templ(cv::Rect(peiceWidth*i, 0, peiceWidth, peiceHeight)));



	cv::Mat frame;
	std::vector<cv::Point> poly;

	int frameNum = 0;

	int skipStep = 2;
	cap >> frame;
	while (!frame.empty())
	{

		if (frame.empty()) {
			std::cout << "Error opening video stream or file" << std::endl;
			return std::vector<cv::Point >();
		}

		// Match the 3 templates
		cv::Mat result;
		//cv::matchTemplate(frame, templ, result, cv::TM_CCOEFF_NORMED); // TM_SQDIFF
		for (int i = 0; i < 3; i++) {
			cv::matchTemplate(frame, templPieces[i], result, cv::TM_CCOEFF_NORMED); // TM_SQDIFF
			//double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
			cv::minMaxLoc(result, &minVals[i], &maxVals[i], &minLocs[i], &maxLocs[i], cv::Mat());
		}

		int bestMatchInd;

		if (maxVals[1] > 0.7)
			bestMatchInd = 1;
		else {
			// Find best matching from 3 pieces:
			auto it = max_element(maxVals.begin(), maxVals.end());
			bestMatchInd = distance(maxVals.begin(), it);
		}

		// Calc middle ROI:
		cv::Scalar color; 

		cv::Rect roi;
		if (bestMatchInd == 1) 
			roi = cv::Rect(maxLocs[bestMatchInd], cv::Size(templPieces[bestMatchInd].cols, templPieces[bestMatchInd].rows));
		else if (bestMatchInd == 0) // left side
			roi = cv::Rect(maxLocs[bestMatchInd] + cv::Point(templPieces[0].cols, 0), cv::Size(templPieces[1].cols , templPieces[1].rows));
		else // right
			roi = cv::Rect(maxLocs[bestMatchInd] - cv::Point(templPieces[0].cols, 0), cv::Size(templPieces[1].cols, templPieces[1].rows));

		// trim ROI to frame size
		roi = roi & cv::Rect(0, 0, frame.cols, frame.rows);

		// balck for low score, green for high score
		bool found = maxVals[bestMatchInd] > 0.7;
		color = found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 0);

		// Draw rectangle around the object
		//cv::Rect roi(maxLoc, cv::Size(templ.cols, templ.rows));
		//cv::rectangle(frame, roi, cv::Scalar(0, 255, 0), 2);
		
		if (found && roi.width > 0 && roi.height > 0) {

			bool pixelate = true;
			bool blurImg = false;
			if (pixelate) {
				cv::Mat roiImg = frame(roi);
				pixelization(roiImg);

				if (0) {
					cv::Rect roiLarge = cv::Rect(roi.x - 10, roi.y - 10, roi.width + 20, roi.height + 20);
					cv::rectangle(frame, roiLarge, color, 2);
				}
			}
			else if (blurImg) {
				cv::blur(frame(roi), frame(roi), cv::Size(5, 5));
				cv::blur(frame(roi), frame(roi), cv::Size(3, 3));
				cv::blur(frame(roi), frame(roi), cv::Size(5, 5));
				cv::blur(frame(roi), frame(roi), cv::Size(7, 7));
				cv::blur(frame(roi), frame(roi), cv::Size(9, 9));
				cv::blur(frame(roi), frame(roi), cv::Size(5, 5));
			}
			else
				cv::rectangle(frame, roi, color, 2);
		}

		cv::putText(frame, std::to_string(int(maxVals[bestMatchInd] * 100.)) + "%", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
		cv::putText(frame, "best match=" + std::to_string(bestMatchInd) , cv::Point(100, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

		cv::imshow("Frame", frame);
		if (frameNum++ == 0)
			cv::waitKey(0);
		else
			cv::waitKey(1);


		/*
		std::vector<cv::Point > polygon;
		for (auto& point : templatePolygon) {
			polygon.push_back(point + maxLocs[bestMatchInd]);
		}
		*/

		poly = UTILS_CONSOLE2::roiToPolygon(roi);

		for (int f=0; f<skipStep; f++)
			cap >> frame;

	}



	return poly;
}

bool test_matchingFalse(cv::Mat img,  cv::Mat templ, float thresh)
{
	cv::Mat result;
	cv::matchTemplate(img, templ, result, cv::TM_CCOEFF_NORMED); // TM_SQDIFF
	double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
	cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	return maxVal > thresh;
}

bool run_test(std::string fname)
{
	if (0)
	{
		cv::Mat srcImg = cv::imread("C:\\tmp\\falsePerson1.png");
		cv::Mat templ = cv::imread("C:\\tmp\\falseTemplate.png");
		float thresh = 0.7;
		test_matchingFalse(srcImg, templ, thresh);
		return true;
	}

	if (0)
	{
		std::vector<cv::Point > ptrojectedPolygon = test_correrlation("C:\\Data\\ptz\\3\\20241009170658.ts", "C:\\Data\\ptz\\3\\ptz_template3.png"); // wrong arguments ???
		return true;
	}


	if (0)
	{
		//std::string videoName = "C:\\Data\\G1\\HowManyWorkersDetected.ts";// "C:\\Data\\ptz\\3\\20241009170658.ts";
		//std::string videoName = "D:\\data\\swimming_pool\\bt_8909_2024_11_20_16_16_13.ts";
		std::string videoName = "C:\\Data\\false_Nov_24\\peyshpesh3 - Copy.ts";
		videoName =  "C:\\Data\\SBD\\שער צהוב\\bt_2183_2024_10_21_11_26_07.ts";
		videoName = "C:\\Data\\dogs\\cam_1104_intelbras7_2024_9_23_16_6_5.ts";
		videoName = "C:\\Data\\SBD4\\SBD_Gdser5A_FalseHuman.ts";
		TEST_drawEdges(videoName);
		return true;
	}



	return false;
}
