#include <thread> 
#include <functional>  // std::ref
#include <future>    // std::promise
#include <condition_variable> 
#include <iostream> 
#include <mutex> 
#include <queue> 
#include <stdlib.h>     /* srand, rand */
#include <numeric>
#include <format>

#include "opencv2/opencv.hpp"

#include <boost/circular_buffer.hpp>

#include "AlgoApi.h" // for MAX_VIDEO const
#include "logger.hpp"
#include "loadBalancer.hpp"
#include "testBalancer.hpp"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include <cuda_runtime.h>

#include "utils.hpp"

boost::random::mt19937 gen;


std::mutex print_mutex;


const int MaxCameras = 20; // DDEBUG DDEBUG TO DELETE

#define USE_LOAD_BALANCER
#ifdef USE_LOAD_BALANCER


const int CYCLES_NUM = 5500;
const int processDuration = 0;
int g_run6Counter = 0;

#ifdef _DEBUG
const int beatTick = 99;
#else
const int beatTick = 33;
#endif 



//-=======================================================================
//-=======================================================================
// PRINT STATISTICS 
//-=======================================================================
//-=======================================================================

//StatisticsInfo  printStatistics(boost::circular_buffer<CCycle> cyclesInfo, std::vector <int> cameraList, int batchSize, int currMaxPriority)
StatisticsInfo  printStatistics(std::vector <CCycle> cyclesInfo, std::vector <int> activeCameras, int batchSize, int currMaxPriority)
{
	StatisticsInfo statisticsRes;

	std::string statisticsFName = "";

	static CTimer timer;
	static bool FirstTime = true;

	if (FirstTime) {
		timer.start();
		FirstTime = false;
	}

	auto LOG_SAVIRITY = DLEVEL::ERROR2;

	std::ofstream queueFile; // (fname);
	std::stringstream msg;

	LOGGER::log(LOG_SAVIRITY, "================  printStatistics   ==========================");
	LOGGER::log(LOG_SAVIRITY, "");

	//msg   << "total cycles = " << cyclesInfo.size();
	int batchCycles = cyclesInfo.size() / batchSize;
	float elapsed = timer.sample();
	float batchFPS = (1000. * batchCycles) / elapsed;


	LOGGER::log(LOG_SAVIRITY, std::string("Batches cycles = " + std::to_string(batchCycles) + "; FPS = " + std::to_string(batchFPS)));
	LOGGER::log(DLEVEL::ERROR2, std::string("( curr Max priority = " + std::to_string(currMaxPriority) + "\n"));


	// Summarize info for each cam
	for (int cam : activeCameras) {
		int camID = cam;
		auto lambda = [camID](CCycle c) { return c.camID == camID; };
		int processNum = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), lambda);
		int activeFrames = std::count_if(cyclesInfo.begin(), cyclesInfo.end(), [cam](CCycle c) { return c.camID == cam && c.activeCamera > 0; });

		// OLD FASION COUNT 
		int sumDetectoins = 0;
		for (auto cycleInf : cyclesInfo) {
			if (cycleInf.camID == cam)
				sumDetectoins += cycleInf.detections > 0 ? 1 : 0;
		}

		//--------------------------
		// Calc avg ellapsed time
		//--------------------------
		int prevFrameNum = -1;
		std::vector <int> framesNum;
		for (int i = 0; i < cyclesInfo.size(); i++) {
			if (cyclesInfo[i].camID == cam)
				framesNum.push_back(cyclesInfo[i].timeStamp);
		}

		float camFPS = (1000. * processNum) / elapsed;

		bool detailed = true;

		if (detailed) {
			int ellapsedTime = 0;
			int max_ellapsedTime = 0;
			int min_ellapsedTime = 99999;
			for (int i = 1; i < framesNum.size(); i++) {
				int ellapsed = framesNum[i] - framesNum[i - 1];
				if (ellapsed == 1)
					int debug = 10;
				ellapsedTime += ellapsed;
				max_ellapsedTime = max(max_ellapsedTime, ellapsed);
				min_ellapsedTime = min(min_ellapsedTime, ellapsed);
			}
			msg << "cam= " << cam << " ; active = " << activeFrames << " ; detections = " << sumDetectoins << " ; cycles = " << processNum << std::setprecision(2) << "; FPS = " << camFPS << std::setprecision(2) << " (min,max = " << min_ellapsedTime << "," << max_ellapsedTime << ")";
		}
		else {
			msg << "cam= " << cam << " ; active = " << activeFrames << " ; detections = " << sumDetectoins << " ; cycles = " << processNum << std::setprecision(2) << "; FPS = " << camFPS << std::setprecision(2);
		}
		LOGGER::log(LOG_SAVIRITY, msg.str());
		LOGGER::log(LOG_SAVIRITY, "--------------------------------------------------------------");
		msg.str("");
		msg.clear();

	}


	statisticsRes.camID = -1; // DDEBUG RETURN FAKED EMPTY DATA  
	return statisticsRes;
} 

#endif  

/*--------------------------------------------------------------------------------------------------------------------------------------

/*--------------------  D A S H B O A R D C L A S S   ------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------------------------------------
* Dashboard information includes :
* (1) Number of total cameras 
* (2) BatchSize
* (3) Number of cameras CURRENTLY in process
* (4) FPS of each camera (sort by...)
* (5) FPS of a batch cycle
* (6) GPU memory in used
* (7) 
 --------------------------------------------------------------------------------------------------------------------------------------*/


/*
size_t GetGraphicDeviceVRamUsage(int _NumGPU)
{
	//cudaSetDevice(_NumGPU);

	size_t l_free = 0;
	size_t l_Total = 0;
	cudaError_t error_id = cudaMemGetInfo(&l_free, &l_Total);

	return (l_Total - l_free);
}
*/

void CDashboard::init(int batchSize, int expectedCams)
{
	cv::Size size(640, 480);
	dashboardImg = cv::Mat(size, CV_8UC3);
	dashboardImg.setTo(90);

	GPU_UTIL::checkForGPUs();
	m_active = true;

}
void CDashboard::update(std::vector <int> camList, StatisticsInfo statistics, int currMaxPrioty) 
{
	static int debug = 0;
	if (!m_active)
		return;

	size_t l_free;
	size_t l_Total;
	GPU_UTIL::cudaMemGetInfo_(l_free, l_Total);

	m_gpuMemTotal= (int)(l_Total / (1024 * 1024));
	m_gpuMemUsed = int((l_Total - l_free) / (1024 * 1024));


	m_camList = camList;
	m_statistics = statistics;
	m_currMaxPrioty = currMaxPrioty;

	

}
void CDashboard::show() {
	if (!m_active)
		return;

	dashboardImg.setTo(90);
	int x = 10;
	int y = cell_h*4;
	drawInfo(cv::Point(x, y), "GPU Used", 16, m_gpuMemUsed, cv::Point2f(0, 300));
	y += cell_h * 2;
	drawInfo(cv::Point(x, y), "GPU Usage %", 16, (float)m_gpuMemUsed*100 / (float)m_gpuMemTotal, cv::Point2f(0, 300));
	y += cell_h * 2;
	drawInfo(cv::Point(x, y), "test", 160, 2000.3, cv::Point2f(0, 3000));

	cv::imshow("Dashboard", dashboardImg);
	cv::waitKey(1);
}


void CDashboard::drawInfo(cv::Point tl, std::string title, float num, float value, cv::Point2f  measureRange) 
{
	if (!m_active)
		return;

	int xPointer = 0;
	cv::Rect titleBox(xPointer, 0, cell_w, cell_h);
	xPointer += cell_w;
	cv::Rect valueBox(xPointer, 0 , cell_w, cell_h);
	xPointer += cell_w;
	cv::Rect barBox(xPointer, 0 , cell_w, cell_h);

	titleBox += tl;
	valueBox += tl;
	barBox   += tl;
	// title
	cv::rectangle(dashboardImg, titleBox, CV_RGB(0, 155, 100), -1);
	cv::rectangle(dashboardImg, titleBox, CV_RGB(100, 100, 0), 2);
	cv::putText(dashboardImg, title, cv::Point(titleBox.x + 10, titleBox.y + int(cell_h / 2) + 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(0, 0, 20), 2);

	// value
	cv::rectangle(dashboardImg, valueBox, CV_RGB(0, 255, 0), -1);
	cv::rectangle(dashboardImg, valueBox, CV_RGB(100, 100, 0), 2);
	if (value == int(value))
		cv::putText(dashboardImg, std::to_string(int(value)), cv::Point(valueBox.x + 10, valueBox.y + int(cell_h / 2) + 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(0, 0, 20), 2);
	else {
		std::ostringstream oss;
		oss << std::setprecision(2) << value;
		cv::putText(dashboardImg, oss.str(), cv::Point(valueBox.x + 10, valueBox.y + int(cell_h / 2) + 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(0, 0, 20), 2);
		//cv::putText(dashboardImg, std::format("The answer is {}.", value), cv::Point(valueBox.x + 10, valueBox.y + int(cell_h / 2) + 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(0, 0, 20), 2);
	}
}

