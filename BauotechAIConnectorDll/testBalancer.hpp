#ifndef TEST_BALANCER_HEADER
#define TEST_BALANCER_HEADER

class StatisticsInfo {
public:
	int camID;
	int activeFrames;
	int sumDetectoins;
	int processNum; // cucles num
	float camFPS;
	float min_ellapsedTime;
	float max_ellapsedTime;

	void clear() {
		camID;
		activeFrames=0;
		sumDetectoins=0;
		processNum=0; // cucles num
		camFPS=0;
		min_ellapsedTime=0;
		max_ellapsedTime=0;

	}
};


StatisticsInfo printStatistics(boost::circular_buffer<CCycle> cyclesInfo, std::vector <int> camList, int batchSize, int currMaxPrioty);

class CDashboard {
public:
	void init(int batchSize, int expectedCams);
	void update(std::vector <int> camList, StatisticsInfo statistics, int currMaxPrioty);
	void show();
	
private:
	void drawInfo(cv::Point tl, std::string title, float num, float measure, cv::Point2f  measureRange);

private:
	// GPU info
	std::string m_GPU_name;
	int m_gpuMemTotal; // GB
	int m_gpuMemUsed;
	int RAMem;

	// Cams info
	std::vector <int> m_camList;
	StatisticsInfo m_statistics;
	int m_currMaxPrioty;
	cv::Mat dashboardImg;
	bool m_active = false;


	int cell_w = 150;
	int cell_h = 35;

};

#endif 