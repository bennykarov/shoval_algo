#pragma once 
#include <Windows.h>
#include <string>
#include <vector>
#include <fstream> 
#include <algorithm>

#include <boost/lexical_cast.hpp> 


// EXCEPTION HANDLING
#define CHECK_warning(COND, MSG)   ((COND) ? (static_cast<void>(0)) : warning_fail(# COND, __FILE__, __LINE__, MSG))
#define CHECK_assertion(COND)   ((COND) ? (static_cast<void>(0)) : assertion_fail(# COND, __FILE__, __LINE__))
#define CHECK_exception(COND, MSG)   ((COND) ? (static_cast<void>(0)) : assertion_fail(# COND, __FILE__, __LINE__, MSG))


template<typename T>
//std::vector<T> to_array(const std::string& s)
std::vector<T> to_array(std::string s)
{
	std::vector<T> result;
	std::string item;

	//remove_if(s.begin(), s.end(), isspace);
	std::string::iterator end_pos = std::remove(s.begin(), s.end(), ' ');
	s.erase(end_pos, s.end());
	std::stringstream ss(s);
	while (std::getline(ss, item, ','))
		result.push_back(boost::lexical_cast<T>(item));
	return result;
}

inline  void assertion_fail(const char* expr, const char* file, int line, const char* msg)
{
	std::string error = msg;
	error.append(";    in expr" + std::string(expr) + ", file: " + file + "(" + std::to_string(line) + ")");
	MessageBoxA(0, error.c_str(), "EXCEPTION!", MB_OK);
	std::exit(1);
}
inline  void assertion_fail(const char* expr, const char* file, int line)
{
	std::string error = ";Error    in expr : " + std::string(expr) + ", file: " + file + "(" + std::to_string(line) + ")";
	MessageBoxA(0, error.c_str(), "EXCEPTION!", MB_OK);
	std::exit(1);
}
inline  void warning_fail(const char* expr, const char* file, int line, const char* msg)
{
	std::string error = msg;
	error.append(";    in expr" + std::string(expr) + ", file: " + file + "(" + std::to_string(line) + ")");
	MessageBoxA(0, error.c_str(), "EXCEPTION!", MB_OK);
	//std::exit(1);
}

template<typename T>
T sqr(T x)
{
	return x * x;
}

template<typename T>
inline double distanceSQR(T a, const T b)
{
	return sqr(a.x - b.x) + sqr(a.y - b.y);
}


// Function for calculating median
template<typename T>
inline T findMedian(std::vector <T> a)
{
	int n = a.size();
	// First we sort the array
	std::sort(a.begin(), a.end());

	// check for even case
	if (n % 2 != 0)
		return (double)a[n / 2];

	return (double)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
}


template<typename T>
inline double distance(T a, const T b)
{
	return sqrt(sqr(a.x - b.x) + sqr(a.y - b.y));
}


enum ANGLES {
	PAN_IDX=0, // Heading 
	TILT_IDX,  // Attiude
	ROLL_IDX   // Bank
};

#define   DEG2RAD(angle)		(angle * 0.0174532925)
#define   RAD2DEG(angle)        (angle / 0.0174532925)
inline cv::Point3f  RAD2DEG_vec3f(cv::Point3f radVec) { return cv::Point3f(RAD2DEG(radVec.x), RAD2DEG(radVec.y), RAD2DEG(radVec.z)); }
inline std::vector <float> RAD2DEG_vec(std::vector <float>  radVec) { std::vector <float>  degVec;  for (auto &rad : radVec) degVec.push_back(RAD2DEG(rad)); return degVec; }


cv::Rect2f  moveByCenter(cv::Rect2f r, cv::Point2f center);
inline cv::Point2f centerOf(cv::Rect2f r) { return (r.br() + r.tl()) * 0.5; }
inline cv::Point centerOf(cv::Rect r) { return cv::Point((r.br() + r.tl()) * 0.5); }
inline int areaOf(cv::Rect2f r) { return int(r.width + r.height); }

inline bool isEmpty(cv::RotatedRect rr) { return (rr.size.width == 0 || rr.size.height == 0); }
inline bool isEmpty(cv::Rect2f r) { return (r.width == 0 || r.height == 0); }
cv::Rect resize(cv::Rect r, cv::Size newDim); 
bool similarAreas(cv::Rect r1, cv::Rect r2, float absRatio);
bool similarBox(cv::Rect r1, cv::Rect r2, float absRatio);

template <typename T>
bool similarBox2(T r1, T r2, float absRatio)
{
	float ratio = (float)r1.width / (float)r2.width;
	if (ratio > 1.)
		ratio = 1. / ratio;
	if (ratio < absRatio)
		return false;
	ratio = (float)r1.height / (float)r2.height;
	if (ratio > 1.)
		ratio = 1. / ratio;
	if (ratio < absRatio)
		return false;

	return true;
}



float similarBox(cv::Rect r1, cv::Rect r2);
float relativeDist(cv::Rect box1, cv::Rect box2);

 using namespace std;


 class UTILS {
	 public:
	 vector<cv::Rect>  alignRois(vector<cv::Rect> rois);
	 static bool drawCorners(cv::Mat &img, const vector<cv::Rect> rois);
	 static void drawCorss(cv::Mat &img, cv::Rect r, cv::Scalar color, int thickness = 1)
	 {
		 cv::line(img, r.tl(), r.br(), color, thickness);
		 cv::line(img, cv::Point(r.br().x, r.tl().y), cv::Point(r.tl().x, r.br().y), color, thickness);
	 };

	 template <typename T>
	 static float ratio(T n1, T n2)
	 {
		 if (n2 > n1)
			 return (float)n1 / (float)n2;
		 else
			 return (float)n2 / (float)n1;
	 }

#if 0
	 template <typename T>
	 static T scaleBBox(T rect, float scale)
	 {
		 T sBBox;

		 sBBox.width = int((float)rect.width * scale);
		 sBBox.height = int((float)rect.height * scale);
		 sBBox.x = int((float)rect.x * scale);
		 sBBox.y = int((float)rect.y * scale);

		 return sBBox;
	 }
#endif 
	 template <typename T> 
	 static bool   checkBounderies(T& box, cv::Size imgSize)
	 {
		 T origBbox = box;

		 if (box.x < 0)
			 box.x = 0;
		 if (box.y < 0)
			 box.y = 0;
		 if (box.x + box.width >= imgSize.width)
			 box.width = imgSize.width - box.x - 1;
		 if (box.y + box.height >= imgSize.height)
			 box.height = imgSize.height - box.y - 1;

		 if (box.x != origBbox.x || box.y != origBbox.y || box.width != origBbox.width || box.height != origBbox.height)
			 int debug = 10;


		 return  (box.x != origBbox.x || box.y != origBbox.y || box.width != origBbox.width || box.height != origBbox.height);
	 }
 
	 static bool nearEdges(cv::Size size, cv::Rect box);

	 static cv::Rect2f scaleBBox(cv::Rect2f rect, float scale);
	 static cv::Rect   scaleBBox(cv::Rect rect, float scale);
	 static cv::Rect   mergeBBoxSize(cv::Rect rect1, cv::Rect rect2, float AlphaBland);
	 static cv::Rect   mergeBBoxes(cv::Rect rect1, cv::Rect rect2, float AlphaBland);

	 //static cv::Rect   mergeBBoxes2(cv::Rect rect1, cv::Rect rect2, float AlphaBland);


	 static bool isFileExist(std::string fname) 
	 {
		 std::string fileName(fname);
		 ifstream fin(fileName.c_str());
		 return (!fin.fail());
		 //return (0xffffffff = !GetFileAttributes(LPCSTR(fname.c_str()));
	 }

 };


 // Find value in mat
 template <class T>
 bool findValue(const cv::Mat &mat, T value, int &i, int &j) {

	 for (int r = 0; r < mat.rows; r++) {
		 const T* row = mat.ptr<T>(r);
		 const T *pos = std::find(row, row + mat.cols, value);
		 if (pos != row + mat.cols) {
			 i = pos - row;
			 j = r;
			 return true;
		 }
	 }
	 return false;
 }

 std::wstring stringToWstring(const std::string& t_str);
 int debug_imshow(std::string title, cv::Mat img, double scale = 0, int waitTime = -1);

// MATH UTILS 
 double interpolate(vector<double> &xData, vector<double> &yData, double x, bool extrapolate);
 std::vector<double> interpolation2(std::vector<double> x, std::vector<double> y, std::vector<double> xx);



#if 0
 //opencv
 cv::Mat rot2euler_CV(const cv::Mat & rotationMatrix);
 cv::Mat euler2rot_CV(cv::Vec3f euler3);

 cv::Vec3f rot2euler_1(const cv::Mat_<float> & rotationMatrix);
 cv::Mat   euler2rot_1(cv::Vec3f euler3);

 // learnOpencv
 cv::Vec3f rot2euler_3(cv::Mat &R);
 cv::Mat   euler2rot_3(cv::Vec3f &theta);
#endif 

 namespace GEO_UTILS {

	 //cv::Vec3f rot2euler(cv::Mat_<float> R);
	 cv::Vec3f rot2euler(cv::Mat R);
	 cv::Mat   euler2rot(cv::Vec3f theta);

	 cv::Mat shiftRotationMatrix(cv::Mat_<float>  rotationMatrix, cv::Vec3f shiftRot);


	 // Y version 
	 cv::Mat euler2rot_2(cv::Vec3f euler3);

	 // DDEBUG DDEBUG TEMP
	 /*
	 cv::Mat rot2euler_CV(cv::Mat_<float>  rotationMatrix);
	 cv::Mat euler2rot_CV(cv::Vec3f euler3);
	 cv::Vec3f rot2euler_1(cv::Mat_<float>  rotationMatrix);
	 cv::Mat   euler2rot_1(cv::Vec3f euler3);
	 // learnOpencv
	 cv::Vec3f rot2euler_3(cv::Mat_<float> R);
	 cv::Mat   euler2rot_3(cv::Vec3f theta);
	  */
 }


 // RECT utilities 
 bool isIn(cv::Point hitPixel, cv::Rect2f  roi);
 cv::Rect2f extendBBox(cv::Rect2f rect_, cv::Point2f p);
 
 
 cv::Rect centerBox(cv::Point center, cv::Size size);
 float    bboxRatio(cv::Rect2f r1, cv::Rect2f r2);
 float    bboxOrderRatio(cv::Rect2f r1, cv::Rect2f r2); // order is matter
 cv::Rect resizeBBox(cv::Rect rect, float scale);
 cv::Rect2f resizeBBox(cv::Rect2f rect, float scale);
 cv::Rect2f resizeBBox(cv::Rect2f rect, cv::Size size, float scale);
 void blendBbox(cv::Rect2f &r1, cv::Rect2f r2, float alpha = 0.3);

 
 float bboxesBounding(cv::Rect2f r1, cv::Rect2f r2); // Ratio of r1 overlapping r2
 float OverlappingRatio(cv::Rect2f r1, cv::Rect2f r2); // Ratio of overlapping (bi directional)

 // Max Ratio of r1 & r2 overlapping 
 template <typename T>
 float OverlappingRatio2(T r1, T r2)
 {
	 int overlappedArea = (r1 & r2).area();
	 if (overlappedArea == 0)
		 return 0;

	 return   r1.area() < r2.area() ? overlappedArea / r1.area() : overlappedArea / r2.area();
 }

 int depth2cvType(int depth);
 cv::Mat converPTR2MAT(void* data, int height, int width, int depth);

 namespace GPU_UTIL {

	 int checkForGPUs();
	 size_t GetGPURamUsage(int _NumGPU);
	 size_t GetGPURamtotal(int _NumGPU);
	 bool cudaMemGetInfo_(size_t& l_free, size_t& l_Total);

 }