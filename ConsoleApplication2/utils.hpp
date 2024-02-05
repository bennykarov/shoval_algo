#ifndef UTILS_CONSOLE2_HPP
#define UTILS_CONSOLE2_HPP
#pragma once

#include <windows.h>
#include <vector>

#define CHECK_exception(COND, MSG)   ((COND) ? (static_cast<void>(0)) : assertion_fail(# COND, __FILE__, __LINE__, MSG))
#define CHECK_error(COND, MSG)   ((COND) ? (static_cast<void>(0)) : assertion_error(# COND, __FILE__, __LINE__, MSG))

/*
inline  void assertion_fail(const char* expr, const char* file, int line)
{
	std::string error = ";\n\nError    in expr : " + std::string(expr) + ", file: " + file + "(" + std::to_string(line) + ")";
	std::cout << error << "\n";
	MessageBoxA(0, error.c_str(), "EXCEPTION!", MB_OK);
	std::exit(1);
}
*/

inline  void assertion_fail(const char* expr, const char* file, int line, const char* msg)
{
	std::string error = msg;
	error.append(";    in expr" + std::string(expr) + ", file: " + file + "(" + std::to_string(line) + ")");
	MessageBoxA(0, error.c_str(), "EXCEPTION!", MB_OK);
	std::exit(1);
}


inline  void assertion_error(const char* expr, const char* file, int line, const char* msg)
{
	std::string error = msg;
	error.append(";    in expr" + std::string(expr) + ", file: " + file + "(" + std::to_string(line) + ")");
	std::cout << error << "\n";
	MessageBoxA(0, error.c_str(), "EXCEPTION!", MB_OK);
}




class CAlert_ { // duplicate structure of CAlert (alert.hpp)

public:
    int m_camID;
    std::vector<cv::Point > m_polyPoints;
    //std::tuple <int,int> label_allowed;
    int m_label = -1;
    int m_maxAllowed = -1;
    cv::Rect  m_bbox;
};


int readCamerasJson(std::string fname, int camID, std::vector <CAlert_>& cameras);
void drawPolygon(cv::Mat& img, std::vector< cv::Point> contour, float scale);

std::string toUpper(std::string str);


#endif 