#ifndef UTILS_TESTS_HPP
#define UTILS_TESTS_HPP
#pragma once

bool run_test(std::string fname);

std::vector<cv::Point > test_correrlation(std::string videoFName, std::string templateFname, std::vector<cv::Point> templatePolygon = std::vector<cv::Point>());
bool test_matchingFalse(cv::Mat img, cv::Mat templ, float thresh);


#endif 