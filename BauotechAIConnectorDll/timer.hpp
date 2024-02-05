#pragma once 
#ifndef TIMER_HEDER
#define TIMER_HEDER
#include <Windows.h>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp> 


class CTimer {
public:
	void  start() {
		m_start_time = m_cur_time = std::chrono::system_clock::now();
	}
	double  sample__() { m_prev_time = m_cur_time;  m_cur_time = std::chrono::system_clock::now(); return (m_cur_time - m_prev_time).count(); }
	double  sample()
	{
		m_cur_time = std::chrono::system_clock::now();

		m_elapsed = m_cur_time - m_prev_time;
		m_prev_time = m_cur_time;
		return m_elapsed.count();
	}

	//double  durationFromStart() { return (m_cur_time - m_start_time).count() / 1000.; } 
	void printDuration(std::string msg)
	{
		double duration = sample(); std::cout << msg << "( duration = " << duration << " )";
	}


private:
	std::chrono::system_clock::time_point  m_start_time, m_prev_time, m_cur_time;
	std::chrono::duration<float, std::milli> m_elapsed;
};



class CTimer_ {
public:
	double  start() { m_start_time = m_cur_time = (double)GetTickCount(); return m_start_time / TICKFREQUENCY; }
	double  sample() { m_prev_time = m_cur_time;  m_cur_time = (double)GetTickCount(); return (m_cur_time - m_prev_time) / TICKFREQUENCY; }
	double  durationFromStart() { return (m_cur_time - m_start_time) / TICKFREQUENCY; } // in sec
	void printDuration(std::string msg)
	{
		double duration = sample(); std::cout << msg << "( duration = " << duration << " )";
	}


private:
	double m_start_time;
	double m_prev_time;
	double m_cur_time;
	float TICKFREQUENCY = 1000.;

};
#endif 