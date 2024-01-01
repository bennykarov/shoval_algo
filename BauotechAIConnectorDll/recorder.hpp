#pragma once
#include <direct.h>


class CRecorder
{
public:
	~CRecorder()
	{
		if (m_videoSave.isOpened())
			m_videoSave.release();
	}
	bool init(std::string outputFolder, std::string fname,  cv::Size img_size, bool isColor, int fps, bool lossless=false)
	{
		int codec;
		/*
		if (lossless)
			codec = cv::VideoWriter::fourcc('L', 'A', 'G', 'S'); // select desired codec (must be available at runtime)
		else
		*/
		codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // select desired codec (must be available at runtime)
		//int codec = cv::VideoWriter::fourcc('p', 'n', 'g', ' '); // select desired codec (must be available at runtime)
		//int codec = cv::VideoWriter::fourcc('H', 'F', 'Y', 'U'); // select desired codec (must be available at runtime)

		std::stringstream fnameStream;
		time_t now = time(0);
		tm *ltm = localtime(&now);
		fnameStream << outputFolder << "/" << fname << "_" << ltm->tm_mday << "-" << ltm->tm_mon << "_" << ltm->tm_hour << "_" << ltm->tm_min << ".avi"; // name of the output video file

		m_videoSave.open(fnameStream.str(), codec, fps, img_size, isColor);

		// check if we succeeded
		if (!m_videoSave.isOpened())
			return false;

		return true;
	}

	void write(cv::Mat img) { m_videoSave.write(img); }


private:
	cv::VideoWriter m_videoSave;
};

class CRecorder2 // 32 float stream of images 
{
public:
	bool init(std::string outputFolder)
	{
		time_t now = time(0);
		tm *ltm = localtime(&now);
		m_folderNameStream << outputFolder << "/"
						   << "thermal"
						   << "_" << ltm->tm_mday << "-" << ltm->tm_mon << "_" << ltm->tm_hour << "_" << ltm->tm_min; // name of the output video file

		int ret = _mkdir(m_folderNameStream.str().c_str());
		return (ret == 0);
	}

	void write(cv::Mat img) { 
		m_fname = m_folderNameStream.str() +  "/frame_" + std::to_string(m_frameNum++) + ".tiff"; // ".PFM";
		cv::imwrite(m_fname, img); 
		}

	std::string getLastFName() // DDEBUG
	{
		return m_fname;
	}

private:
	std::stringstream m_folderNameStream;
	std::string m_fname; // stores last file name

	int m_frameNum=1;
};

