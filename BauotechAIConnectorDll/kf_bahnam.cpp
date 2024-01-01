//http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <chrono>
#include <iomanip>
 
//#include <Eigen/Dense>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
//#include "rapidcsv.h"

#include "Kalman2.hpp"


bool g_predictOnly = false;

namespace Eigen
{
    namespace internal
    {
        template<typename Scalar>
        struct scalar_normal_dist_op
        {
            static boost::mt19937 rng;    // The uniform pseudo-random algorithm
            mutable boost::normal_distribution<Scalar> norm;  // The gaussian combinator
 
            EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)
 
            template<typename Index>
            inline const Scalar operator() (Index, Index = 0) const { return norm(rng); }
        };
 
        template<typename Scalar> boost::mt19937 scalar_normal_dist_op<Scalar>::rng;
 
        template<typename Scalar>
        struct functor_traits<scalar_normal_dist_op<Scalar> >
        { enum
            { Cost = 50 * NumTraits<Scalar>::MulCost, PacketAccess = false, IsRepeatable = false };
        };
    }
}
 
template<typename Clock, typename Duration>
std::ostream &operator<<(std::ostream &stream,  const std::chrono::time_point<Clock, Duration> &time_point)
{
    const time_t time = Clock::to_time_t(time_point);
    struct tm tm;
    localtime_r(&time, &tm);
    return stream << std::put_time(&tm, "%c"); // Print standard date&time
 
}
 
auto start = std::chrono::high_resolution_clock::now();
int k=5;
 
Eigen::MatrixXd multivariateNormalDistribution()
{
    int size = 2; // Dimensionality (rows)
    int nn=1;     // How many samples (columns) to draw
    Eigen::internal::scalar_normal_dist_op<double> randN; // Gaussian functor
    //Eigen::internal::scalar_normal_dist_op<double>::rng.seed(1);
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    //std::cout<<"microseconds:" <<microseconds <<std::endl;
    Eigen::internal::scalar_normal_dist_op<double>::rng.seed(microseconds);
    Eigen::VectorXd mean(size);
    Eigen::MatrixXd covar(size,size);
 
    mean  <<  0,  0;
    covar <<  k*1e-0, 0,
       0,  k*1e-0;
 
    Eigen::MatrixXd normTransform(size,size);
    Eigen::LLT<Eigen::MatrixXd> cholSolver(covar);
 
    // We can only use the cholesky decomposition if
    // the covariance matrix is symmetric, pos-definite.
    // But a covariance matrix might be pos-semi-definite.
    // In that case, we'll go to an EigenSolver
    if (cholSolver.info()==Eigen::Success)
    {
    // Use cholesky solver
        normTransform = cholSolver.matrixL();
    }
    else
    {
        // Use eigen solver
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }
 
    Eigen::MatrixXd samples = (normTransform * Eigen::MatrixXd::NullaryExpr(size,nn,randN)).colwise() + mean;
//    std::cout << "Mean\n" << mean << std::endl;
//    std::cout << "Covar\n" << covar << std::endl;
//    std::cout << "Samples\n" << samples << std::endl;
    return samples;
}
 
struct mouse_info_struct { int x,y; };
struct mouse_info_struct mouse_info = {-1,-1}, last_mouse;
 
std::vector<cv::Point> groundTruth,kalmanv,measurmens;
 
void on_mouse(int event, int x, int y, int flags, void* param) {
	{

		if (event == cv::EVENT_RBUTTONDOWN)
			g_predictOnly = true;
		else if (event == cv::EVENT_LBUTTONDOWN)
			g_predictOnly = false;

		// get mouse data
		last_mouse = mouse_info;
		mouse_info.x = x;
		mouse_info.y = y;

	}
}
 
// plot points
#define CV_AA 8
#define drawCross( center, color, d )                                 \
cv::line( img, cv::Point( center.x - d, center.y - d ),                \
cv::Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
cv::line( img, cv::Point( center.x + d, center.y - d ),                \
cv::Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )
 

//------------------------------------------------------------------------------------------------
class CKalman3 {
public:
	void init(int dynamParams, int measureParams, int controlParams , float dt = 1.)
	{
		m_KF = cv::KalmanFilter(dynamParams, measureParams, controlParams); //  (4, 2, 0);
		m_state = cv::Mat_<float>(dynamParams, 1); // (x, y, Vx, Vy) 
		m_processNoise = cv::Mat(dynamParams, 1, CV_32F);
		m_measurement = cv::Mat_<float>(measureParams, 1);

		m_measurement.setTo(cv::Scalar(0));

		cv::setIdentity(m_KF.measurementMatrix);
		cv::setIdentity(m_KF.processNoiseCov, cv::Scalar::all(1e-0));
		cv::setIdentity(m_KF.measurementNoiseCov, cv::Scalar::all(k*1e-0));
		cv::setIdentity(m_KF.errorCovPost, cv::Scalar::all(.2));
		cv::setIdentity(m_KF.errorCovPre, cv::Scalar::all(.1));

		cv::Mat  transitionMatrix;
		if (dynamParams == 4)
			transitionMatrix  = (cv::Mat_<float>(4, 4) <<	1, 0, m_delta_t, 0, 
															0, 1, 0,		 m_delta_t, 
															0, 0, 1,		 0, 
															0, 0, 0, 1);
		else if (dynamParams == 6)
			transitionMatrix =  (cv::Mat_<float>(6, 6) << 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0.5, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1);

		m_KF.transitionMatrix = transitionMatrix;

	}

	cv::Point predict()
	{
		m_prediction = m_KF.predict();
		m_predictPt = cv::Point(m_prediction.at<float>(0), m_prediction.at<float>(1));
		return m_predictPt;
	}

	cv::Point correct(cv::Mat measurement)
	{
		estimated = m_KF.correct(measurement);
		statePt = cv::Point(estimated.at<float>(0), estimated.at<float>(1));
		return statePt;
	}

	void setDT(float delta_t)
	{
		m_delta_t = delta_t;
		cv::Mat  transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, m_delta_t, 0, 0, 1, 0, m_delta_t, 0, 0, 1, 0, 0, 0, 0, 1);
		m_KF.transitionMatrix = transitionMatrix;
	}


private:
	cv::KalmanFilter m_KF;
	cv::Mat_<float> m_state; /* (x, y, Vx, Vy) */
	cv::Mat m_processNoise;
	cv::Mat_<float> m_measurement;
	cv::Mat m_prediction;
	cv::Point m_predictPt;
	cv::Mat estimated;
	cv::Point statePt;
	double m_delta_t = 1 / 20.0;

};

#if 0
int test_KF_mouse () 
{
	int key = 0;
	cv::Mat img(800, 1200, CV_8UC3);
	cv::Mat_<float> measurement(2, 1); measurement.setTo(cv::Scalar(0));

	char code = char(-1);

	CKalman3 kalman;
	
    cv::namedWindow("Mouse Tracking with Kalman Filter");
    cv::setMouseCallback("Mouse Tracking with Kalman Filter", on_mouse, nullptr);
    double delta_t=1/20.0;

	//kalman.init(4, 2, 0, 1);
	kalman.init(4, 2, 0, delta_t);
	// init
    for(;;)
    {
		if (mouse_info.x < 0 || mouse_info.y < 0) {
            imshow("Mouse Tracking with Kalman Filter", img);
            cv::waitKey(30);
			continue;
		}
	
        measurmens.clear();
        groundTruth.clear();
		kalmanv.clear();
 
        for(;;)
        {
	
			cv::Point predictPt = kalman.predict();
 
            measurement(0) = mouse_info.x+multivariateNormalDistribution()(0,0);
            measurement(1) = mouse_info.y+multivariateNormalDistribution()(1,0);
 
            cv::Point groundtruth(mouse_info.x,mouse_info.y);
            groundTruth.push_back(groundtruth);
 
            cv::Point measPt(measurement(0),measurement(1));
            measurmens.push_back(measPt);


			cv::Point statePt;
			if (!g_predictOnly)
				statePt = kalman.correct(measurement);
			else
				statePt = predictPt;

			kalmanv.push_back(statePt);
 
			multivariateNormalDistribution();
 
            img = cv::Scalar::all(0);
            drawCross( statePt, cv::Scalar(255,255,255), 5 );
            drawCross( measPt, cv::Scalar(0,0,255), 5 );
            for (std::size_t i = 0; i < groundTruth.size()-1; i++)
            {
                line(img, groundTruth[i], groundTruth[i+1], cv::Scalar(0,255,0), 1);
			}
            for (std::size_t i = 0; i < kalmanv.size()-1; i++)
            {
                line(img, kalmanv[i], kalmanv[i+1], cv::Scalar(255,0,0), 1);
			}
            for (std::size_t i = 0; i < measurmens.size()-1; i++)
            {
                line(img, measurmens[i], measurmens[i+1], cv::Scalar(0,255,255), 1);
            }
            cv::putText(img,"Noisy Measurements",cv::Point(10,10),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,255) 	);
            cv::putText(img,"Real Mouse Position(ground truth)",cv::Point(10,25),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
            cv::putText(img,"Kalman Sate",cv::Point(10,35),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0));
 
 
            imshow( "Mouse Tracking with Kalman Filter", img );
            code = char(cv::waitKey(1000.0*delta_t));
			
			if (code > 0)
				break;
			else if (code == 'p')
				g_predictOnly = true;
			else if (code == 'o')
				g_predictOnly = false;
			std::cout << key << "\n";
		}
        if( code == 27 || code == 'q' || code == 'Q' )
            break;
    }
#endif 

	int test_KF_mouse()
	{
		int key = 0;
		cv::Mat img(800, 1200, CV_8UC3);
		cv::Mat_<float> measurement(2, 1); measurement.setTo(cv::Scalar(0));

		char code = char(-1);

		CKalman2 kalman;

		cv::namedWindow("Mouse Tracking with Kalman Filter");
		cv::setMouseCallback("Mouse Tracking with Kalman Filter", on_mouse, nullptr);
		double delta_t = 1 / 20.0;

		//kalman.init(4, 2, 0, 1);
		kalman.init();
		// init
		for (;;)
		{
			if (mouse_info.x < 0 || mouse_info.y < 0) {
				imshow("Mouse Tracking with Kalman Filter", img);
				cv::waitKey(30);
				continue;
			}

			measurmens.clear();
			groundTruth.clear();
			kalmanv.clear();

			for (;;)
			{

				//cv::Point predictPt = kalman.predict();
				kalman.predict();
				cv::Point predictPt = kalman.getCenter();

				measurement(0) = mouse_info.x + multivariateNormalDistribution()(0, 0);
				measurement(1) = mouse_info.y + multivariateNormalDistribution()(1, 0);

				cv::Point groundtruth(mouse_info.x, mouse_info.y);
				groundTruth.push_back(groundtruth);

				cv::Point measPt(measurement(0), measurement(1));
				measurmens.push_back(measPt);


				cv::Point statePt;
				if (!g_predictOnly) {
					//statePt = kalman.correct(measurement);
					kalman.update(measurement(0), measurement(1));
					statePt = kalman.getCenter();
				}
				else
					statePt = predictPt;

				kalmanv.push_back(statePt);

				multivariateNormalDistribution();

				img = cv::Scalar::all(0);
				drawCross(statePt, cv::Scalar(255, 255, 255), 5);
				drawCross(measPt, cv::Scalar(0, 0, 255), 5);
				for (std::size_t i = 0; i < groundTruth.size() - 1; i++)
				{
					line(img, groundTruth[i], groundTruth[i + 1], cv::Scalar(0, 255, 0), 1);
				}
				for (std::size_t i = 0; i < kalmanv.size() - 1; i++)
				{
					line(img, kalmanv[i], kalmanv[i + 1], cv::Scalar(255, 0, 0), 1);
				}
				for (std::size_t i = 0; i < measurmens.size() - 1; i++)
				{
					line(img, measurmens[i], measurmens[i + 1], cv::Scalar(0, 255, 255), 1);
				}
				cv::putText(img, "Noisy Measurements", cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
				cv::putText(img, "Real Mouse Position(ground truth)", cv::Point(10, 25), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::putText(img, "Kalman Sate", cv::Point(10, 35), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 0));


				imshow("Mouse Tracking with Kalman Filter", img);
				code = char(cv::waitKey(1000.0*delta_t));

				if (code > 0)
					break;
				else if (code == 'p')
					g_predictOnly = true;
				else if (code == 'o')
					g_predictOnly = false;
				std::cout << key << "\n";
			}
			if (code == 27 || code == 'q' || code == 'Q')
				break;
		}

#if 0
    std::ofstream groundTruthfile("groundTruth.csv",std::ofstream::ate );
    groundTruthfile<<"x,y"<<std::endl;
    for (std::size_t i = 0; i < groundTruth.size(); i++)
    {
        groundTruthfile<<groundTruth[i].x << ","<<groundTruth[i].y <<std::endl;
    }
    groundTruthfile.close();

 
 
 
    std::ofstream kalmanvfile("kalmanv.csv",std::ofstream::ate );
    kalmanvfile<<"x,y"<<std::endl;
    for (std::size_t i = 0; i < kalmanv.size()-1; i++)
    {
        kalmanvfile<<kalmanv[i].x << ","<<kalmanv[i].y <<std::endl;
    }
    kalmanvfile.close();
 
    std::ofstream measurmensfile("measurmens.csv",std::ofstream::ate );
    measurmensfile<<"x,y"<<std::endl;
    for (std::size_t i = 0; i < kalmanv.size()-1; i++)
    {
        measurmensfile<<measurmens[i].x << ","<<measurmens[i].y <<std::endl;
    }
    measurmensfile.close();
#endif 

    return 0;
}