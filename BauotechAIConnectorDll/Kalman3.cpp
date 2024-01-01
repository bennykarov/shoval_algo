#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/videoio.hpp"


#include "utils.hpp"
#include "Kalman3.hpp"

using namespace cv;


/*
template <class T>
void CKalman3::init(float x, float y)
{

}
*/
template <class T>
void CKalman3_<T>::init(T smoothness, T rapidness, bool bUseAccel) {
	if (bUseAccel) {
		KF.init(9, 3, 0); // 9 variables (position+velocity+accel) and 3 measurements (position)

		KF.transitionMatrix = (Mat_<T>(9, 9) <<
			1, 0, 0, 1, 0, 0, 0.5, 0, 0,
			0, 1, 0, 0, 1, 0, 0, 0.5, 0,
			0, 0, 1, 0, 0, 1, 0, 0, 0.5,
			0, 0, 0, 1, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 1,
			0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1);

		measurement = Mat_<T>::zeros(3, 1);

		KF.statePre = Mat_<T>::zeros(9, 1);
	}
	else {
		KF.init(6, 3, 0); // 6 variables (position+velocity) and 3 measurements (position)

		KF.transitionMatrix = (Mat_<T>(6, 6) <<
			1, 0, 0, 1, 0, 0,
			0, 1, 0, 0, 1, 0,
			0, 0, 1, 0, 0, 1,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1);

		measurement = Mat_<T>::zeros(3, 1);

		KF.statePre = Mat_<T>::zeros(6, 1);
	}
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(smoothness));
	setIdentity(KF.measurementNoiseCov, Scalar::all(rapidness));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
}


template <class T>
void CKalman3_<T>::update(T x, T y) {
	// First predict, to update the internal statePre variable
	prediction = KF.predict();

	// The "correct" phase that is going to use the predicted value and our measurement
	measurement(0) = x;
	measurement(1) = y;
	measurement(2) = 0;
	estimated = KF.correct(measurement);
}

#if 0
void CKalman3_::predict()
{
	// First predict, to update the internal statePre variable
	Mat prediction = m_KF.predict();
	m_predictPt = Point(prediction.at<float>(0), prediction.at<float>(1));
/*
	// Get mouse point
	m_measurement(0) = x;
	m_measurement(1) = y;
	Point measPt(m_measurement(0), m_measurement(1));
	// The "correct" phase that is going to use the predicted value and our measurement

	Mat estimated = m_KF.correct(m_measurement);

	m_statePt = cv::Point2f(estimated.at<float>(0), estimated.at<float>(1));
*/
}
#endif 