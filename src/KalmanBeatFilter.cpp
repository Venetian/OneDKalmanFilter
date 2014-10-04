//
//  KalmanBeatFilter.cpp
//  KalmanFilterBeatTest
//
//  Created by Andrew Robertson on 03/10/2014.
//
//

#include "KalmanBeatFilter.h"


//Kslman filter for beat tracking
//observe points ON THE BEAT
//so we have x, the time is ms
//and delta_x, or v, considered the velocity or the beat period






KalmanBeatFilter::KalmanBeatFilter(){
    KF.init(2, 2, 0);
	
    
    processSigmaX = 5;//std dev for process noise x in msec, does the underlying beat move?
    //yes I suppose so
    
    processSigmaV = 4;//std dev for period v
    
    
    measureSigmaX = 10;//std dev for x in msec, measuring the expressive timing, observation error
    measureSigmaV = 6;//std dev for period v
    
    
    //x_n = x_(n-1) + v_(n-1) + noise
    //v_n = v_{n-1} + noise
    
    //the noise here is process noise - perhaps the movement of underlying beat
    
    //the measurement noise would be error in observation, and say expressive timing
    
    
    
	KF.transitionMatrix = *(cv::Mat_<float>(2, 2) <<
                            1,1,
                            0,1);
    
    measurement = cv::Mat_<float>::zeros(2,1);
    
    init();
    
    //an example set of observations: first is beat position, then the period
    
    if (false){//set to true for example of this working
        newMeasurement(510, 505);
        newMeasurement(1012, 506.4);

        newMeasurement(1502, 502.4);
        newMeasurement(2008, 500.4);
    
        newMeasurement(2548, 507.4);
    
        newMeasurement(2978, 500.4);
    }
    
}




KalmanBeatFilter::~KalmanBeatFilter(){
    

}


void KalmanBeatFilter::setInitialTempo(float beatTime, float period){
    latestPositionEstimate = beatTime;
    latestPeriodEstimate = period;
    KF.statePost = *(cv::Mat_<float>(2,1) << beatTime, period);
}

void KalmanBeatFilter::init(){
    setInitialTempo(0, 500);
    
    //measurement we will set to identity
    //as we will provide an estimate for the period and for the beat location
    cv::setIdentity(KF.measurementMatrix);
    
    
    //< process noise covariance matrix (Q)
   // cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
    
    KF.processNoiseCov = *(cv::Mat_<float>(2, 2) <<
                           processSigmaX*processSigmaX, 0,
                           0, processSigmaV*processSigmaV);
    
    
    //measurement noise covariance matrix (R)
   // cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    
    KF.measurementNoiseCov = *(cv::Mat_<float>(2, 2) <<
                           measureSigmaX*measureSigmaX,0,
                           0,measureSigmaV*measureSigmaV);
    
    
    //priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)
    //cv::setIdentity(KF.errorCovPost, cv::Scalar::all(.1));
    
    KF.errorCovPost = *(cv::Mat_<float>(2, 2) <<
                        15,0,
                        0,3);


    
}


void KalmanBeatFilter::newMeasurement(float observedX, float observedV){
    
    cv::Mat prediction = KF.predict();
    
    printf("\nKBF: observed %f, %f\n", observedX, observedV);
    
    
    printf("KBF: preditc %f, %f\n", KF.statePre.at<float>(0), KF.statePre.at<float>(1));
    
    measurement(0) = observedX;
    measurement(1) = observedV;
    
    
    
    cv::Mat estimated = KF.correct(measurement);
    
    latestPositionEstimate = estimated.at<float>(0);
    latestPeriodEstimate = estimated.at<float>(1);
    
    printf("KBF: estimate %f, %f\n", latestPositionEstimate, latestPeriodEstimate);
    
}

float KalmanBeatFilter::latestPrediction(){
    return latestPositionEstimate;
}