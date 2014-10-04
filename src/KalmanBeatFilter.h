//
//  KalmanBeatFilter.h
//  KalmanFilterBeatTest
//
//  Created by Andrew Robertson on 03/10/2014.
//
//  do anything you like with it

#ifndef __KalmanFilterBeatTest__KalmanBeatFilter__
#define __KalmanFilterBeatTest__KalmanBeatFilter__


#include "ofxCv.h"

//generated with project generator so headers are linked in search path
//and other linker has opencv library

#include <iostream>

class KalmanBeatFilter{
public:
    
    KalmanBeatFilter();
    ~KalmanBeatFilter();
    
    void init();
    
    void newMeasurement(float observedX, float observedV);
    void setInitialTempo(float beatTime, float period);
    
    float processSigmaX, processSigmaV;
    
    float measureSigmaX, measureSigmaV;
    
    cv::KalmanFilter KF;
	cv::Mat_<float> measurement;
    
    float latestPrediction();
    
    float latestPositionEstimate;
    float latestPeriodEstimate;
};

#endif /* defined(__KalmanFilterBeatTest__KalmanBeatFilter__) */
