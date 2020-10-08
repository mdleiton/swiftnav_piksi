//
// Created by mdleiton on 9/8/20.
//
#include "swiftnav_piksi/GPS.h"


GPS::GPS(){

};

void GPS::init(swiftnav_piksi::PIKSI& piksi, int n, int k, int metricType){
    this->piksi = &piksi;
    this->piksi -> setK(k);
    this->piksi -> setN(n);
    this->piksi -> setMetricType(metricType);
};

bool GPS::start(){
    piksi -> PIKSIOpen();
    return piksi -> isConnected;
}

void GPS::setOffsetNED(double offsetNED[3]){
    piksi -> setOffsetNED(offsetNED);
}

void GPS::setTime(std_msgs::Time time){
    piksi -> setTime(time);
}

bool GPS::isSendingData(){
    return piksi -> isSendingData();
}
void GPS::setSendingData(bool sendingData){
    piksi -> setSendingData(sendingData);
}

bool GPS::isConnected(){
    return piksi -> isConnected;
}

void GPS::sendData(){
    return piksi-> sendData();
}