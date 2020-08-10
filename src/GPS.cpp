//
// Created by mdleiton on 9/8/20.
//
#include "swiftnav_piksi/GPS.h"


GPS::GPS(){

};

void GPS::init(swiftnav_piksi::PIKSI& piksi){
    this->piksi = &piksi;
};

bool GPS::start(){
    piksi -> PIKSIOpen();
    return piksi -> isConnected;
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