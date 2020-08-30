//
// Created by mdleiton on 9/8/20.
//

#ifndef SENSORESCVR_GPS_H
#define SENSORESCVR_GPS_H
#include "swiftnav_piksi/driver.h"

class GPS {

    public:
        GPS();
        swiftnav_piksi::PIKSI *piksi;

        void init(swiftnav_piksi::PIKSI &piksi, int n, int k, int metricType);
        bool start();
        void setTime(std_msgs::Time time);
        bool isSendingData();
        void setSendingData(bool sendingData);
        bool isConnected();
        void sendData();

};

#endif //SENSORESCVR_GPS_H
