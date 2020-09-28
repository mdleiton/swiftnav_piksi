/***************************************************************************//**
 * \file piksi_node.cpp
 *
 * \brief Single GPS node
 * \author Scott K Logan
 * \author Caleb Jamison
 * \author Mauricio Leiton
 * \date February 23, 2014
 *
 * This binary creates a simple node for communication with a single Swift
 * Navigation Piksi GPS.
 *
 * \section license License (BSD-3)
 * Copyright (c) 2013, Scott K Logan\n
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * - Neither the name of Willow Garage, Inc. nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include "swiftnav_piksi/driver.h"
#include "swiftnav_piksi/GPS.h"
#include <ros/ros.h>
#include <cstdlib>
#include <std_srvs/SetBool.h>
#include <iostream>
#include <string>
#include "std_msgs/Time.h"
#include "control_pi/utils.h"

GPS *gps;
int k=5, n=10, metricType;

bool EstadoPiksi(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
    if(gps->isConnected()){
        std::string pi = "Nodo Piksi ejecutándose. Obteniendo datos: " + bool_str(gps->isSendingData());
        res.message = pi;
    }else{
        std::string pi = "Nodo Piksi no está ejecutándose.";
        res.message = pi;
    }
    res.success =  gps->isConnected();
    ROS_INFO("Consultado estado de Piksi. Respuesta : [%s]-[%s]",bool_str(res.success).c_str(), res.message.c_str());
    return true;
}

void inicio(std_msgs::Time time){
    gps->setTime(time);
    gps->sendData();
}

int main( int argc, char *argv[] ){
    ros::init( argc, argv, "piksi_node" );

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv( "~" );

    std::string port;
    nh_priv.param( "port", port, (const std::string)"/dev/ttyUSB0");

    swiftnav_piksi::PIKSI piksi( nh, nh_priv, port);
    gps = new GPS();
    gps->init(piksi, n, k, metricType);
    ros::ServiceServer s_estado = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("EstadoPiksi", EstadoPiksi);
    ros::Subscriber sub = nh.subscribe("inicioLectura", 1, inicio);
    ROS_DEBUG( "Opening Piksi on %s", port.c_str( ) );
    if(!gps->start()){
        ROS_ERROR( "Failed to open Piksi on %s", port.c_str( ) );
    }else{
        ROS_INFO( "Piksi opened successfully on %s", port.c_str( ) );
    }
    ros::spin();
    std::exit( EXIT_SUCCESS );
}

