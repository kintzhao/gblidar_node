
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <serial/serial.h>
#include <iostream>
#include "GBlidarDriver.h"
#include <stdlib.h>
#include <string.h>
using namespace std; 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidarNode");
    std::string serialPort;
    int serialBaudrate;
    std::string frameId;

    ros::NodeHandle nhPrivate("~");

    nhPrivate.param<std::string>("serial_port", serialPort, "/dev/ttyUSB0");
    nhPrivate.param<int>("serial_baudrate", serialBaudrate, 460800);
    nhPrivate.param<std::string>("frame_id", frameId, "laser_frame");

    GBlidarRead gblidarRead;
    gblidarRead.setFrameId(frameId);
    gblidarRead.Connect(serialPort.c_str(), serialBaudrate);
    if(gblidarRead.CheckConnect())
    {
        //std::cout << "CheckConnect succuss.." << std::endl;
        gblidarRead.work2();
    }
    else
    {
        //std::cout << "CheckConnect error.." << std::endl;
        return -1;
    }

    //ros::spin();
    return 0;
}
