#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <serial/serial.h>
#include <iostream>
#include "GblidarDriver.h"
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
    nhPrivate.param<int>("serial_baudrate", serialBaudrate, 460800);//TODO:: need transform
    nhPrivate.param<std::string>("frame_id", frameId, "laser_frame");
    ros::Publisher lidarPub_ = nhPrivate.advertise<sensor_msgs::LaserScan>("scan", 10);
    GblidarDriver::OpenOptions openOptions = {
        true,                          //bool autoOpen;
        GblidarDriver::BR230400,       //波特率:230400    //TODO:: need transform
        GblidarDriver::DataBits8,      // 数据位:8
        GblidarDriver::StopBits1,      //停止位:1
        GblidarDriver::ParityNone,     //流控:None;
        false,                         // input xon
        false,                         // input xoff
        false,                         // input xany
        0,                             // c_cc vmin
        50,                            // c_cc vtime
    };


    GblidarDriver mGblidarDriver;
    //std::string strPath("/dev/ttyUSB0");

    mGblidarDriver.SetSerialPort(serialPort,openOptions); //设置串口信息

    if(mGblidarDriver.open())                          //打开串口连接
    {
        printf("CheckConnect succuss..\n");
        //std::cout << "CheckConnect succuss.." << std::endl;
    }
    else
    {
        printf("CheckConnect error..\n");
        //std::cout << "CheckConnect error.." << std::endl;
        return -1;
    }

    bool isOpen = mGblidarDriver.isOpen();
    //printf("--------------------------------isOpen : %d \n", isOpen);
    while (isOpen && ros::ok())
    {
        ros::Time start_scan_time = ros::Time::now();
        RadarData mRadarData;  //TODO:: RadarData's ranges and intensities need store as One-dimensional data
        bool isUpdateOk = mGblidarDriver.GetScanData(mRadarData);//TODO:: need transform
        if(isUpdateOk)
        {
            ros::Time end_scan_time = ros::Time::now();

            float scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

            sensor_msgs::LaserScan scan_msg;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frameId;

            scan_msg.angle_min =  -M_PI;
            scan_msg.angle_max =  M_PI;

            scan_msg.angle_increment = -(scan_msg.angle_max - scan_msg.angle_min)/(INDEX_COUNT*DATA_COUNT);//-12.0/DATACOUNT*M_PI/180;

            scan_msg.scan_time = scan_duration;
            scan_msg.time_increment = scan_duration/(INDEX_COUNT*DATA_COUNT);
            scan_msg.range_min = 0.05;
            scan_msg.range_max = 15;//8.0;

            scan_msg.intensities.resize(INDEX_COUNT*DATA_COUNT);
            scan_msg.ranges.resize(INDEX_COUNT*DATA_COUNT);

            //float degree = 0;
            for(int i = 0;i<INDEX_COUNT;i++){
                //std::cout << "-degree:" << degree << ",distance(m):"<< data.data[i][0]*0.001 << std::endl;
                for(int j=0 ;j< DATA_COUNT;j++){
                    scan_msg.ranges[i*DATA_COUNT+j] = mRadarData.ranges[i][j]*0.001;//m
                    scan_msg.intensities[i*DATA_COUNT+j] = mRadarData.intensities[i][j];
                }
            }

            lidarPub_.publish(scan_msg);
        }

        ros::spinOnce();
    }

    mGblidarDriver.close( );   //关闭
    return 0;
}
