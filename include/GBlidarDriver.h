
#ifndef GBLIBARDRIVER_H_
#define GBLIBARDRIVER_H_
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <serial/serial.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

//#define DATACOUNT 24 //0.5
 #define DATACOUNT 36 //0.33

struct RadarData
{
   int speed;
   int data[30][DATACOUNT];//数据

   void ClearData()
   {
		speed = 0;
        memset(data, '\0', 30*DATACOUNT);
   }
};


class GBlidarRead
{
public:
	GBlidarRead();
    virtual ~GBlidarRead();
	void Connect(const char* portName,int baudrate);
	void DisConnect();
	bool CheckConnect();
    void ReadData();
    void setFrameId(const std::string& frame_id){frameId = frame_id;}

private:
	void CheckLineBeComplete(uint8_t* data, int size, bool* bComplete, bool* bStartLine);
	void AddOneLineData(uint8_t* data, int size);
    void processOneBlock(uint8_t* data, int size);
	void Hex2Str( const char *sSrc,  char *sDest, int nSrcLen );
    void updateLidarDatas();

private:
    RadarData radarData;
	serial::Serial sp;


private:
    ros::NodeHandle nh_;
    ros::Publisher lidarPub_;
    ros::NodeHandle nhPrivate_;
public:
    std::string serialPort;
    int serialBaudrate;
    std::string frameId;

    //GBlidarRead gblidarRead;
    //实现父类的虚方法
    std::vector<int> blockUpdateFlag_;
    void Update(const RadarData&  data);
    void init(){}
    void work();
    void work2();
};


#endif /* GBLIBARDRIVER_H_ */
