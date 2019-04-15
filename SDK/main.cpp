#include <QCoreApplication>

#include "GblidarDriver.h"
#include "sys/select.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    GblidarDriver::OpenOptions openOptions = {
        true,                          //bool autoOpen;
        GblidarDriver::BR230400,       //波特率:230400
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
    std::string strPath("/dev/ttyUSB0");

    mGblidarDriver.SetSerialPort(strPath,openOptions); //设置串口信息

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

    while (true)
    {
        bool isOpen = mGblidarDriver.isOpen();
        //printf("--------------------------------isOpen : %d \n", isOpen);
        RadarData mRadarData = mGblidarDriver.GetScanData();   //获取一周数据
        if(mRadarData.bOk)
        {
            printf("----------------speed:%d \n",mRadarData.speed);
            float degree = 0;
            float degreeIncrement = 360.0 / INDEX_COUNT / DATA_COUNT;//=0.33;//=0.5
            for(int i = 0;i<INDEX_COUNT;i++){
                for(int j=0 ;j< DATA_COUNT;j++){
                    degree = degree + degreeIncrement;
                    if(LIDAR_TYPE == 0)//低速雷达
                    {
                        printf("%0.2f,%d,%d ",degree,mRadarData.ranges[i][j],mRadarData.intensities[i][j]);
                        //std::cout << degree << "," << data.ranges[i][j]<<"," << data.intensities[i][j]<< ";";
                    }
                    else//高速雷达
                    {
                        printf("%0.2f,%d ",degree,mRadarData.ranges[i][j]);
                        //std::cout << degree << "," << data.ranges[i][j]<< ";";
                    }
                }
            }

            printf("\n-------------end----------------\n");
        }

        usleep(50);
    }

    mGblidarDriver.close( );   //关闭
    return a.exec();
}


