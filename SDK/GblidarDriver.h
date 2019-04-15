#ifndef GBLIDARDRIVER_H
#define GBLIDARDRIVER_H


#include <string>
#include <vector>

#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include "sys/time.h"
#include "sys/epoll.h"
#include "string.h"

#include <stdio.h>

/*
*高速雷达0.5:INDEX_COUNT=30,DATA_COUNT=24,DATA_LEN=2,LIDAR_TYPE=1
*高速雷达0.33:INDEX_COUNT=30,DATA_COUNT=36,DATA_LEN=2,LIDAR_TYPE=1
*低速雷达:INDEX_COUNT=90,DATA_COUNT=4,DATA_LEN=4,LIDAR_TYPE=0
*/
#define INDEX_COUNT 90      //角度编号
#define DATA_COUNT 4      //一帧数据包数
#define DATA_LEN 4          //
#define LIDAR_TYPE 0        //雷达类型　０＝低速雷达，１＝高速雷达

#define SHOW_LOG 2          //是否打印log,2=原始报文打印;1=基本打印，0=不打印

struct RadarData
{
   bool bOk;                                //是否完整一周
   int speed;                               //马达转速
   int ranges[INDEX_COUNT][DATA_COUNT];     //距离数据
   int intensities[INDEX_COUNT][DATA_COUNT];//强度数据

   void ClearData()
   {
       bOk = false;
        speed = 0;
        memset(ranges, '\0', INDEX_COUNT * DATA_COUNT);
        memset(intensities, '\0', INDEX_COUNT * DATA_COUNT);
   }
};


#define MAXLEN 1024*10

struct termios;
class GblidarDriver
{
public:
    //波特率
    enum BaudRate {
        BR0 = 0000000,
        BR50 = 0000001,
        BR75 = 0000002,
        BR110 = 0000003,
        BR134 = 0000004,
        BR150 = 0000005,
        BR200 = 0000006,
        BR300 = 0000007,
        BR600 = 0000010,
        BR1200 = 0000011,
        BR1800 = 0000012,
        BR2400 = 0000013,
        BR4800 = 0000014,
        BR9600 = 0000015,
        BR19200 = 0000016,
        BR38400 = 0000017,
        BR57600 = 0010001,
        BR115200 = 0010002,
        BR230400 = 0010003,
        BR460800 = 0010004,
        BR500000 = 0010005,
        BR576000 = 0010006,
        BR921600 = 0010007,
        BR1000000 = 0010010,
        BR1152000 = 0010011,
        BR1500000 = 0010012,
        BR2000000 = 0010013,
        BR2500000 = 0010014,
        BR3000000 = 0010015,
        BR3500000 = 0010016,
        BR4000000 = 0010017
    };

    enum DataBits {
        DataBits5,
        DataBits6,
        DataBits7,
        DataBits8,
    };

    enum StopBits {
        StopBits1,
        StopBits2
    };

    enum Parity {
        ParityNone,
        ParityEven,
        PariteMark,
        ParityOdd,
        ParitySpace
    };

    struct OpenOptions {
        bool autoOpen;          //
        BaudRate baudRate;     //波特率
        DataBits dataBits;
        StopBits stopBits;
        Parity parity;
        bool xon;
        bool xoff;
        bool xany;
        int vmin;
        int vtime;
    };

    static const OpenOptions defaultOptions;

    GblidarDriver();
    virtual ~GblidarDriver();

    bool SetSerialPort(const std::string& path, const OpenOptions options);

    bool open();
    bool open(const std::string& path, const OpenOptions& options);

    bool isOpen() const;

    int write(const void *data, int length);
    int read(void *data, int length);

    RadarData GetScanData();
    bool GetScanData(RadarData& cacheData){ return true;} //TODO

    void close();

    static std::vector<std::string > list();

protected:

    void termiosOptions(termios& tios, const OpenOptions& options);


private:
    std::string _path;
    OpenOptions _open_options;
    int _tty_fd;
    bool _is_open;

    bool _is_Complete_Data;

    int epid; //epoll标识符
    epoll_event event;
    epoll_event events[6];//事件集合
    uint8_t RecvBuff[MAXLEN];//接受到的数据
    pthread_t pid;//接受数据线程的Id
    static void * ReadThreadFunction(void * arg);//接受数据的线程函数

    time_t start_scan_time;
    time_t end_scan_time;
    std::vector<int> blockUpdateFlag_;
    RadarData _radarData;
    void CheckLineBeComplete(uint8_t* data, int size, bool* bComplete, bool* bStartLine);
    void processOneBlock(uint8_t* data,const int size);
    void Hex2Str( const char *sSrc,  char *sDest, int nSrcLen );
};


bool operator==(const GblidarDriver::OpenOptions& lhs, const GblidarDriver::OpenOptions& rhs);
bool operator!=(const GblidarDriver::OpenOptions& lhs, const GblidarDriver::OpenOptions& rhs);


#endif // GBLIDARDRIVER_H
