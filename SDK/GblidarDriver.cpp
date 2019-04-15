#include "GblidarDriver.h"

/************************************
 功能：构造函数
 参数：无
 返回值：无
 ***********************************/
GblidarDriver::GblidarDriver()
{
    this->_is_Complete_Data = false;
    this->_tty_fd = 0;
    this->epid = epoll_create(6);

    _radarData.speed = -1;
    for(int i =0;i<INDEX_COUNT;i++){
        for(int j =0;j<DATA_COUNT; j++){
            _radarData.ranges[i][j] = -1;
            _radarData.intensities[i][j] = -1;
        }
    }
}

/************************************
 功能：析构函数
 参数：无
 返回值：无
 ***********************************/
GblidarDriver::~GblidarDriver() {
    close();
    _radarData.speed = -1;
    for(int i =0;i<INDEX_COUNT;i++){
        for(int j =0;j<DATA_COUNT; j++){
            _radarData.ranges[i][j] = -1;
            _radarData.intensities[i][j] = -1;
        }
    }
}




bool GblidarDriver::SetSerialPort(const std::string &path, const OpenOptions options){
    _path = path;
    _open_options = options;
    if(options.autoOpen) {
        _is_open = open(_path, _open_options);
    }
    return true;
}


bool GblidarDriver::open() {
    _is_open = open(_path, _open_options);

    if(_is_open)
    {
        //下面开始创建接受线程。
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        // 设置线程绑定属性
        int res = pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        // 设置线程分离属性
        res += pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        //创建线程
        pthread_create(&pid, &attr, ReadThreadFunction, (void *) this);
    }
    return _is_open;
}


bool GblidarDriver::open(const std::string &path, const OpenOptions &options) {

    if(_path != path) _path = path;
    if(_open_options != options) _open_options = options;

    _tty_fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(_tty_fd < 0) {
        return false;
    }


    struct termios  tios;
    termiosOptions(tios, options);
    tcsetattr(_tty_fd, TCSANOW, &tios);
    tcflush(_tty_fd, TCIOFLUSH);

    return true;
}

void GblidarDriver::termiosOptions(termios &tios, const OpenOptions &options) {

    tcgetattr(_tty_fd, &tios);

    tios.c_oflag = 0;
    tios.c_iflag = 0;
    tios.c_lflag = 0;

    cfsetispeed(&tios, options.baudRate);
    cfsetospeed(&tios, options.baudRate);

    tios.c_iflag |= (options.xon ? IXON : 0)
            | (options.xoff ? IXOFF: 0)
            | (options.xany ? IXANY : 0);

    // data bits

    int databits[] =  {CS5, CS6, CS7, CS8};
    tios.c_cflag &= ~0x30;
    tios.c_cflag |= databits[options.dataBits];

    // stop bits
    if(options.stopBits == StopBits2) {
        tios.c_cflag |= CSTOPB;
    } else {
        tios.c_cflag &= ~CSTOPB;
    }

    // parity
    if(options.parity == ParityNone) {
        tios.c_cflag &= ~PARENB;
    } else {
        tios.c_cflag |= PARENB;

        if(options.parity == PariteMark) {
            tios.c_cflag |= PARMRK;
        } else {
            tios.c_cflag &= ~PARMRK;
        }

        if(options.parity == ParityOdd) {
            tios.c_cflag |= PARODD;
        } else {
            tios.c_cflag &= ~PARODD;
        }
    }

    tios.c_cc[VMIN] = options.vmin;
    tios.c_cc[VTIME] = options.vtime;
}

bool GblidarDriver::isOpen() const {
    return _is_open;
}

int GblidarDriver::write(const void *data, int length) {
    return ::write(_tty_fd, data, length);
}

int GblidarDriver::read(void *data, int length) {
    return ::read(_tty_fd, data, length);
}

void GblidarDriver::close() {
    ::close(_tty_fd);
    _is_open = false;
}

std::vector<std::string> GblidarDriver::list() {
    DIR *dir;
    struct dirent *ent;
    dir = opendir("/dev");
    std::vector<std::string> ttyList;

    while(ent = readdir(dir), ent != NULL) {//TODO nullptr ==> NULL
        if("tty" == std::string(ent->d_name).substr(0, 3)) {
            ttyList.push_back(ent->d_name);//TODO emplace_back ==> push_back
        }
    }

    return ttyList;
}
bool operator==(const GblidarDriver::OpenOptions& lhs, const GblidarDriver::OpenOptions& rhs)
{
    return lhs.autoOpen == rhs.autoOpen
            && lhs.baudRate == rhs.baudRate
            && lhs.dataBits == rhs.dataBits
            && lhs.parity == rhs.parity
            && lhs.stopBits == rhs.stopBits
            && lhs.vmin == rhs.vmin
            && lhs.vtime == rhs.vtime
            && lhs.xon == rhs.xon
            && lhs.xoff == rhs.xoff
            && lhs.xany == rhs.xany;
}

bool operator!=(const GblidarDriver::OpenOptions& lhs, const GblidarDriver::OpenOptions& rhs){
    return !(lhs == rhs);
}


/************************************
 功能：数据接收线程的函数
 参数：对象指针
 返回值： 无
 ***********************************/
void * GblidarDriver::ReadThreadFunction(void *arg) {
    printf("------------start ReadThread--------------\n");

    GblidarDriver *pGblidarDriver = (GblidarDriver*) arg;

    //epoll设置
    pGblidarDriver->event.data.fd = pGblidarDriver->_tty_fd;
    pGblidarDriver->event.events = EPOLLET | EPOLLIN;
    if (epoll_ctl(pGblidarDriver->epid, EPOLL_CTL_ADD, pGblidarDriver->_tty_fd, &pGblidarDriver->event) != 0) { //将读事件添加到epoll的事件队列中
        printf("set epoll error!\n");
        return NULL;
    }
    printf("------------set epoll ok!-----------------\n");

    //下面开始epoll等待
    bool bLastFA = false;
    uint8_t tempData[200];
    int tempPos = 0;

    while (true) {
        //接受数据
        int len = pGblidarDriver->read(pGblidarDriver->RecvBuff, MAXLEN);

        if(len > 0)
        {
            if(SHOW_LOG > 0)
            {
                printf("GBlidarRead::ReadData buffer length:%d-----tempPos:%d\n",len,tempPos);
            }

            for(int i = 0; i < len; i++)
            {
                if(pGblidarDriver->RecvBuff[i] == 0xfa )
                {
                    //printf("\n---------------------\n");
                    if(bLastFA)
                    {
                        pGblidarDriver->processOneBlock(tempData, tempPos);//分割一条数据

                    }
                    bLastFA = true;
                    tempPos = 0;
                    tempData[tempPos++] = pGblidarDriver->RecvBuff[i];
                }
                else if(bLastFA)
                {
                    tempData[tempPos++] = pGblidarDriver->RecvBuff[i];
                }
            }
        }
    }
    return NULL;
}


//检查是否是完整的语句或者是完整的开头语句
void GblidarDriver::CheckLineBeComplete(uint8_t* data, int size, bool* bComplete, bool* bStartLine)
{
    if(size == (DATA_COUNT * DATA_LEN + 6)){
        if(data[0] == 0xFA){//如果首位是0xFA
            int sum = 0;
            for(int i = 0;i<size -2;i++){//最后2位是校验位
                sum += (int)(data[i]);
            }
            int checkSum = (int)(data[size -1]<<8) + (int)(data[size -2]);
            if((int)sum == checkSum){
                *bComplete = true;
                if(data[1] == 0x00){//校验成功了再去验证是不是首帧数据
                    *bStartLine = true;
                }else{
                    *bStartLine = false;
                }
            }else{
                *bComplete = false;
            }
        }

    }else{
        *bComplete = false;
        *bStartLine = false;
    }
}

//解析一帧数据
void GblidarDriver::processOneBlock(uint8_t* data, int size)
{
    static uint8_t dataTemp[DATA_COUNT * DATA_LEN + 6] = {'\0'};//缓存一条语句
    static int tempCount = 0;
    //打印输出

    if(SHOW_LOG == 2)
    {
        for(int i = 0; i < size; i++)
        {
            char str[3] = {'\0'};
            char buf = (char)(data[i]);
            Hex2Str(&buf,str,1);
            printf("%s ", str);
        }
        printf("\n------------------------\n");
    }

    bool bComplete = false;
    bool bStart = false;
    CheckLineBeComplete(data, size, &bComplete, &bStart);


    if(bComplete)
    {
        if(tempCount == (DATA_COUNT * DATA_LEN + 6))
        {
            bool bComplete2 = false;
            bool bStart2 = false;
            CheckLineBeComplete(dataTemp, tempCount, &bComplete2, &bStart2);//测试校验上一帧不完整数据拼接起来的数据是否正确

            //获取index
            int index = (int)(dataTemp[1]);


            if(index >= 0 && index<= INDEX_COUNT)
            {
                blockUpdateFlag_.push_back(index);
                //更新转速
                _radarData.speed = (int)(dataTemp[3] << 8) + (int)(dataTemp[2]);

                if(SHOW_LOG > 0)
                {
                    //printf("------------完整一帧数据end-------index:%d------speed:%d-----\n",index,_radarData.speed);
                    if(index == INDEX_COUNT -1 )
                    {
                        printf("------------最后一帧数据1---------------------------------------\n");
                    }
                }

                if(SHOW_LOG == 2)
                {
                    printf("------------index1:%d,speed:%d------",index,_radarData.speed);
                }

                for(int i = 0; i < DATA_COUNT; i++)
                {
                    //距离数据
                    int curDis = (int)(dataTemp[4 + DATA_LEN*i + 1] << 8) + (int)(dataTemp[4 + DATA_LEN*i]);
                    if(curDis == 32768)
                    {
                        curDis = -1;
                    }
                    _radarData.ranges[index][i] = curDis;

                    if(LIDAR_TYPE == 0)//低速雷达的强度数据
                    {
                        int curintensities = (int)(dataTemp[4 + DATA_LEN*i + 3] << 8) + (int)(dataTemp[4 + DATA_LEN*i + 2]);
                        if(curintensities == 32768)
                        {
                            curintensities = -1;
                        }
                        _radarData.intensities[index][i] = curintensities;
                    }

                    if(SHOW_LOG == 2)
                    {
                        printf("%d:%d,%d;",curDis,_radarData.ranges[index][i],_radarData.intensities[index][i]);
                    }
                }

                if(SHOW_LOG == 2)
                {
                    printf("---\n");
                }

                tempCount = 0;
                memset(dataTemp, '\0', DATA_COUNT * DATA_LEN + 6);
            }
        }

        if(bStart)
        {
            start_scan_time = time(NULL);
            //下一组数据来了，请在这里处理解析好的一组数据，

            if(blockUpdateFlag_.size() == INDEX_COUNT)
            {
                if(SHOW_LOG > 0)
                {
                     printf("-----完整一周３６０度的数据--\n");
                }

                end_scan_time = time(NULL);
            }

            //这里清除这一组数据
            blockUpdateFlag_.clear();
            this->_is_Complete_Data = true;
            //_radarData.ClearData();
        }

        //缓存断层的帧数据
        int index2 = (int)(data[1]);//获取index
        if(index2 >= 0 && index2 <= INDEX_COUNT - 1 && size == (DATA_COUNT * DATA_LEN + 6))//54 = 6 + 24 * 2
        {
            blockUpdateFlag_.push_back(index2);
            //更新转速
            _radarData.speed = (int)(data[3] << 8) + (int)(data[2]);

            if(SHOW_LOG > 0)
            {
                //printf("------------完整一帧数据end-------index:%d-----speed:%d------\n",index2,_radarData.speed);

                if(index2 == INDEX_COUNT -1 )
                {
                    printf("------------最后一帧数据2------------\n");
                }
            }

            if(SHOW_LOG == 2)
            {
                printf("------------index2:%d,speed:%d------",index2,_radarData.speed);
            }

            for(int i = 0; i < DATA_COUNT; i++)
            {
                //距离数据
                int curDis = (int)(data[4 + DATA_LEN*i + 1] << 8) + (int)(data[4 + DATA_LEN*i]);
                if(curDis == 32768)
                {
                    curDis = -1;
                }

                _radarData.ranges[index2][i] = curDis;
                if(LIDAR_TYPE == 0)//低速雷达的强度数据
                {
                    int curintensities = (int)(data[4 + DATA_LEN*i + 3] << 8) + (int)(data[4 + DATA_LEN*i + 2]);
                    if(curintensities == 32768)
                    {
                        curintensities = -1;
                    }
                    _radarData.intensities[index2][i] = curintensities;
                }

                if(SHOW_LOG == 2)
                {
                    printf("%d:%d,%d;",curDis,_radarData.ranges[index2][i],_radarData.intensities[index2][i]);
                }
            }

            if(SHOW_LOG == 2)
            {
                printf("---\n");
            }
        }

    }
    else
    {
        if(tempCount + size <= (DATA_COUNT * DATA_LEN + 6))
        {
            memcpy(dataTemp + tempCount, data, size);
            tempCount += size;
        }
    }
}

//16进制转字符串
void GblidarDriver::Hex2Str( const char *sSrc,  char *sDest, int nSrcLen )
{
    int  i;
    char szTmp[3];

    for( i = 0; i < nSrcLen; i++ )
    {
        sprintf( szTmp, "%02X", (unsigned char) sSrc[i] );
        memcpy( &sDest[i * 2], szTmp, 2 );
    }
}

RadarData GblidarDriver::GetScanData()
{
    if(this->_is_Complete_Data)
    {
        this->_is_Complete_Data = false;
        this->_radarData.bOk = true;
    }
    else {
        this->_radarData.bOk = false;
    }
    return this->_radarData;
}
