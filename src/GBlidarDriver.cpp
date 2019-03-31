
#include "GBlidarDriver.h"

GBlidarRead::GBlidarRead():nhPrivate_("~")
{
    //ROS_INFO_STREAM("GBlidarRead create.--------.");
    std::cout << "GBlidarRead create.." << std::endl;
    radarData.speed = -1;
    for(int i =0;i<30;i++){
        for(int j =0;j<DATACOUNT; j++){
            radarData.data[i][j] = -1;
        }
    }
    //init();
    lidarPub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
}

GBlidarRead::~GBlidarRead()
{
    sp.close();
    //ROS_INFO_STREAM("GBlidarRead free.--------.");
    std::cout << "GBlidarRead free.." << std::endl;
}

void GBlidarRead::Connect(const char* portName,int baudrate)
{
    std::cout << "GBlidarRead Connect..1" << std::endl;

    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort(portName);//"/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(baudrate);//
    //串口设置timeout
    sp.setTimeout(to);
    std::cout << "GBlidarRead Connect..2" << std::endl;
    try
    {
        //打开串口
        std::cout << "GBlidarRead Connect..open1" << std::endl;
        sp.open();
        std::cout << "GBlidarRead Connect..open2" << std::endl;
    }
    catch(serial::IOException& e)
    {
        std::cout << "GBlidarRead Connect..3" << std::endl;
    }
    std::cout << "GBlidarRead Connect..4" << std::endl;
}

void GBlidarRead::DisConnect()
{
    sp.close();
}

bool GBlidarRead::CheckConnect()
{
    return sp.isOpen();
}

void GBlidarRead::ReadData()
{
    std::cout << "GBlidarRead::ReadData start.." << std::endl;
    bool bLastFA = false;
    uint8_t tempData[200]; 
    int tempPos = 0;
    while(true)
    {
        //获取缓冲区内的字节数
        //size_t n = sp.available();

        int n = sp.available();
        std::cout << "GBlidarRead::ReadData 2..|| "<< n << std::endl;
        if(n!=0)
        {
            uint8_t buffer[1024 * 1024];
            //读出数据
            //std::cout << "GBlidarRead::ReadData 3.." << std::endl;
            n = sp.read(buffer, n);
            std::cout << "GBlidarRead::ReadData buffer length:" << n<< std::endl;
            for(int i = 0; i < n; i++)
            {
                if(buffer[i] == 0xFA)
                {
                    if(bLastFA)
                    {
                        AddOneLineData(tempData, tempPos);//分割一条数据
                    }
                    bLastFA = true;
                    tempPos = 0;
                    tempData[tempPos++] = buffer[i];
                }
                else if(bLastFA)
                {
                    tempData[tempPos++] = buffer[i];
                }
            }
        }
    }
    std::cout << "GBlidarRead::ReadData end.." << std::endl;
}

//检查是否是完整的语句或者是完整的开头语句
void GBlidarRead::CheckLineBeComplete(uint8_t* data, int size, bool* bComplete, bool* bStartLine)
{
    if(size == (DATACOUNT * 2 + 6)){

        if(data[0] == 0xFA){//如果首位是0xFA
            int sum = 0;
            for(int i = 0;i<size -2;i++){//最后2位是校验位
                sum += (int)(data[i]);
            }     
            int checkSum = (int)(data[size -1]<<8) + (int)(data[size -2]);
            if((int)sum == checkSum){
                *bComplete = true;
                //std::cout << "-----校验成功，校验和：" << std::hex <<std::setw(2)<<std::setfill('0')<<(data[size -1] & 0xff)<< std::endl;
                if(data[1] == 0x00){//校验成功了再去验证是不是首帧数据
                    *bStartLine = true;
                }else{
                    *bStartLine = false;
                }
            }else{
                //char chSum[10];
                //sprintf(chSum, "%d %d ", sum, checkSum);
                //std::cout << "-----校验失败:" << chSum << std::endl;
                *bComplete = false;
            }
        }        
     
    }else{
        *bComplete = false;
        *bStartLine = false;
    }

};

void GBlidarRead::AddOneLineData(uint8_t* data, int size)
{
    //static RadarData radarData;
	static uint8_t dataTemp[DATACOUNT * 2 + 6] = {'\0'};//缓存一条语句
	static int tempCount = 0;
	//打印输出
    //std::cout <<"------------------------------"<< std::endl;

    for(int i = 0; i < size; i++)
    {
        char str[3] = {'\0'};
        char buf = (char)(data[i]);
        //memcpy(&buf, &(data[i]), 1);
        Hex2Str(&buf,str,1);
        //std::cout << str << " ";
    }
    //std::cout << std::endl;

	bool bComplete = false;
	bool bStart = false;
	CheckLineBeComplete(data, size, &bComplete, &bStart);
	if(bComplete)
	{
		if(tempCount == (DATACOUNT * 2 + 6))
		{
            bool bComplete2 = false;
	        bool bStart2 = false;
            CheckLineBeComplete(dataTemp, tempCount, &bComplete2, &bStart2);//测试校验上一帧不完整数据拼接起来的数据是否正确

			//获取index
			int index = (int)(dataTemp[1]);
			if(index >= 0 && index<= 29) 
			{
				//更新转速
				radarData.speed = (int)(dataTemp[3] << 8) + (int)(dataTemp[2]);
				for(int i = 0; i < DATACOUNT; i++)
				{
					int iD = (int)(dataTemp[4 + 2*i + 1] << 8) + (int)(dataTemp[4 + 2*i]);
                    if(iD == 32768)
                        iD = -1;
					radarData.data[index][i] = iD;//赋值
				}
                
                tempCount = 0;
                memset(dataTemp, '\0', 54);
		    }
        }
		
		if(bStart)
		{	
            //下一组数据来了，请在这里处理解析好的一组数据，
            //std::cout << "-----请在这里处理解析好的一组数据"<< std::endl;
            Update(radarData);

            //这里清除这一组数据
			radarData.ClearData();
		}
		//获取index
		int index2 = (int)(data[1]);
		if(index2 >= 0 && index2 <= 29 && size == (DATACOUNT * 2 + 6))//54 = 6 + 24 * 2 
		{
			//更新转速
			radarData.speed = (int)(data[3] << 8) + (int)(data[2]);
			for(int i = 0; i < DATACOUNT; i++)
			{
				int iD = (int)(data[4 + 2*i + 1] << 8) + (int)(data[4 + 2*i]);
                if(iD == 32768)
                    iD = -1;
				radarData.data[index2][i] = iD;//赋值
			}
		}
    }
	else
	{
		if(tempCount + size <= (DATACOUNT * 2 + 6))
		{
			memcpy(dataTemp + tempCount, data, size);
			tempCount += size;
		}
	}
}


void GBlidarRead::Hex2Str( const char *sSrc,  char *sDest, int nSrcLen )
{
    //std::cout <<"----------start--------"<<std::endl;//换行
    int  i;
    char szTmp[3];
 
    for( i = 0; i < nSrcLen; i++ )
    {
        //std::cout <<"----------start--------0"<<std::endl;//换行
        sprintf( szTmp, "%02X", (unsigned char) sSrc[i] );
        //std::cout <<"----------start--------1"<<std::endl;//换行
        memcpy( &sDest[i * 2], szTmp, 2 );
        //std::cout <<"----------start--------2"<<std::endl;//换行
    }
    return ;
}

void GBlidarRead::Update(const RadarData& data)
{
    static int i =0;
    std::cout << "data update: " << i++<< std::endl;
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = ros::Time::now();
    scan_msg.header.frame_id = frameId;

    scan_msg.angle_min =  -M_PI;
    scan_msg.angle_max =  M_PI;

    scan_msg.angle_increment = 12.0/DATACOUNT*M_PI/180;

    scan_msg.scan_time = 1.0/(216000/30/DATACOUNT);
    scan_msg.time_increment = 1.0/(216000);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = 15;//8.0;

    scan_msg.intensities.resize(30* DATACOUNT);
    scan_msg.ranges.resize(30* DATACOUNT);

    //float degree = 0;
    for(int i = 0;i<30;i++){
        //std::cout << "-degree:" << degree << ",distance(m):"<< data.data[i][0]*0.001 << std::endl;
        for(int j=0 ;j< DATACOUNT;j++){
            scan_msg.ranges[i*30+j] = data.data[i][j]*0.001;//m
            scan_msg.intensities[i*30+j] = 0.0;
            //degree = degree + 0.333;
        }
    }

    lidarPub_.publish(scan_msg);
}


void GBlidarRead::work()
{
    std::cout << "GBlidarRead::ReadData start.." << std::endl;
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    //ros::Rate loop_rate(20);//睡眠10毫秒
    bool bLastFA = false;
    uint8_t tempData[200];
    int tempPos = 0;
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        //std::cout << "GBlidarRead::ReadData 2..|| "<<sp.available() << std::endl;
        int n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024 * 1024];
            //读出数据
            //std::cout << "GBlidarRead::ReadData 3.." << std::endl;
            n = sp.read(buffer, n);
            std::cout << "GBlidarRead::ReadData buffer length:" << n<< std::endl;
            for(int i = 0; i < n; i++)
            {
                if(buffer[i] == 0xFA)
                {
                    if(bLastFA)
                    {
                        AddOneLineData(tempData, tempPos);//分割一条数据
                    }
                    bLastFA = true;
                    tempPos = 0;
                    tempData[tempPos++] = buffer[i];
                }
                else if(bLastFA)
                {
                    tempData[tempPos++] = buffer[i];
                }
            }
        }
        //loop_rate.sleep();
        ros::spinOnce();

    }
    std::cout << "GBlidarRead::ReadData end.." << std::endl;
}


void GBlidarRead::work2()
{
    std::cout << "GBlidarRead::ReadData start.." << std::endl;
    ros::Rate loop_rate(20);//睡眠10毫秒
    bool bLastFA = false;
    uint8_t tempData[200];
    int oneBlockCount = -1;
    bool isCheckNewBlock = false;
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        //std::cout << "GBlidarRead::ReadData 2..|| "<<sp.available() << std::endl;
        int n = sp.available();
        std::cout << "GBlidarRead::ReadData 2..|| "<< n << std::endl;
        if(n!=0)
        {
            uint8_t buffer[1024 * 1024];
            //读出数据
            //std::cout << "GBlidarRead::ReadData 3.." << std::endl;
            n = sp.read(buffer, n);
            std::cout << "GBlidarRead::ReadData buffer length:" << n<< std::endl;
            bool isUpdateOneBlock = false;
            //bool isCheckNewBlock = true;
            for(int i = 0; i < n; i++)
            {
                if(buffer[i] == 0xFA)
                {
                    isCheckNewBlock = true;
                    memset(tempData, 0, DATACOUNT * 2 + 6);
                    oneBlockCount = 0;
                    tempData[oneBlockCount++] = buffer[i];
                }
                else
                {
                    if(isCheckNewBlock)
                        tempData[oneBlockCount++] = buffer[i];
                }

                if(isCheckNewBlock && oneBlockCount == DATACOUNT * 2 + 6 )
                {
                    std::cout<<"oneBlockCount calc: "<<oneBlockCount<<std::endl;
                    isCheckNewBlock = false;
                    processOneBlock(tempData, oneBlockCount);
                    oneBlockCount = -1;
                    memset(tempData, 0, DATACOUNT * 2 + 6);
                }
            }
        }
        loop_rate.sleep();
        ros::spinOnce();

    }
    std::cout << "GBlidarRead::ReadData end.." << std::endl;
}

void GBlidarRead::processOneBlock(uint8_t* data, int size)
{
    //static RadarData radarData;
    /*//打印输出
    //std::cout <<"------------------------------"<< std::endl;

    for(int i = 0; i < size; i++)
    {
        char str[3] = {'\0'};
        char buf = (char)(data[i]);
        //memcpy(&buf, &(data[i]), 1);
        Hex2Str(&buf,str,1);
        //std::cout << str << " ";
    }
    //std::cout << std::endl;
*/
    static int lastBlockIndex = -1;
    int checkSum = 0;
    bool isUpdateOneBlock = false;
    bool isNewScan = false;
    if(size == (DATACOUNT * 2 + 6) && data[0] == 0xFA)
    {
        for(int i = 0;i<size -2;i++){//最后2位是校验位
            checkSum += (int)(data[i]);
        }
        int checkSum = (int)(data[size -1]<<8) + (int)(data[size -2]);
        if((int)checkSum == checkSum)
        {
            isUpdateOneBlock = true;
        }
        else
        {
            isUpdateOneBlock = false;
        }
        std::cout<<"count OK! isUpdateOneBlock checkSum: "<<isUpdateOneBlock<<std::endl;
    }
    else
    {
        isUpdateOneBlock = false;
    }
    std::cout<<"run UpdateOneBlock "<<std::endl;

    if(isUpdateOneBlock)
    {
        int curBlockIndex = (int)(data[1]);
        std::cout<<" UpdateOneBlock +||+  "<<curBlockIndex<<std::endl;
        if(curBlockIndex < lastBlockIndex)
        {
            std::cout<<" Update"<<std::endl;
            Update(radarData);
            //radarData.ClearData();
        }

        if(curBlockIndex >= 0 && curBlockIndex<= 29)
        {
            //更新转速
            radarData.speed = (int)(data[3] << 8) + (int)(data[2]);
            for(int i = 0; i < DATACOUNT; i++)
            {
                int distanceMm = (int)(data[4 + 2*i + 1] << 8) + (int)(data[4 + 2*i]);
                if(distanceMm == 32768)
                    distanceMm = -1;
                radarData.data[curBlockIndex][i] = distanceMm;//赋值
            }
        }

        if(curBlockIndex == 29)
        {
            std::cout<<" Update"<<std::endl;
            Update(radarData);
            //radarData.ClearData();
        }

        lastBlockIndex = curBlockIndex;
    }
}

