
#ifndef __TSL1401
#define __TSL1401
#include "ebox.h"
#define TSL1401_DATA_LENGTH 128

//基础TSL1401驱动
class TSL1401
{
public:
	
	//构造函数，确定IO口
	TSL1401(Gpio* pinADC, Gpio* pinSI, Gpio* pinCLK);

	//初始化函数，初始化IO口和ADC
	void begin();

	//曝光函数，调节时钟间隔clk_time减小曝光时间，缺省值为1us,SI间隔delay_time，增加曝光时间，
	void capture(int delay_time);

	//读取数据函数
	u16* getData();
private:
	Gpio* pinADC;
	//void* delayMsFunction;
	Gpio* pinSI;
	Gpio* pinCLK;
	u16 data[TSL1401_DATA_LENGTH];

};

class TSL1401Auto:public TSL1401
{
public:
	TSL1401Auto(Gpio* pinADC, Gpio* pinSI, Gpio* pinCLK);
	int capture_time(int cru_time);
private:

};
#endif