//###########################################################################
//--------------------------------------------------------------------------
// FILE:Constant.h
// DESCRIPTION:
//         including those constants used in the 1140V SVG system
//--------------------------------------------------------------------------
// Author: Chao  Zhang
// Date: Dec  16, 2015
// Target System: DSP F28335 +　FPGA
// Version: V1.0
//--------------------------------------------------------------------------
//###########################################################################
#ifndef SVG_EEPROM_H
#define SVG_EEPROM_H

#include "SVG_Variable.h"
#include "SVG_Constant.h"
#include "SVG_ModBus.h"

//通信相关
void Read_data(int ad,int n); //读取数据，起始地址ad,数据量n
void Para_set1(void);
void Para_set2(void);
void Parameter_Setting(void);       //参数配置函数
void Display(void);
void Display_CON3S2S(float, float, float);//计算电网电压显示时用到的3S/2S变换
void Display_Current(void);         //电流波形显示
void Display_Para(void);
void Display_Send_Ready( int, int );
void Para_set_end(int ad, int n);

const unsigned Version_No = 0x0001;///程序版本号,占用一个字节
int Restore_Factory_Flag = 0;
//**************EEPROM操作时产生的各种错误标记**************
//unsigned RD_EEPROM_Fail_Sign = 0;//读EEPROM错标志
//unsigned WR_EEPROM_Fail_Sign = 0;//读EEPROM错标志
unsigned EEPROM_No_WRed_Sign = 0;//EEPROM未被写标志
unsigned Version_Erro_Sign = 0; //版本错标志

int RD_EEPROM_Data = 0;   //临时存储从EEPROM中读出来的数据
int MsgBuffer_Send[15] = {0};	//将参数值写入EEPROM时被用到,一次最多能写7个数
int MsgBuffer_Receive[15] = {0};	//存放从EEPROM中接收到的有效数据,最多一次能读7个数

void RD_DATA_PROCESS(unsigned Byte_Sum);
void WR_DATA_PROCESS(unsigned Word_Sum);
void RD_EEPROM(unsigned Byte_Sum,unsigned Offset_Addr,unsigned Page_Addr);
void WR_EEPROM(unsigned Byte_Sum,unsigned Offset_Addr,unsigned Page_Addr);
void Restore_Factory_Set(void);
void Ctrl_Params_Initial(void);

//通用函数
void delay_t1(int); 	 //100大约是8个us, 频率150M

//------------------------------eeprom-------------------------------------------//

void RD_EEPROM(unsigned Byte_Sum,unsigned Offet_Addr,unsigned Page_Addr)
{	//最多读7个字!!!
	unsigned i=0;
	unsigned I2C_com=0;
	if(Byte_Sum>14)
	{
		Byte_Sum = 14;
		ERROR_flag |= RD_EEPROM_OV7_ERROR;
	}
loop1:	I2caRegs.I2CSAR = 0x50+Page_Addr;
	I2caRegs.I2CCNT = 1;
	I2caRegs.I2CDXR = Offet_Addr;
	I2caRegs.I2CMDR.all = 0x6E20;

	I2C_com=0;
	while(I2caRegs.I2CMDR.bit.STP == 1)
	{
		I2C_com++;
		for(i=0;i<10;i++){;}
		if(I2C_com>10000)
		{
			I2C_com=0;
			goto loop1;
		}
	}
	I2caRegs.I2CMDR.bit.STP = 1; //清总线忙状态
	I2caRegs.I2CCNT = Byte_Sum;

	I2caRegs.I2CMDR.all = 0x6C20;
	I2C_com=0;
	while(I2caRegs.I2CMDR.bit.STP == 1)
	{
	 	I2C_com++;
		for(i=0;i<10;i++){;}
		if(I2C_com>10000)
		{
			I2C_com=0;
			goto loop1;
		}
	}
	I2caRegs.I2CMDR.bit.STP = 1;
	for(i=0; i<Byte_Sum; i++)
	{
		MsgBuffer_Receive[i] = I2caRegs.I2CDRR;
	}

}

void WR_EEPROM(unsigned Byte_Sum,unsigned Offet_Addr,unsigned Page_Addr)
{	//最多处理7个字!!!!
	unsigned int  i=0;
	unsigned I2C_com=0;

	if(Byte_Sum>14)
	{
		Byte_Sum = 14;
		ERROR_flag |= WR_EEPROM_OV7_ERROR;
	}
loop2: I2caRegs.I2CSAR = 0x50+Page_Addr;// EEPROM某一页地址（由命令吧璞负�010A2和页P1P0组成）
	I2caRegs.I2CCNT = Byte_Sum+1;// 发送的字节Byte_Sum + 2个诘刂罚ㄊ导视凶纸诿看最多发送14个剑�
	I2caRegs.I2CDXR = Offet_Addr; //EEPROM页内偏移地址首地址
	//依次向I2CDXR中写入要发送的字节数（FIFO）
	for(i=0; i<Byte_Sum; i++)
	{
	  I2caRegs.I2CDXR = MsgBuffer_Send[i];
	}
	//启动发(自由方式不受断点影响；置开始模式；停模式；主动模式；发送；I2C模槭鼓�
	I2caRegs.I2CMDR.all = 0x6E20;
	I2C_com=0;
	while(I2caRegs.I2CMDR.bit.STP == 1)
	{
		I2C_com++;
		for(i=0;i<10;i++){;}
		if(I2C_com>10000)
		{
			I2C_com=0;
			goto loop2;
		}
	}
	I2caRegs.I2CMDR.bit.STP = 1;
	for(i=0;i<30000;i++){;}//AT24C08每次写完必须等待毫秒
}

void Restore_Factory_Set()
{
	//恢复出厂设置
	int i=0;
	MsgBuffer_Send[i++] = 0x55AA;	//EEPROM非空校验字, 0x00
	MsgBuffer_Send[i++] = Version_No;	//程序版本号,存入EEPROM, 0x02
	WR_DATA_PROCESS(i);		//处理2个Words,处理之后变为4个字节,最多处理7个字!!!!
//	向EEPROM第0x4页页内偏移地址为0x00的空间中写入4个字节
	WR_EEPROM((i<<1),0x00,0x4);	//写入4个字节,偏移地址为0x00

//	注意: 不能一次从EEPROM中读写超过7个字 !!!
	i = 0;
	MsgBuffer_Send[i++] = UDCREF_initial;
 	MsgBuffer_Send[i++] = UDC_Kp_initial;
 	MsgBuffer_Send[i++] = UDC_Ki_initial;
 	MsgBuffer_Send[i++] = Ud_Kp_initial;
 	MsgBuffer_Send[i++] = Ud_Ki_initial;
 	MsgBuffer_Send[i++] = Kp_P2P_Neg_initial;
 	MsgBuffer_Send[i++] = PLL_Swtich_initial;
	WR_DATA_PROCESS(i);
	WR_EEPROM((i<<1),0x50,0x4);//写入14个字节,偏移地址为0x50

	i = 0;
	MsgBuffer_Send[i++] = Kp_in_initial;
	MsgBuffer_Send[i++] = UDCMAX_initial;
 	MsgBuffer_Send[i++] = UDCMIN_initial;
 	MsgBuffer_Send[i++] = Q_REF_initial;
 	MsgBuffer_Send[i++] = Ipcc_Amp_initial;
 	MsgBuffer_Send[i++] = Kp_Rpt_initial;
 	MsgBuffer_Send[i++] = THETA_COM_initial;
 	WR_DATA_PROCESS(i);
 	WR_EEPROM((i<<1),0x70,0x4);//写入14个字节,偏移地址为0x70
}

void Ctrl_Params_Initial()  //12个可调节参数
{
	int i=0;
//	读出EEPROM 0x4页, 页内偏移地址为0x50的连续6个数据, 共12字节
	RD_EEPROM(14,0x50,0x4);	//读0x4页偏移地址0x50
	RD_DATA_PROCESS(14);
//	初始化各参数
	i=0;
	Ctrl_Para.UDCREF = MsgBuffer_Receive[i++]; //直流侧电压给定值
	Ctrl_Para.UDC_Kp = MsgBuffer_Receive[i++]; //电压环Kp参数
	Ctrl_Para.UDC_Ki = MsgBuffer_Receive[i++]; //电压环Ki
	Ctrl_Para.Ud_Kp = MsgBuffer_Receive[i++];  //电流环Kp参数
	Ctrl_Para.Ud_Ki = MsgBuffer_Receive[i++];  //电流环Ki
	Ctrl_Para.Kp_P2P_Neg = MsgBuffer_Receive[i++];  //相间平衡Kp参数
	PLL_Switch = MsgBuffer_Receive[i++];	   //锁相环选择器

	i=0;
	RD_EEPROM(14,0x70,0x4);	//读0x4页偏移地址0x70
	RD_DATA_PROCESS(14);
	Ctrl_Para.Kp_in = MsgBuffer_Receive[i++];  //相内平衡Kp参数
	Ctrl_Para.UDCMAX = MsgBuffer_Receive[i++]; //直流母线电压最高限幅值
	Ctrl_Para.UDCMIN = MsgBuffer_Receive[i++]; //直流母线电压最低限幅值
	Q_REF = MsgBuffer_Receive[i++]; 		   //无功电流给定值
	Ipcc_Amp = MsgBuffer_Receive[i++];  	   //并网电流过流保护阈值,扩大100倍存储
	Ctrl_Para.Rpt_Kp = MsgBuffer_Receive[i++]; //重复控制Kp参数
	THETA_COM_Usq = MsgBuffer_Receive[i++];	   //调制波补偿相角
}

//读触摸屏参数
void Read_data(int ad,int n)
{
	 int n_data;
loop: com1=0;
	GpioDataRegs.GPBSET.bit.GPIO61=1;		//置发送模式,引至MAX485的/RE端
	ScicRegs.SCITXBUF=0x1B;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}    //0：SCITXBUF满； 1：SCITXBUF准备接收下一字符
	ScicRegs.SCITXBUF=0x52;		//发送0x1B52
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x00;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=ad;		//发送起始地址：0x00ad
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x00;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=n;		//发送数据个数：0x00n
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0xD3;		//发送0xD300
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}

	GpioDataRegs.GPBCLEAR.bit.GPIO61=1;		//置为接收模式,引至MAX485的/RE端
	ScicRegs.SCICTL1.bit.SWRESET=0;			//软件复位SCI, 清标志位
	ScicRegs.SCICTL1.bit.SWRESET=1; 	//重启SCI
	while(ScicRegs.SCIRXST.bit.RXRDY==0)  //1:数据读操作就绪,CPU可以从SCIRXBUF中读取新字符; 0:SCIRXBUF中没有新字符
	{
		com1++;
		delay_t1(100);
		if(com1>2000)  //超时重发
		{
			goto loop;
		}
	}
	com1=0;
    ReceivedChar = ScicRegs.SCIRXBUF.all;
	ReceivedChar&=0x00FF;				//接受？1B
	if(ReceivedChar!=0x001B)
    goto loop;

    while(ScicRegs.SCIRXST.bit.RXRDY==0)
   	{
		com1++;
		delay_t1(100);
		if(com1>1000)
		{
			goto loop;
		}
	}
    ReceivedChar = ScicRegs.SCIRXBUF.all;
    ReceivedChar&=0x00FF;				//接受？41
	if(ReceivedChar!=0x0041)
	goto loop;
	for(n_data=0;n_data<(2*n);n_data++)
	{
		com1=0;
		while(ScicRegs.SCIRXST.bit.RXRDY==0)
		{
			com1++;
			delay_t1(100);
			if(com1>1000)
			{
				goto loop;
			}
		}
    	ReceivedChar = ScicRegs.SCIRXBUF.all;
    	ReceivedChar&=0x00FF;
		Com_data[n_data]=ReceivedChar;
	}
}
void Display(void)
{
	int i=0;
	GpioDataRegs.GPBSET.bit.GPIO61=1;		//置发送模式,引至MAX485的DE控制端,若用232通信则不必考虑此引脚
//	 接收数据个数（Words个数 = 1/2 * Byte个数）
//	 动态显示数据段1：高位在前发送
	display_data_one[i++]=(int)(I_q*100)>>8;  //扩大100倍显示  //201
	display_data_one[i++]=(int)(I_q*100)%256;
	display_data_one[i++]=(int)(I_d*100)>>8;  //扩大100倍显示  //202
	display_data_one[i++]=(int)(I_d*100)%256;
	display_data_one[i++]=(int)(Idref*100)>>8;  //扩大100倍显示  //203
 	display_data_one[i++]=(int)(Idref*100)%256;

 	Display_Send_Ready(201,(i>>1));//准备发送显示数据；触摸屏起始地址201,连续显示i/2个数据
	for(com=0;com<i;com++)
	{
		ScicRegs.SCITXBUF=display_data_one[com];
		while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	}
	ScicRegs.SCITXBUF=0x8D;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}

//动态显示数据段2：高位在前发送,三相电网电压,SVG并网电流
   	i = 0;
	display_data_two[i++]=(int)(U_ADRF)>>8;
	display_data_two[i++]=(int)(U_ADRF)%256;  //301
	display_data_two[i++]=(int)(U_BDRF)>>8;
	display_data_two[i++]=(int)(U_BDRF)%256;  //302
	display_data_two[i++]=(int)(U_CDRF)>>8;
	display_data_two[i++]=(int)(U_CDRF)%256;   //303
	display_data_two[i++]=(int)(IASVG*100)>>8;
	display_data_two[i++]=(int)(IASVG*100)%256;    //304
	display_data_two[i++]=(int)(IBSVG*100)>>8;
	display_data_two[i++]=(int)(IBSVG*100)%256;   //305
	display_data_two[i++]=(int)(ICSVG*100)>>8;
	display_data_two[i++]=(int)(ICSVG*100)%256;   //306
	display_data_two[i++]=(int)(IA_Load*100)>>8;
	display_data_two[i++]=(int)(IA_Load*100)%256;    //307
	display_data_two[i++]=(int)(IB_Load*100)>>8;
	display_data_two[i++]=(int)(IB_Load*100)%256;   //308
	display_data_two[i++]=(int)(IC_Load*100)>>8;
	display_data_two[i++]=(int)(IC_Load*100)%256;   //309

	Display_Send_Ready(301,(i>>1));//准备发送显示数据;触摸屏起始地址301,连续显示i/2个数据
	for(com=0;com<i;com++)
	{
		ScicRegs.SCITXBUF=display_data_two[com];//此处用数组的方式，编程简单，但是如何使用，将数组中的数和另外一个数对应还是问题
		while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	}	                                   //以上为向触摸屏写所有8位字节，即实际要写的内容。高字节在前低字节在后
	ScicRegs.SCITXBUF=0x8D;
	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}//以上为写触摸屏结束命令

//动态显示数据段3：高位在前发送,A相各模块母线电压
	i = 0;
	display_data_two[i++]=(int)(UDC_AR1)>>8;  //501
	display_data_two[i++]=(int)(UDC_AR1)%256;
	display_data_two[i++]=(int)(UDC_AR2)>>8;  //502
	display_data_two[i++]=(int)(UDC_AR2)%256;
	display_data_two[i++]=(int)(UDC_AR3)>>8;  //503
	display_data_two[i++]=(int)(UDC_AR3)%256;

	Display_Send_Ready(501,(i>>1));//准备发送显示数据；触摸屏起始地址501,连续显示i/2个数据
	for(com=0;com<i;com++)
	{
		ScicRegs.SCITXBUF=display_data_two[com];//此处用数组的方式，编程简单，但是如何使用，将数组中的数和另外一个数对应还是问题
		while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	}	                                   //以上为向触摸屏写所有8位字节，即实际要写的内容。高字节在前低字节在后
	ScicRegs.SCITXBUF=0x8D;
	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}//以上为写触摸屏结束命令

//动态显示数据段4：高位在前发送,B相各模块母线电压
	i = 0;
	display_data_two[i++]=(int)(UDC_BR1)>>8;  //513
	display_data_two[i++]=(int)(UDC_BR1)%256;
	display_data_two[i++]=(int)(UDC_BR2)>>8;  //514
	display_data_two[i++]=(int)(UDC_BR2)%256;
	display_data_two[i++]=(int)(UDC_BR3)>>8;  //515
	display_data_two[i++]=(int)(UDC_BR3)%256;

	Display_Send_Ready(513,(i>>1));//准备发送显示数据；触摸屏起始地址513,连续显示i/2个数据
	for(com=0;com<i;com++)
	{
		ScicRegs.SCITXBUF=display_data_two[com];//此处用数组的方式，编程简单，但是如何使用，将数组中的数和另外一个数对应还是问题
		while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	}	                                   //以上为向触摸屏写所有8位字节，即实际要写的内容。高字节在前低字节在后
	ScicRegs.SCITXBUF=0x8D;
	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}//以上为写触摸屏结束命令

//动态显示数据段5：高位在前发送,C相各模块母线电压
	i = 0;
	display_data_two[i++]=(int)(UDC_CR1)>>8;  //525
	display_data_two[i++]=(int)(UDC_CR1)%256;
	display_data_two[i++]=(int)(UDC_CR2)>>8;  //526
	display_data_two[i++]=(int)(UDC_CR2)%256;
	display_data_two[i++]=(int)(UDC_CR3)>>8;  //527
	display_data_two[i++]=(int)(UDC_CR3)%256;

	Display_Send_Ready(525,(i>>1));//准备发送显示数据；触摸屏起始地址525,连续显示i/2个数据
	for(com=0;com<i;com++)
	{
		ScicRegs.SCITXBUF=display_data_two[com];
		while(ScicRegs.SCICTL2.bit.TXRDY==0){;}   //0：SCITXBUF满； 1：SCITXBUF准备接收下一字符
	}	                                   //以上为向触摸屏写所有8位字节，即实际要写的内容。高字节在前低字节在后
	ScicRegs.SCITXBUF=0x8D;
	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}//以上为写触摸屏结束命令
}

//触摸屏参数显示
void Display_Para(void)
{
	int i=0;
	GpioDataRegs.GPBSET.bit.GPIO61=1;//置发送模式,引至MAX485的DE控制端,若用232通信则不必考虑此引脚

//----------------------显示从eeprom中读出来的初始参数值--------------------------------
//静态显示数据段1：高位在前发送
	Para1_Intdata[i++] = ((int)Ctrl_Para.UDC_Kp>>8);	//高8位 //101
	Para1_Intdata[i++] = ((int)Ctrl_Para.UDC_Kp%256);   //低8位
	Para1_Intdata[i++] = ((int)Ctrl_Para.UDC_Ki>>8);	//102
	Para1_Intdata[i++] = ((int)Ctrl_Para.UDC_Ki%256);
	Para1_Intdata[i++] = ((int)Ctrl_Para.Ud_Kp>>8);	//103
	Para1_Intdata[i++] = ((int)Ctrl_Para.Ud_Kp%256);
	Para1_Intdata[i++] = ((int)Ctrl_Para.Ud_Ki>>8);	//104
	Para1_Intdata[i++] = ((int)Ctrl_Para.Ud_Ki%256);
	Para1_Intdata[i++] = ((int)Ctrl_Para.Kp_P2P_Neg>>8);	//105
	Para1_Intdata[i++] = ((int)Ctrl_Para.Kp_P2P_Neg%256);
	Para1_Intdata[i++] = ((int)Ctrl_Para.Kp_in>>8);	//106
	Para1_Intdata[i++] = ((int)Ctrl_Para.Kp_in%256);
	Para1_Intdata[i++] = 0x00;	//107
	Para1_Intdata[i++] = (int)U_FtoI;
	Para1_Intdata[i++] = 0x00;	//108
	Para1_Intdata[i++] = (int)Ctrl_Para.Cloop;
	Para1_Intdata[i++] = ((int)Q_REF>>8);	//109
	Para1_Intdata[i++] = ((int)Q_REF%256);

	Display_Send_Ready(101,(i>>1));//准备发送显示数据；触摸屏起始地址101,连续显示i/2个数据
	for(com=0;com<i;com++)
	{
		ScicRegs.SCITXBUF=Para1_Intdata[com];//此处用数组的方式，编程简单，但是如何使用，将数组中的数和另外一个数对应还是问题
		while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	}
	ScicRegs.SCITXBUF=0x8D;
	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
}

void Display_Current(void)
{
	ScicRegs.SCITXBUF=0x1B;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x57;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x01;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x2C;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x00;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x98;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x00;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x03;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x00;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x96;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	for(com=0;com<150;com++)
	{
	ScicRegs.SCITXBUF=Current_IA[com];//此处用数组的方式，编程简单，但是如何使用，将数组中的
	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	}
	ScicRegs.SCITXBUF=0x8D;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}

}

void Para_set1(void)
{
	int i=0;
	Read_data(0x96,1);  //从地址ad开始连续读n个数据

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//151
	if(EEPROM_Compare != (int)Q_REF)
	{
		Q_REF = EEPROM_Compare;
		MsgBuffer_Send[0] = (int)Q_REF;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x28,0x4);
	}

    Para_set_end(0xAC, 0x02);//用于发送触摸屏参数读完毕命令？？？
}
void Para_set2(void)
{
	int i=0;
	Read_data(101,9);

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++]; //101
	if(EEPROM_Compare != (int)Ctrl_Para.UDC_Kp)//判断是否将数据写入EEPROM
	{
		Ctrl_Para.UDC_Kp=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x18,0x4);
	}

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//102
	if(EEPROM_Compare != (int)Ctrl_Para.UDC_Ki)//判断是否将数据写入EEPROM
	{
		Ctrl_Para.UDC_Ki=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x1A,0x4);
	}

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//103
	if(EEPROM_Compare != (int)Ctrl_Para.Ud_Kp)//判断是否将数据写入EEPROM
	{
		Ctrl_Para.Ud_Kp=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x1C,0x4);
	}

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//104
	if(EEPROM_Compare != (int)Ctrl_Para.Ud_Ki)//判断是否将数据写入EEPROM
	{
		Ctrl_Para.Ud_Ki=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x1E,0x4);
	}

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//105
	if(EEPROM_Compare != (int)Ctrl_Para.Kp_P2P_Neg)//判断是否将数据写入EEPROM
	{
		Ctrl_Para.Kp_P2P_Neg=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x20,0x4);
	}

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//106
	if(EEPROM_Compare != (int)Ctrl_Para.Kp_in)//判断是否将数据写入EEPROM
	{
		Ctrl_Para.Kp_in=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x22,0x4);
	}

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//107
	if(EEPROM_Compare != (int)U_FtoI)//判断是否将数据写入EEPROM
	{
		U_FtoI=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x24,0x4);
	}

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//108
	if(EEPROM_Compare != (int)Ctrl_Para.Cloop)//判断是否将数据写入EEPROM
	{
		Ctrl_Para.Cloop=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x26,0x4);
	}

	EEPROM_Compare=(Com_data[i++]<<8)|Com_data[i++];	//109
	if(EEPROM_Compare != (int)Q_REF)//判断是否将数据写入EEPROM
	{
		Q_REF=EEPROM_Compare;
		MsgBuffer_Send[0] = EEPROM_Compare;
		WR_DATA_PROCESS(1);
		WR_EEPROM(2,0x28,0x4);
	}

//	Para_set_end(0xAC, 0x04);
}

//此函数的目的是每次参数设置完成之后（即按确定键之后）使对应的"等待参数设置"显示灯变亮，
//提示用户参数设置完成。此函数实质就是写触摸屏
void Para_set_end(int ad, int n)//变量n表示要写入的内容
{
	GpioDataRegs.GPBSET.bit.GPIO61=1;		//置发送模式
	ScicRegs.SCITXBUF=0x1B;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x57;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}//写触摸屏命令
	ScicRegs.SCITXBUF=0x00;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=ad;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}//触摸屏首地址
	ScicRegs.SCITXBUF=0x00;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x01;
	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}//写一个16位的字
	ScicRegs.SCITXBUF=0x00;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=n;
	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}//要写入的内容，高字节在前，低字节在后
	ScicRegs.SCITXBUF=0x8D;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;} //结束写操作命令
}

void Display_Send_Ready(int ad, int n)
{
	int ad_HighByte = 0;
	int ad_LowByte = 0;
	int n_HighByte = 0;
	int n_LowByte = 0;
	if(ad > 255)
	{
		ad_LowByte = (ad%256);
		ad_HighByte = (ad>>8);
	}
	else
	{
		ad_LowByte = ad;
		ad_HighByte = 0x00;
	}
	if(n > 255)
	{
		n_LowByte = (n%256);
		n_HighByte = (n>>8);
	}
	else
	{
		n_LowByte = n;
		n_HighByte = 0x00;
	}

	ScicRegs.SCITXBUF=0x1B;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=0x57;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=ad_HighByte;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=ad_LowByte;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=n_HighByte;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}
	ScicRegs.SCITXBUF=n_LowByte;
   	while(ScicRegs.SCICTL2.bit.TXRDY==0){;}

}


void RD_DATA_PROCESS(unsigned Byte_Sum)
{
 	unsigned int  i = 0;
 	unsigned int  j = 0;
	unsigned int  High_byte;
	unsigned int  Low_byte;
	unsigned int Data_Buffer[16] = {0};

	for(i=0; i<(Byte_Sum*2); i++)
	{
		j++;
		Low_byte = MsgBuffer_Receive[i];
		High_byte = MsgBuffer_Receive[++i];
		High_byte = High_byte<<8;
		Data_Buffer[i-j] =  High_byte | Low_byte;
	}
	//两个连续字喜⒊梢祸种匦伦叭隡sgBuffer_Receive[]
	for(i=0; i<Byte_Sum/2; i++)
	{
		MsgBuffer_Receive[i] = Data_Buffer[i];
	}
}


void WR_DATA_PROCESS(unsigned Word_Sum)
{
	unsigned int  HIGH_RESVD = 0xff00;
	unsigned int  LOW_RESVD  = 0x00FF;
	unsigned int  High_byte;
	unsigned int  Low_byte;
	unsigned int  i = 0;
	unsigned int  j = 0;
	unsigned int  Data_Process_Buffer = 0;
	unsigned int Dada_Buffer[15] = {0};
	for(i=0; i<Word_Sum; i++)
	{
		Dada_Buffer[i] = MsgBuffer_Send[i];
	}
	//一个字拆成2个字节存入MsgBuffer_Send[]
	for(i=0; i<Word_Sum*2; i++)
	{
		Data_Process_Buffer = Dada_Buffer[i-j];
		Low_byte = (Data_Process_Buffer & LOW_RESVD);
		MsgBuffer_Send[i] = Low_byte;
		High_byte = (Data_Process_Buffer & HIGH_RESVD);
		High_byte = High_byte>>8;
		MsgBuffer_Send[++i] = High_byte;
		j++;
		//注意j的使用，为的是在处理Data_Buffer[]时不会遗漏数据
	}
}

/******************************通用函数*********************************/
void delay_t1(int a)
{
	int n=0;
  for(n=0;n<a;n++)
  {
  ;
  }
}


#endif  	// end of SVG_MODBUS_H definition
