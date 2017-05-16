//###########################################################################
//--------------------------------------------------------------------------
// FILE:  Main.c
// DESCRIPTION:
//         including those functions used in the 10KV SVG system
//--------------------------------------------------------------------------
// $Author: LI Shuzhen $
// $Date: September 04, 2016$
// $Target System: DSP F28335 + FPGA   高压SVG $
// $Version: v3.1 $
//--------------------------------------------------------------------------
//###########################################################################
#include "SVG_Variable.h"
#include "SVG_Constant.h"
#include "SVG_ModBus.h"
#include "SVG_EEPROM.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "math.h"

//初始化配置
void InitSysCtrl(void);     //系统时钟及外设时钟初始化
void InitGpio(void);        //用到的GPIO引脚初始化
void InitXintf(void); 
void InitECan(void);
//void InitSpi(void);

//通用函数
void delay_t(int); 	 //100大约是8个us, 频率150M

//通信相关
void InitSci();

//FLASH使能
void InitFlash(void);

//eeprom
void InitI2CGpio();
void InitI2C();

//控制相关
interrupt void cpu_timer0_isr(void);    //定时器0中断
interrupt void xint1_isr(void);
void InitInterrupts(void);				//CpuTimer0定时中断初始化
void InitCpuTimers0(void);
void UDC_PI(float Kp,float Ki);    	    //全局直流侧电压PI调节函数
void Current_PI1( float, float );       //有功电流环PI调节函数
void Current_PI2( float, float );       //无功电流环PI调节函数
//float Repetive_PI1( float Kp, float Ki );
//float Repetive_PI2( float Kp, float Ki );
void DataCheck(void);
void Read_Fault_State(void);
void Local_DC_Process(void);
void P2P_Balance_Neg_U(void);
//void P2P_Balance_Neg_I(void);
void P2P_Balance_Z(void);
void DA_Initialization(void);
void CON3S2S(float,float,float);		//三相静止到两相静止(3S/2S)坐标变换
void CON2S3S(float,float);				//两相静止到三相静止(2S/3S)坐标反变换
void Sin_Value(float);					//求正/余弦值
void CON2R2S_W(float Um,float Ut);      //两相旋转到两相静止(2R/2S)坐标反变换
void CON2S2R_W(float U_alpha,float U_beta); //两相静止到两相旋转(2S/2R)坐标变换
float ARCTAN(float,float);
void Main_DA_OUT(void);
void Phase_Lock(void);
float filter_2nd_dq(const float In, float* buffer);
float filter_Band_Stop(float In, float* buffer);
//float filter_2nd_70Hz(const float In, float* buffer);
//void DDACSM(const float I_in, const float wt_in, const float delt_in, float* buffer, float filter_2rd_buffer[][4], float* result);
//void DDACSM_2(const float I_alpha_in, const float I_belta_in, const float wt_in, const float delt_in);

//************************FLASH************************
#pragma CODE_SECTION(InitFlash, "ramfuncs");//必须添加这条语句，不然下载到FLASH无法运行
//#pragma CODE_SECTION(MemCopy, "ramfuncs"); //不能添加这条语句,因为MemCopy得放在praga之后
#pragma CODE_SECTION(delay_t, "ramfuncs");
#pragma CODE_SECTION(cpu_timer0_isr, "ramfuncs");
#pragma CODE_SECTION(xint1_isr, "ramfuncs");
#pragma CODE_SECTION(UDC_PI, "ramfuncs");
#pragma CODE_SECTION(Current_PI1,"ramfuncs");
#pragma CODE_SECTION(Current_PI2,"ramfuncs");
#pragma CODE_SECTION(DataCheck, "ramfuncs");
#pragma CODE_SECTION(Read_Fault_State, "ramfuncs");
#pragma CODE_SECTION(Local_DC_Process, "ramfuncs");
#pragma CODE_SECTION(P2P_Balance_Neg_U, "ramfuncs");
#pragma CODE_SECTION(P2P_Balance_Z, "ramfuncs");
#pragma CODE_SECTION(CON3S2S, "ramfuncs");
#pragma CODE_SECTION(CON2S3S, "ramfuncs");
#pragma CODE_SECTION(Sin_Value, "ramfuncs");
#pragma CODE_SECTION(CON2R2S_W, "ramfuncs");
#pragma CODE_SECTION(CON2S2R_W, "ramfuncs");
#pragma CODE_SECTION(ARCTAN, "ramfuncs");
#pragma CODE_SECTION(Main_DA_OUT, "ramfuncs");
#pragma CODE_SECTION(Phase_Lock, "ramfuncs");
#pragma CODE_SECTION(filter_2nd_dq, "ramfuncs");
#pragma CODE_SECTION(filter_Band_Stop, "ramfuncs");
//#pragma CODE_SECTION(filter_2nd_70Hz, "ramfuncs");
//#pragma CODE_SECTION(DDACSM, "ramfuncs");
//#pragma CODE_SECTION(DDACSM_2, "ramfuncs");

#pragma CODE_SECTION(Sci_Modbus_main, "ramfuncs");
#pragma CODE_SECTION(ModBusCRC16, "ramfuncs");
#pragma CODE_SECTION(ModBusCRC16_TBC, "ramfuncs");
#pragma CODE_SECTION(RD_Switch01_ServFun, "ramfuncs");
#pragma CODE_SECTION(RD_Reg03_ServFun, "ramfuncs");
#pragma CODE_SECTION(WE_S_Switch05_ServFun, "ramfuncs");
#pragma CODE_SECTION(WE_S_Reg06_ServFun, "ramfuncs");
#pragma CODE_SECTION(Scan_Switch_Addr_RD, "ramfuncs");
#pragma CODE_SECTION(Scan_Reg_Addr_RD, "ramfuncs");
#pragma CODE_SECTION(Scan_S_Switch_Addr_WE, "ramfuncs");
#pragma CODE_SECTION(Scan_S_Reg_Addr_WE, "ramfuncs");
#pragma CODE_SECTION(Sci_Tx_Modbus, "ramfuncs");

#pragma CODE_SECTION(RD_DATA_PROCESS, "ramfuncs");
#pragma CODE_SECTION(WR_DATA_PROCESS, "ramfuncs");
#pragma CODE_SECTION(RD_EEPROM, "ramfuncs");
#pragma CODE_SECTION(WR_EEPROM, "ramfuncs");
#pragma CODE_SECTION(Restore_Factory_Set, "ramfuncs");
#pragma CODE_SECTION(Ctrl_Params_Initial, "ramfuncs");

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
void MemCopy(Uint16 *SourceAddr, Uint16 *SourceEndAddr, Uint16 *DestAddr);
//MemCopy必须放在#progma CODE_SECTION();声明之后
//------------------------------end--------------------------------------

void main(void)
{	
	//系统时钟初始化
	InitSysCtrl();
	// Disable CPU interrupts and clear all CPU interrupt flags:
    DINT;

#ifdef FLASH
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	asm(" RPT #8 || NOP");
	InitFlash();
#endif

    InitGpio();		//初始化GPIO
	//InitSpi();
	InitXintf();
	InitI2CGpio();	//与EEROM通信用
    InitI2C();		//初始化I2C
    InitSci(); 
	InitInterrupts();

//----------------User Difined Initialization Function-------------------//
	Run_Stop_Flag = 0;  	//初始未启动并网
	Reset_Flag = 1;  		//先初始化
	CpuTimer0Count = 0;
	DA_Initialization();	//先建立起DA阈值电压
	delay_t(200);			//100大约是8us, DSP频率150M

//	PI控制器参数初始化
	Ctrl_Para.UDCMAX = 900;
	Ctrl_Para.UDCMIN = 600;		//N=12,10KV并网时,每个模块不控整流电压为650V
	Ctrl_Para.UDCREF = 800;
	Ctrl_Para.UDC_SLIP = 0.04;	//1s上升250V
	Ctrl_Para.UDC_Kp = 1;
	Ctrl_Para.UDC_Ki = 1;
	Ctrl_Para.Ud_Kp = 9;
	Ctrl_Para.Ud_Ki = 60;
	Ctrl_Para.Kp_P2P_Neg = 2;
	Ctrl_Para.Kp_P2P_Z = 5;
	Ctrl_Para.Kp_in = 2;
	U_FtoI = 1;			//电压前馈系数
	Ctrl_Para.In_Flag = 1;
	Ctrl_Para.P2P_Flag = 1;
	Ctrl_Para.Cloop = 1;  //2：多电平测试模式; 1：闭环并网模式; 0：开环并网，必须串联电阻
	Ctrl_Para.UDC_ORF = 70;  //开环并网UDC参考值
	Ctrl_Para.Rpt_Kp = 0.05;

//------------------------------EEPROM结束--------------------------------//
	RXDC;   //初始DSP与触摸屏通信处于接收状态
	StartCpuTimer0();
	while(1)
	{
//		检测是否系统复位
		if(Reset_Flag)
		{
			StopCpuTimer0();  //设置初始状态时,禁止DSP中断

			*( DRAM_data + State_Reset ) = 100;  //因为DA还没开始工作时Reset不成功,所以必须在此之前先初始化DA_OUT()
			*( DRAM_data + State_Reset ) = 100;  //确保复位成功
//			delay_t(200);  //此全局复位信号需放在PWM初始化之前。

			Run_Stop_Flag = 0;
			CpuTimer0Count = 0;	//中断计数器清零及电容平衡控制使能
			F_State.Flag = 1;
			D_Check.Data = 0;
			D_Check.Flag = 1;
			ERROR_flag = 0;
			ERROR_module = 0;
			Run_Flag = 0;
//-------------PLL初始化----------------
			integral_Omiga = 0;
			integral_Vd1 = 0;
			integral_Vd2 = 0;
//			Vq_P1_integ = 0;
//			theta_integ = 0;
//-------------PLL初始化结束-------------
//			DA_Initialization();	//先建立起DA阈值电压
//			delay_t(400);
			CommuError.A = 0;
			CommuError.B = 0;
			CommuError.C = 0;
			CommuError.Flag = 1;
			if(ERROR_flag == 0) //无故障才启动定时器
				StartCpuTimer0();
			Reset_Flag = 0;
		}

		if(D_Check.Flag)	//检查数据链路是否正常,有无节点失联的情况
		{
			StopCpuTimer0();
			DataCheck();
			D_Check.Flag = 0;
			StartCpuTimer0();
		}

		if(F_State.Flag)	//检查分控故障状态
		{
			StopCpuTimer0();
			Read_Fault_State();
			F_State.Flag = 0;
			StartCpuTimer0();
		}

		if(CommuError.Flag)
		{
			StopCpuTimer0();
			CommuError.A = *( DRAM_data +  Error_Check_A);
			CommuError.B = *( DRAM_data +  Error_Check_B);
			CommuError.C = *( DRAM_data +  Error_Check_C);
			delay_t(200);
			CommuError.Flag = 0;
			StartCpuTimer0();
		}
//--------------------------------触摸屏通信-------------------------------//
		if(Restore_Factory_Flag)
		{
			Restore_Factory_Set();	   //第一次出厂设置;以后要恢复出厂设置通过触摸屏操作
			Restore_Factory_Flag = 0;  //只恢复一次出厂设置
			Ctrl_Params_Initial();
		}
		Sci_Modbus_main();  //以ModBus协议进行SCI通信
//------------------------------触摸屏通信结束-------------------------------//

//-------------------------------while end-------------------------------//
	}
}

void Main_DA_OUT(void)
{
	int i = 0;
	//添加地址信息
	DA_Test[0] = Main_DA[0] + 0;      //数据 + TLV5630地址通道  A OC
	DA_Test[1] = Main_DA[1] + 4096;   //B OV
	DA_Test[2] = Main_DA[2] + 8192;   //C
	DA_Test[3] = Main_DA[3] + 12288;  //D
	DA_Test[4] = Main_DA[4] + 16384;  //E
	DA_Test[5] = Main_DA[5] + 20480;  //F
	DA_Test[6] = Main_DA[6] + 24576;  //G
	DA_Test[7] = Main_DA[7] + 28672;  //H

//	DA传输速率为15MHZ,传输数据16bit,故需要的最小延时(不包括DSP向FPGA传数据)为1.067us
	for(i=0;i<8;i++)
	{
		*( DRAM_data + DA_Main_Addr )  = (((int)DA_Test[i])&0xFFFF) ;
		delay_t(13);//此处需要延时约1us=delay_t(12),等到DA转换完成再写入下一个数据
	}
	return;
}

void DA_Initialization(void)
{
	*( DRAM_data + DA_Main_Addr )  = 0x8002 ;          //DA芯片固有设置控制字
	delay_t(20);
	*( DRAM_data + DA_Main_Addr )  = 0x8002 ;          //DA芯片固有设置控制字
	delay_t(20);
	return;
}

void InitInterrupts(void)
{
	DINT; // Disable CPU interrupts and clear all CPU interrupt flags:

	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the DSP2833x_PieCtrl.c file.
	InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

	//初始化中断向量表
	InitPieVectTable();

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.XINT1 = &xint1_isr;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;    // This is needed to disable write to EALLOW protected registers

	InitCpuTimers0();
	// Configure CPU-Timer 0, interrupt every second:
	// 三个参数，第一个表示哪个定时器，第二个表示定时器频率，第三个表示定时器中断周期值单位us
	// 20:50K 40:25K  80:12k 200:5K
	ConfigCpuTimer(&CpuTimer0, 150, 80);   //控制周期80us

	// Use write-only instruction to set TSS bit = 0
	// 相当于StartCpuTimer0(),此处不使用，故注释掉
	//CpuTimer0Regs.TCR.all = 0x4001;

	IER |= M_INT1;
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  //Enable the PIE block,使能PIE
	PieCtrlRegs.PIEIER1.bit.INTx4 = 1;  //Enable PIE Gropu 1 INT4
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;	//Enable TINT0 in the PIE: Group 1 interrupt 7 CpuTimer0 IER

	// GPIO17 is XINT1
    EALLOW;
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 17;   // Xint1 is GPIO17
    EDIS;
	XIntruptRegs.XINT1CR.bit.POLARITY = 0;	// 0:下降沿触发; 1:上升沿触发
    XIntruptRegs.XINT1CR.bit.ENABLE = 1;  	// Enable Xint1

	EINT;
	ERTM;
}

void InitFlash(void)
{
	EALLOW;
//Enable Flash Pipeline mode to improve performance
//of code executed from Flash.
	FlashRegs.FOPT.bit.ENPIPE = 1;

//                CAUTION
//Minimum waitstates required for the flash operating
//at a given CPU rate must be characterized by TI.
//Refer to the datasheet for the latest information.
//Set the Paged Waitstate for the Flash
	FlashRegs.FBANKWAIT.bit.PAGEWAIT = 5;

//Set the Random Waitstate for the Flash
	FlashRegs.FBANKWAIT.bit.RANDWAIT = 5;

//Set the Waitstate for the OTP
	FlashRegs.FOTPWAIT.bit.OTPWAIT = 8;

//                CAUTION
//ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
	FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
	FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;
	EDIS;

//Force a pipeline flush to ensure that the write to
//the last register configured occurs before returning.

	delay_t(2);
}

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
	while (SourceAddr < SourceEndAddr)
	{
		*DestAddr++ = *SourceAddr++;
	}
	return;
}

//----------------------------  控制相关     ----------------------------
//XINT1n中断服务程序,主要处理fault信号,保证及时的故障响应,下降沿触发
interrupt void xint1_isr(void)
{
	GpioDataRegs.GPBSET.bit.GPIO48 = 1; //计算XINT1n中断执行时间

	Run_Stop_Flag = 0;

	*( DRAM_data + Shut ) = 100;  //关闭PWM
	delay_t(10);
	*( DRAM_data + Shut ) = 100;
	delay_t(10);

	ERROR_flag |= IGBT_Fault_State;
	Read_Fault_State();
	//失联：ERROR_flag=0x0020；分控故障：ERROR_flag=0x0060

	GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;
	// Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//主要的中断控制函数
interrupt void cpu_timer0_isr(void)
{
	GpioDataRegs.GPBSET.bit.GPIO35 = 1;   //统计中断时间
	CpuTimer0Count++;

	*( DRAM_data + Req_DC_Voltage ) = 100; //全局指令帧开始下发到回传数据全部接收需要24us
	*( DRAM_data + Sample ) = 100;
//------------------载波清零,保证每个载波相移都是90us,即1/12个开关周期-------------//
	switch (CpuTimer0Count)
	{
	case 1: {
				*( DRAM_data + L_Carrier_A1_Addr) = 100;
				*( DRAM_data + L_Carrier_B1_Addr) = 100;
				*( DRAM_data + L_Carrier_C1_Addr) = 100;
				break;
			}
	case 2: {
				*( DRAM_data + L_Carrier_A2_Addr) = 100;
				*( DRAM_data + L_Carrier_B2_Addr) = 100;
				*( DRAM_data + L_Carrier_C2_Addr) = 100;
				break;
			}
	case 3: {
				*( DRAM_data + L_Carrier_A3_Addr) = 100;
				*( DRAM_data + L_Carrier_B3_Addr) = 100;
				*( DRAM_data + L_Carrier_C3_Addr) = 100;
				break;
			}
	case 4: {
				*( DRAM_data + L_Carrier_A4_Addr) = 100;
				*( DRAM_data + L_Carrier_B4_Addr) = 100;
				*( DRAM_data + L_Carrier_C4_Addr) = 100;
				break;
			}
	case 5: {
				*( DRAM_data + L_Carrier_A5_Addr) = 100;
				*( DRAM_data + L_Carrier_B5_Addr) = 100;
				*( DRAM_data + L_Carrier_C5_Addr) = 100;
				break;
			}
	case 6: {
				*( DRAM_data + L_Carrier_A6_Addr) = 100;
				*( DRAM_data + L_Carrier_B6_Addr) = 100;
				*( DRAM_data + L_Carrier_C6_Addr) = 100;
				break;
			}
	case 7: {
				*( DRAM_data + L_Carrier_A7_Addr) = 100;
				*( DRAM_data + L_Carrier_B7_Addr) = 100;
				*( DRAM_data + L_Carrier_C7_Addr) = 100;
				break;
			}
	case 8: {
				*( DRAM_data + L_Carrier_A8_Addr) = 100;
				*( DRAM_data + L_Carrier_B8_Addr) = 100;
				*( DRAM_data + L_Carrier_C8_Addr) = 100;
				break;
			}
	case 9: {
				*( DRAM_data + L_Carrier_A9_Addr) = 100;
				*( DRAM_data + L_Carrier_B9_Addr) = 100;
				*( DRAM_data + L_Carrier_C9_Addr) = 100;
				break;
			}
	case 10: {
				*( DRAM_data + L_Carrier_A10_Addr) = 100;
				*( DRAM_data + L_Carrier_B10_Addr) = 100;
				*( DRAM_data + L_Carrier_C10_Addr) = 100;
				break;
			}
	case 11: {
				*( DRAM_data + L_Carrier_A11_Addr) = 100;
				*( DRAM_data + L_Carrier_B11_Addr) = 100;
				*( DRAM_data + L_Carrier_C11_Addr) = 100;
				break;
			}
	case 12: {
				*( DRAM_data + L_Carrier_A12_Addr) = 100;
				*( DRAM_data + L_Carrier_B12_Addr) = 100;
				*( DRAM_data + L_Carrier_C12_Addr) = 100;
				break;
			}
	case 13: {
				*( DRAM_data + R_Carrier_A1_Addr) = 100;
				*( DRAM_data + R_Carrier_B1_Addr) = 100;
				*( DRAM_data + R_Carrier_C1_Addr) = 100;
				break;
			}
	case 14: {
				*( DRAM_data + R_Carrier_A2_Addr) = 100;
				*( DRAM_data + R_Carrier_B2_Addr) = 100;
				*( DRAM_data + R_Carrier_C2_Addr) = 100;
				break;
			}
	case 15: {
				*( DRAM_data + R_Carrier_A3_Addr) = 100;
				*( DRAM_data + R_Carrier_B3_Addr) = 100;
				*( DRAM_data + R_Carrier_C3_Addr) = 100;
				break;
			}
	case 16: {
				*( DRAM_data + R_Carrier_A4_Addr) = 100;
				*( DRAM_data + R_Carrier_B4_Addr) = 100;
				*( DRAM_data + R_Carrier_C4_Addr) = 100;
				break;
			}
	case 17: {
				*( DRAM_data + R_Carrier_A5_Addr) = 100;
				*( DRAM_data + R_Carrier_B5_Addr) = 100;
				*( DRAM_data + R_Carrier_C5_Addr) = 100;
				break;
			}
	case 18: {
				*( DRAM_data + R_Carrier_A6_Addr) = 100;
				*( DRAM_data + R_Carrier_B6_Addr) = 100;
				*( DRAM_data + R_Carrier_C6_Addr) = 100;
				break;
			}
	case 19: {
				*( DRAM_data + R_Carrier_A7_Addr) = 100;
				*( DRAM_data + R_Carrier_B7_Addr) = 100;
				*( DRAM_data + R_Carrier_C7_Addr) = 100;
				break;
			}
	case 20: {
				*( DRAM_data + R_Carrier_A8_Addr) = 100;
				*( DRAM_data + R_Carrier_B8_Addr) = 100;
				*( DRAM_data + R_Carrier_C8_Addr) = 100;
				break;
			}
	case 21: {
				*( DRAM_data + R_Carrier_A9_Addr) = 100;
				*( DRAM_data + R_Carrier_B9_Addr) = 100;
				*( DRAM_data + R_Carrier_C9_Addr) = 100;
				break;
			}
	case 22: {
				*( DRAM_data + R_Carrier_A10_Addr) = 100;
				*( DRAM_data + R_Carrier_B10_Addr) = 100;
				*( DRAM_data + R_Carrier_C10_Addr) = 100;
				break;
			}
	case 23: {
				*( DRAM_data + R_Carrier_A11_Addr) = 100;
				*( DRAM_data + R_Carrier_B11_Addr) = 100;
				*( DRAM_data + R_Carrier_C11_Addr) = 100;
				break;
			}
	case 24: {
				*( DRAM_data + R_Carrier_A12_Addr) = 100;
				*( DRAM_data + R_Carrier_B12_Addr) = 100;
				*( DRAM_data + R_Carrier_C12_Addr) = 100;
				break;
			}
	default:
		{ break; }
	}
//-------------------------------载波清零结束-------------------------------//

//--------------------------------PWM使能---------------------------------//
	if(ERROR_flag!=0)
	{
		Run_Stop_Flag = 0;
	}

	if((Run_Stop_Flag == 1) &&(ERROR_flag==0)) //通过触摸屏点击系统启动，同时也没有任何外部故障
	{
		if(S_OFF)
		{
			Run_Stop_Flag = 0;
			ERROR_flag |= Conductor_Fault;	//启动电阻尚未旁路
			*( DRAM_data + Shut ) = 100;
			delay_t(150);
		}
		else if(Run_Flag == 1)  //CMPR在计数峰值重载,所以延迟1个周期使能PWM
		{
			*( DRAM_data + Run ) = 100;   //开通分控上的PWM
			delay_t(10);
//				GpioDataRegs.GPBSET.bit.GPIO35 = 1;
		}
		Run_Flag = Run_Flag + 1;
		if(Run_Flag > 32000) Run_Flag = 2;  //只发送一次同步Run信号
	}
	else if(Run_Flag != (-1)) //赋值-1是为了只发送一次Shut信号
	{
		*( DRAM_data + Shut ) = 100;  //由触摸屏给定使能，否则危险
		delay_t(150);
		Run_Flag = -1;  //只发送一次Shut信号
		*( DRAM_data + Shut ) = 100;
	}
//------------------------------PWM使能结束--------------------------------//

//--------------------------DSP与FPGA开始交换数据----------------------------//
//读取并网点电压
	ADC_VAB = *( DRAM_data + AD_PCC_VA_Addr );
	ADC_VBC = *( DRAM_data + AD_PCC_VB_Addr );
	ADC_VCA = *( DRAM_data + AD_PCC_VC_Addr );
//读取负载电流
//	ADC_IA = *( DRAM_data + AD_Load_IA_Addr );
//	ADC_IB = *( DRAM_data + AD_Load_IB_Addr );
//	ADC_IC = *( DRAM_data + AD_Load_IC_Addr );
//读取SVG变流器输出电流
	ADC_IA_INV = *( DRAM_data + AD_INV_IA_Addr );  //V6
	ADC_IB_INV = *( DRAM_data + AD_INV_IB_Addr );  //V5
	ADC_IC_INV = *( DRAM_data + AD_INV_IC_Addr );  //V4
//		IA_Load_Pre = ADC_IA * K_IA;
//		IB_Load_Pre = ADC_IB * K_IB;

// ----------------------------数据简单处理模块------------------------------//
// -------作用：
// -------1.零漂补偿
// -------2.定标
// ----------------------------------------------------------------------
//零漂补偿
//1.先将调压器调整到零，采集回来的信号是否为零，不为零则进行补偿，使得补偿后的信号为零
//电网线电压定标方法及原理：
//2.不断的改变调压器的输出，根据得到的结果来拟合，从而得到UA，UB，UC等
//由于定标的时候没有考虑到采样电路的相位关系，所以导致：
//ADC_VA实际对应的是AB线电压
//ADC_VB实际对应的是BC线电压
//ADC_VC实际对应的是CA线电压
	UBC_DRF = ADC_VBC * K_UB + UB_Z;
	UCA_DRF = ADC_VCA * K_UC + UC_Z;
	UAB_DRF = ADC_VAB * K_UA + UA_Z;
// 当检测线电压时,做如下变换：将瞬时线电压变换为对应的瞬时相电压
// 线电压和相电压之间的瞬时值相差sqrt(3) 即 线电压 = sqrt(3)* 相电压
	U_ADRF = (UAB_DRF - UCA_DRF) * UL2P;
	U_BDRF = (UBC_DRF - UAB_DRF) * UL2P;
	U_CDRF = -U_ADRF - U_BDRF;
//原理图与PCB不对应,原理图是ABC,PCB是CAB,所以在软件中调换顺序
	ICSVG = (ADC_IA_INV * K_IA_INV + IA_Z_INV);  //SVG侧电流不能直接去除直流分量,控制不好时就会存在直流分量,这是需要检测回来的
	IASVG = (ADC_IB_INV * K_IB_INV + IB_Z_INV);  //WX定义的电流正方向与实验室相反
	IBSVG = -IASVG  - ICSVG;
//	IA_Load = (IA_Load_Pre - IA_Load_Z) * KLoad_ICT;
//	IB_Load = (IB_Load_Pre - IB_Load_Z) * KLoad_ICT;
//	IC_Load = -IA_Load - IB_Load;

// -------------------3/2变化变换模块及开环并网电压指令计算------------------------
// -------作用：
// -------1.电网电压和电流的3/2变换
// ------------------------------------------------------------------------
//--------------------------graph_data波形，用于调试观察----------------------//
//		if(m>=400)
//			m = 0;
//		else
//			m += 1;
//		graph_data1[m]= U_ADRF;
//		graph_data2[m]= U_BDRF;
//		graph_data3[m]= U_CDRF;
//----------------------------graph_data波形观察结束------------------------//
//并网电压d-q变换
	CON3S2S(U_ADRF,U_BDRF,U_CDRF);  //3S2S变换Ualpha和Ubeta的幅值扩大了sqrt(3/2)倍
	U_alpha_open_pre = LS1;  //未作相角补偿前的开环电压
	U_beta_open_pre = LS2;
	US_Pre = sqrt(U_alpha_open_pre * U_alpha_open_pre + U_beta_open_pre * U_beta_open_pre); //math.h自带开平方函数
	US = ( 1 - Delta_T * Ts_US ) * US + US_Pre * Delta_T * Ts_US; //差分方程，实现滤波
	US_Display = SQRT2_3 * US;  //应该缩小sqrt(2/3)倍才是实际的相电压幅值，3S2S变换扩大了sqrt(2/3)倍
//----------------------------开始PLL锁相---------------------------------//
//利用旋转坐标(alpha_beta), Clark变换求电网电压相角,作为参考对照THETA_C_ab = Theta_Upcc_Ref
//		THETA_C_ab = ARCTAN(U_alpha_open_pre,U_beta_open_pre);  //Clark变换仅作为参考观测量
	Phase_Lock();
	Sin_Value(THETA_C);
	U_alpha_open_after = US * cosVal;  //作完补偿的开环前馈电压,以cos方式锁相
	U_beta_open_after =  US * sinVal;
//此处电压幅值相对于Ualpha和Ubeta又缩小了sqrt(2/3),所以幅值与采样得到的幅值近似相等
	CON2S3S(U_alpha_open_after, U_beta_open_after);
//前馈电压 并网电压指令
	U_aout_open=LS1;  //作完补偿后的开环前馈电压
	U_bout_open=LS2;
	U_cout_open=LS3;

//并网电流d-q变换
	CON3S2S(IASVG, IBSVG, ICSVG);
	I_alpha = LS1;  //IA、IB、IC是按照实际电流定标的
	I_beta = LS2;
	CON2S2R_W(I_alpha, I_beta);  //直接进行2S2R变换
	I_d_Pre = SQRT2_3 * LS1;  //此处I_d应缩小sqrt(2/3)倍,因为3S2S扩大了sqrt(3/2)
	I_q_Pre = SQRT2_3 * LS2;  //此处I_q也应缩小sqrt(2/3)倍
	I_d = filter_2nd_dq(I_d_Pre, filter_2nd_d);
	I_q = filter_2nd_dq(I_q_Pre, filter_2nd_q);

	ISVG_Amp = fabs(I_q);
	if(ISVG_Amp<3) ISVG_Amp = 3;  //3A以下直接限幅
// -------------------------3/2变化变换模块块结束---------------------

// --------------------------直流侧电压控制--------------------------
// -----作用：
// -----1.求取直流侧电压的相内平均值
// -----2.定标
// -----部分参数说明：
// -----UDCREF：直流侧电容电压参考值，由触摸屏赋值
// -----Idref： 直流侧稳压控制PI输出
// ------------------------------------------------------------------------------
	delay_t(Sine_m);
	Local_DC_Process(); //从进入中断开始下发读取直流侧电压数据的全局指令，到这里历时24us
//1. 直流母线电压平均值控制
//求取直流侧电压的相内平均值,针对六个级联单元,N=6
	UDC_averA = ( UDC_AS1 + UDC_AS2 + UDC_AS3 + UDC_AS4 + UDC_AS5 + UDC_AS6 + UDC_AS7 + UDC_AS8 + UDC_AS9 + UDC_AS10 + UDC_AS11 + UDC_AS12) * avg_Nm;
	UDC_averB = ( UDC_BS1 + UDC_BS2 + UDC_BS3 + UDC_BS4 + UDC_BS5 + UDC_BS6 + UDC_BS7 + UDC_BS8 + UDC_BS9 + UDC_BS10 + UDC_BS11 + UDC_BS12) * avg_Nm;
	UDC_averC = ( UDC_CS1 + UDC_CS2 + UDC_CS3 + UDC_CS4 + UDC_CS5 + UDC_CS6 + UDC_CS7 + UDC_CS8 + UDC_CS9 + UDC_CS10 + UDC_CS11 + UDC_CS12) * avg_Nm;
//	UDC_averA = ( UDC_AR1 + UDC_AR2 + UDC_AR3 + UDC_AR4 + UDC_AR5 + UDC_AR6) * avg_Nm;
//	UDC_averB = ( UDC_BR1 + UDC_BR2 + UDC_BR3 + UDC_BR4 + UDC_BR5 + UDC_BR6) * avg_Nm;
//	UDC_averC = ( UDC_CR1 + UDC_CR2 + UDC_CR3 + UDC_CR4 + UDC_CR5 + UDC_CR6) * avg_Nm;
//此处将UDC_aver从int改为float,会频繁进行微小的充放电
	UDC_aver = ( UDC_averA + UDC_averB + UDC_averC ) * 0.333333;  //每个电容电压的平均值

// -----------------------------有功无功解耦控制-----------------------------//
// ----作用：
// ----1.有功无功解耦
// ----2.定标
// ----部分参数说明：
// ----Idref：直流侧稳压有功指令
// ----Q_REF：无功指令
// ----------------------------------------------------------------------
	if((Run_Stop_Flag==1)&&(ERROR_flag==0))  //if(系统启动，同时也没有任何外部故障)
	{
		if(Udc_slip < Ctrl_Para.UDCREF - 1)   //全局直流母线电压控制 直流侧电容电压参考值，触摸屏赋值
		{//直流母线电压升高 斜坡函数
			Udc_slip = Udc_slip + Ctrl_Para.UDC_SLIP;	  //直流电压给定为一斜坡函数,0.004对应1s涨25V
		//	Udc_slip = UDC_aver + Ctrl_Para.UDC_SLIP;
		}
		else if( Udc_slip > Ctrl_Para.UDCREF + 1)
		{//直流母线电压下降 斜坡函数
			Udc_slip = Udc_slip - Ctrl_Para.UDC_SLIP;
		}
		else
		{
			Udc_slip = Ctrl_Para.UDCREF;
		}
//全局直流母线电压PI调节，斜坡给定直流电压参考值  该PI函数内部对有功指令Idref进行了更改及限幅
		UDC_err = UDC_aver - Udc_slip; //WS定义电流正方向为SVG流入Grid,把SVG视作无功电源,而实验室把SVG视作负载,电流正方向为Grid流入SVG
		//电压环PI
		UDC_PI(Ctrl_Para.UDC_Kp ,Ctrl_Para.UDC_Ki ); //在子函数UDC_PI中已经：Idref = UDC_PI_out;

		Q = Out_Limit(Q_REF, 60, -60);	//给定值限幅
		I_derror = Idref - I_d;   		//d轴电流Error
		I_qerror = Q - I_q;   			//q轴电流Error
	  //有功电流环PI
		Current_PI1(Ctrl_Para.Ud_Kp ,Ctrl_Para.Ud_Ki );
	  //无功电流环PI
		Current_PI2(Ctrl_Para.Ud_Kp ,Ctrl_Para.Ud_Ki );
	  //有功无功耦合项
		U_dF = U_dout;  //暂不解耦
		U_qF = U_qout;

		Sin_Value(THETA_C);
		CON2R2S_W(U_dF,U_qF);
		U_alpha_close = LS1;
		U_beta_close = LS2;
		CON2S3S(U_alpha_close, U_beta_close);
		U_aout_close = LS1;
		U_bout_close = LS2;
		U_cout_close = LS3;
	}
	else
	{
		Udc_slip = UDC_aver;
		UDC_err = 0;
		UDC_PI_integ = 0;
		Idref = 0;
		Curr_PI1_integ = 0;
		Curr_PI2_integ = 0;
		U_dout = 0;
		U_qout = 0;
		U_aout_close = 0;
		U_bout_close = 0;
		U_cout_close = 0;
	}
//		if(m==2) GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;
//		if(Q < 40) GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;
//		else GpioDataRegs.GPBSET.bit.GPIO35 = 1;

//		if((Run_Stop_Flag==1)&&(ERROR_flag==0)) //if(系统启动，同时也没有任何外部故障)
//		{
//------------------------------P+重复控制算法-----------------------------//
//			if (ahead_n >= 125)  //每次++
//				ahead_n = 0;
//			ii = ahead_n + Ahead_Point;
//			if (ii >= 125)
//				ii -= 125;
//			new_out_d = out_pwm_d[ahead_n] * Qr + ahead_pwm_d[ii] * Ctrl_Para.Rpt_Kp;
//			new_out_q = out_pwm_q[ahead_n] * Qr + ahead_pwm_q[ii] * Ctrl_Para.Rpt_Kp;
//			/* 输出反馈通道的重复误差 */
//			out_pwm_d[ahead_n] = new_out_d;
//			out_pwm_q[ahead_n] = new_out_q;
//			/* new error:前向通道通道误差,2nd滤波器为C(Z)的补偿器,消除高频分量,提高稳定性 */
////			ahead_pwm_d[ahead_n] = filter_2nd_dq(I_derror, burrfer_error_d);
////			ahead_pwm_q[ahead_n] = filter_2nd_dq(I_qerror, burrfer_error_q);
//			ahead_pwm_d[ahead_n] = I_derror;
//			ahead_pwm_q[ahead_n] = I_qerror;
//			ahead_n += 1;
//			/* rcp 原误差 + 重复误差 */
//			Repetive_d = I_derror + new_out_d;  //P+重复控制
//			Repetive_q = I_qerror + new_out_q;
//
//			/* 比例调整输出,重复控制P增益 = Kr_Rpt*I_Kp */
//			Repetive_out_d = Repetive_PI1(Ctrl_Para.Ud_Kp ,Ctrl_Para.Ud_Ki );
//			Repetive_out_q = Repetive_PI2(Ctrl_Para.Ud_Kp ,Ctrl_Para.Ud_Ki );
//
//		  	Sin_Value(THETA_C);
//		  	CON2R2S_W(Repetive_out_d,Repetive_out_q);
//		 	U_alpha_close = LS1;
//		  	U_beta_close = LS2;
//		  	CON2S3S(U_alpha_close, U_beta_close);
//		  	U_aout_close = LS1;
//		  	U_bout_close = LS2;
//		  	U_cout_close = LS3;
//------------------------------P+重复控制结束-----------------------------//
//	   	}
//		else
//	   	{
//		  	Rpt_PI1_integ = 0;
//		  	Rpt_PI2_integ = 0;
//		  	for( i=0; i<125; i++)
//		  	{
//		  		out_pwm_d[i] = 0;
//		  		out_pwm_q[i] = 0;
//		  		ahead_pwm_d[i] = 0;
//		  		ahead_pwm_q[i] = 0;
//		  	}
//	   	}
//		if(m==3) GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;

// ------------------------------相内平衡控制-------------------------------//
	if((Run_Stop_Flag==1) && (ERROR_flag==0)) //若SVG启动,UDC的PI开始调节
	{
		if(ISVG_Amp > 3)
		{
			if(I_q < 0)
				Gama = -90 - THETA_d_Com;	//Iq＜0,SVG为电容,为90°
			else Gama = 90 + THETA_d_Com;	//Iq＞0,SVG为电感
//				Gama = ARCTAN(I_d,I_q);			//电流方向定义为SVG流向电网
			THETA_IA = THETA_C + Gama;		//A相电流相角,做相内平衡用
		}
	}
	else	 //不使能相内平衡时, 若干参数清零
	{
		UDC.inA1=0;
		UDC.inA2=0;
		UDC.inA3=0;
		UDC.inA4=0;
		UDC.inA5=0;
		UDC.inA6=0;
		UDC.inA7=0;
		UDC.inA8=0;
		UDC.inA9=0;
		UDC.inA10=0;
		UDC.inA11=0;
		UDC.inA12=0;

		UDC.inB1=0;
		UDC.inB2=0;
		UDC.inB3=0;
		UDC.inB4=0;
		UDC.inB5=0;
		UDC.inB6=0;
		UDC.inB7=0;
		UDC.inB8=0;
		UDC.inB9=0;
		UDC.inB10=0;
		UDC.inB11=0;
		UDC.inB12=0;

		UDC.inC1=0;
		UDC.inC2=0;
		UDC.inC3=0;
		UDC.inC4=0;
		UDC.inC5=0;
		UDC.inC6=0;
		UDC.inC7=0;
		UDC.inC8=0;
		UDC.inC9=0;
		UDC.inC10=0;
		UDC.inC11=0;
		UDC.inC12=0;
	}
//		if(m==4) GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;
//	 --------------------------相间平衡控制--------------------------
//	 -------------------------------------------------------------
	UDC.deltA = UDC_averA - UDC_aver;  //参考万的论文
	UDC.deltB = UDC_averB - UDC_aver;
	UDC.deltC = UDC_averC - UDC_aver;
	UDC.delt_alpha = SQRT3_2 * UDC.deltA;  //sqrt(1.5)
	UDC.delt_beta = SQRT2d2 * (UDC.deltB - UDC.deltC); //sqrt(2)/2
	UDC.delta_amp = sqrt(UDC.delt_alpha*UDC.delt_alpha + UDC.delt_beta*UDC.delt_beta);

	if((Run_Stop_Flag==1)&&(ERROR_flag==0))
	{
		P2P_Balance_Neg_U();  //会向电网电压中注入负序电流,所以直接将负序电流添加到电流环会不会更好
	}
	else
	{
		UDC.delta_amp = 0;
		U_neg_amp = 0;
		U_Z_amp = 0;
		UA_P2P = 0;
		UB_P2P = 0;
		UC_P2P = 0;
	}

//-------------------------------PWM调制波输出------------------------------//
	M_out.Pa = U_aout_open * U_FtoI + U_aout_close + UA_P2P;
	M_out.Pb = U_bout_open * U_FtoI + U_bout_close + UB_P2P;
	M_out.Pc = U_cout_open * U_FtoI + U_cout_close + UC_P2P;
	switch(CpuTimer0Count)
	{
	case 1:
	case 7:
	case 13:
	case 19:
	{
		UDC.deltA1 = UDC_averA - UDC_AS1;
		UDC.deltA2 = UDC_averA - UDC_AS2;
		Sin_Value(THETA_IA);		//A相电流相位
		UDC.inA1 = -Ctrl_Para.Kp_in * UDC.deltA1 * cosVal;
		UDC.inA2 = -Ctrl_Para.Kp_in * UDC.deltA2 * cosVal;

		UDC.deltB1 = UDC_averB - UDC_BS1;
		UDC.deltB2 = UDC_averB - UDC_BS2;
		Sin_Value(THETA_IA - 120);	//B相电流相位
		UDC.inB1 = -Ctrl_Para.Kp_in * UDC.deltB1 * cosVal;
		UDC.inB2 = -Ctrl_Para.Kp_in * UDC.deltB2 * cosVal;

		UDC.deltC1 = UDC_averC - UDC_CS1;
		UDC.deltC2 = UDC_averC - UDC_CS2;
		Sin_Value(THETA_IA + 120);	//C相电流相位
		UDC.inC1 = -Ctrl_Para.Kp_in * UDC.deltC1 * cosVal;
		UDC.inC2 = -Ctrl_Para.Kp_in * UDC.deltC2 * cosVal;
//并网第1步,启动相内平衡和相间平衡,标幺化
		M_out.Pa1 =(M_out.Pa + UDC.inA1) * Avg_N / UDC_AR1;
		M_out.Pb1 =(M_out.Pb + UDC.inB1) * Avg_N / UDC_BR1;
		M_out.Pc1 =(M_out.Pc + UDC.inC1) * Avg_N / UDC_CR1;
		M_out.Pa2 =(M_out.Pa + UDC.inA2) * Avg_N / UDC_AR2;
		M_out.Pb2 =(M_out.Pb + UDC.inB2) * Avg_N / UDC_BR2;
		M_out.Pc2 =(M_out.Pc + UDC.inC2) * Avg_N / UDC_CR2;
//直流偏置,将负数变为正值
		M_out.Oa1 = (unsigned int)(M_out.Pa1 + Half_period);
		M_out.Ob1 = (unsigned int)(M_out.Pb1 + Half_period);
		M_out.Oc1 = (unsigned int)(M_out.Pc1 + Half_period);
		M_out.Oa2 = (unsigned int)(M_out.Pa2 + Half_period);
		M_out.Ob2 = (unsigned int)(M_out.Pb2 + Half_period);
		M_out.Oc2 = (unsigned int)(M_out.Pc2 + Half_period);
//PWM调制波输出限幅
		M_out.Oa1 = Out_Limit(M_out.Oa1, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob1 = Out_Limit(M_out.Ob1, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc1 = Out_Limit(M_out.Oc1, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oa2 = Out_Limit(M_out.Oa2, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob2 = Out_Limit(M_out.Ob2, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc2 = Out_Limit(M_out.Oc2, ModuWaveLimit_High, ModuWaveLimit_Low);
//写调制波数据
		*( DRAM_data + Modulation_A1_Addr ) = M_out.Oa1 ;
		*( DRAM_data + Modulation_B1_Addr ) = M_out.Ob1 ;
		*( DRAM_data + Modulation_C1_Addr ) = M_out.Oc1 ;
		*( DRAM_data + Modulation_A2_Addr ) = M_out.Oa2 ;
		*( DRAM_data + Modulation_B2_Addr ) = M_out.Ob2 ;
		*( DRAM_data + Modulation_C2_Addr ) = M_out.Oc2 ;
		break;
	}
	case 2:
	case 8:
	case 14:
	case 20:
	{
		UDC.deltA3 = UDC_averA - UDC_AS3;
		UDC.deltA4 = UDC_averA - UDC_AS4;
		UDC.deltB3 = UDC_averB - UDC_BS3;
		UDC.deltB4 = UDC_averB - UDC_BS4;
		UDC.deltC3 = UDC_averC - UDC_CS3;
		UDC.deltC4 = UDC_averC - UDC_CS4;
		Sin_Value(THETA_IA);
		UDC.inA3 = -Ctrl_Para.Kp_in * UDC.deltA3 * cosVal;
		UDC.inA4 = -Ctrl_Para.Kp_in * UDC.deltA4 * cosVal;
		Sin_Value(THETA_IA - 120);
		UDC.inB3 = -Ctrl_Para.Kp_in * UDC.deltB3 * cosVal;
		UDC.inB4 = -Ctrl_Para.Kp_in * UDC.deltB4 * cosVal;
		Sin_Value(THETA_IA + 120);
		UDC.inC3 = -Ctrl_Para.Kp_in * UDC.deltC3 * cosVal;
		UDC.inC4 = -Ctrl_Para.Kp_in * UDC.deltC4 * cosVal;

		M_out.Pa3 =(M_out.Pa + UDC.inA3) * Avg_N / UDC_AR3;
		M_out.Pb3 =(M_out.Pb + UDC.inB3) * Avg_N / UDC_BR3;
		M_out.Pc3 =(M_out.Pc + UDC.inC3) * Avg_N / UDC_CR3;
		M_out.Pa4 =(M_out.Pa + UDC.inA4) * Avg_N / UDC_AR4;
		M_out.Pb4 =(M_out.Pb + UDC.inB4) * Avg_N / UDC_BR4;
		M_out.Pc4 =(M_out.Pc + UDC.inC4) * Avg_N / UDC_CR4;

		M_out.Oa3 = (unsigned int)(M_out.Pa3 + Half_period);
		M_out.Ob3 = (unsigned int)(M_out.Pb3 + Half_period);
		M_out.Oc3 = (unsigned int)(M_out.Pc3 + Half_period);
		M_out.Oa4 = (unsigned int)(M_out.Pa4 + Half_period);
		M_out.Ob4 = (unsigned int)(M_out.Pb4 + Half_period);
		M_out.Oc4 = (unsigned int)(M_out.Pc4 + Half_period);

		M_out.Oa3 = Out_Limit(M_out.Oa3, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob3 = Out_Limit(M_out.Ob3, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc3 = Out_Limit(M_out.Oc3, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oa4 = Out_Limit(M_out.Oa4, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob4 = Out_Limit(M_out.Ob4, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc4 = Out_Limit(M_out.Oc4, ModuWaveLimit_High, ModuWaveLimit_Low);

		*( DRAM_data + Modulation_A3_Addr ) = M_out.Oa3 ;
		*( DRAM_data + Modulation_B3_Addr ) = M_out.Ob3 ;
		*( DRAM_data + Modulation_C3_Addr ) = M_out.Oc3 ;
		*( DRAM_data + Modulation_A4_Addr ) = M_out.Oa4 ;
		*( DRAM_data + Modulation_B4_Addr ) = M_out.Ob4 ;
		*( DRAM_data + Modulation_C4_Addr ) = M_out.Oc4 ;
		break;
	}
	case 3:
	case 9:
	case 15:
	case 21:
	{
		UDC.deltA5 = UDC_averA - UDC_AS5;
		UDC.deltA6 = UDC_averA - UDC_AS6;
		UDC.deltB5 = UDC_averB - UDC_BS5;
		UDC.deltB6 = UDC_averB - UDC_BS6;
		UDC.deltC5 = UDC_averC - UDC_CS5;
		UDC.deltC6 = UDC_averC - UDC_CS6;
		Sin_Value(THETA_IA);
		UDC.inA5 = -Ctrl_Para.Kp_in * UDC.deltA5 * cosVal;
		UDC.inA6 = -Ctrl_Para.Kp_in * UDC.deltA6 * cosVal;
		Sin_Value(THETA_IA - 120);
		UDC.inB5 = -Ctrl_Para.Kp_in * UDC.deltB5 * cosVal;
		UDC.inB6 = -Ctrl_Para.Kp_in * UDC.deltB6 * cosVal;
		Sin_Value(THETA_IA + 120);
		UDC.inC5 = -Ctrl_Para.Kp_in * UDC.deltC5 * cosVal;
		UDC.inC6 = -Ctrl_Para.Kp_in * UDC.deltC6 * cosVal;

		M_out.Pa5 =(M_out.Pa + UDC.inA5) * Avg_N / UDC_AR5;
		M_out.Pb5 =(M_out.Pb + UDC.inB5) * Avg_N / UDC_BR5;
		M_out.Pc5 =(M_out.Pc + UDC.inC5) * Avg_N / UDC_CR5;
		M_out.Pa6 =(M_out.Pa + UDC.inA6) * Avg_N / UDC_AR6;
		M_out.Pb6 =(M_out.Pb + UDC.inB6) * Avg_N / UDC_BR6;
		M_out.Pc6 =(M_out.Pc + UDC.inC6) * Avg_N / UDC_CR6;

		M_out.Oa5 = (unsigned int)(M_out.Pa5 + Half_period);
		M_out.Ob5 = (unsigned int)(M_out.Pb5 + Half_period);
		M_out.Oc5 = (unsigned int)(M_out.Pc5 + Half_period);
		M_out.Oa6 = (unsigned int)(M_out.Pa6 + Half_period);
		M_out.Ob6 = (unsigned int)(M_out.Pb6 + Half_period);
		M_out.Oc6 = (unsigned int)(M_out.Pc6 + Half_period);

		M_out.Oa5 = Out_Limit(M_out.Oa5, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob5 = Out_Limit(M_out.Ob5, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc5 = Out_Limit(M_out.Oc5, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oa6 = Out_Limit(M_out.Oa6, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob6 = Out_Limit(M_out.Ob6, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc6 = Out_Limit(M_out.Oc6, ModuWaveLimit_High, ModuWaveLimit_Low);

		*( DRAM_data + Modulation_A5_Addr ) = M_out.Oa5 ;
		*( DRAM_data + Modulation_B5_Addr ) = M_out.Ob5 ;
		*( DRAM_data + Modulation_C5_Addr ) = M_out.Oc5 ;
		*( DRAM_data + Modulation_A6_Addr ) = M_out.Oa6 ;
		*( DRAM_data + Modulation_B6_Addr ) = M_out.Ob6 ;
		*( DRAM_data + Modulation_C6_Addr ) = M_out.Oc6 ;
		break;
	}
	case 4:
	case 10:
	case 16:
	case 22:
	{
		UDC.deltA7 = UDC_averA - UDC_AS7;
		UDC.deltA8 = UDC_averA - UDC_AS8;
		UDC.deltB7 = UDC_averB - UDC_BS7;
		UDC.deltB8 = UDC_averB - UDC_BS8;
		UDC.deltC7 = UDC_averC - UDC_CS7;
		UDC.deltC8 = UDC_averC - UDC_CS8;
		Sin_Value(THETA_IA);
		UDC.inA7 = -Ctrl_Para.Kp_in * UDC.deltA7 * cosVal;
		UDC.inA8 = -Ctrl_Para.Kp_in * UDC.deltA8 * cosVal;
		Sin_Value(THETA_IA - 120);//B相电流相位
		UDC.inB7 = -Ctrl_Para.Kp_in * UDC.deltB7 * cosVal;
		UDC.inB8 = -Ctrl_Para.Kp_in * UDC.deltB8 * cosVal;
		Sin_Value(THETA_IA + 120);//C相电流相位
		UDC.inC7 = -Ctrl_Para.Kp_in * UDC.deltC7 * cosVal;
		UDC.inC8 = -Ctrl_Para.Kp_in * UDC.deltC8 * cosVal;

		M_out.Pa7 =(M_out.Pa + UDC.inA7) * Avg_N / UDC_AR7;
		M_out.Pb7 =(M_out.Pb + UDC.inB7) * Avg_N / UDC_BR7;
		M_out.Pc7 =(M_out.Pc + UDC.inC7) * Avg_N / UDC_CR7;
		M_out.Pa8 =(M_out.Pa + UDC.inA8) * Avg_N / UDC_AR8;
		M_out.Pb8 =(M_out.Pb + UDC.inB8) * Avg_N / UDC_BR8;
		M_out.Pc8 =(M_out.Pc + UDC.inC8) * Avg_N / UDC_CR8;

		M_out.Oa7 = (unsigned int)(M_out.Pa7 + Half_period);
		M_out.Ob7 = (unsigned int)(M_out.Pb7 + Half_period);
		M_out.Oc7 = (unsigned int)(M_out.Pc7 + Half_period);
		M_out.Oa8 = (unsigned int)(M_out.Pa8 + Half_period);
		M_out.Ob8 = (unsigned int)(M_out.Pb8 + Half_period);
		M_out.Oc8 = (unsigned int)(M_out.Pc8 + Half_period);

		M_out.Oa7 = Out_Limit(M_out.Oa7, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob7 = Out_Limit(M_out.Ob7, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc7 = Out_Limit(M_out.Oc7, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oa8 = Out_Limit(M_out.Oa8, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob8 = Out_Limit(M_out.Ob8, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc8 = Out_Limit(M_out.Oc8, ModuWaveLimit_High, ModuWaveLimit_Low);

		*( DRAM_data + Modulation_A7_Addr ) = M_out.Oa7 ;
		*( DRAM_data + Modulation_B7_Addr ) = M_out.Ob7 ;
		*( DRAM_data + Modulation_C7_Addr ) = M_out.Oc7 ;
		*( DRAM_data + Modulation_A8_Addr ) = M_out.Oa8 ;
		*( DRAM_data + Modulation_B8_Addr ) = M_out.Ob8 ;
		*( DRAM_data + Modulation_C8_Addr ) = M_out.Oc8 ;
		break;
	}
	case 5:
	case 11:
	case 17:
	case 23:
	{
		UDC.deltA9 = UDC_averA - UDC_AS9;
		UDC.deltA10 = UDC_averA - UDC_AS10;
		UDC.deltB9 = UDC_averB - UDC_BS9;
		UDC.deltB10 = UDC_averB - UDC_BS10;
		UDC.deltC9 = UDC_averC - UDC_CS9;
		UDC.deltC10 = UDC_averC - UDC_CS10;
		Sin_Value(THETA_IA);
		UDC.inA9 = -Ctrl_Para.Kp_in * UDC.deltA9 * cosVal;
		UDC.inA10 = -Ctrl_Para.Kp_in * UDC.deltA10 * cosVal;
		Sin_Value(THETA_IA - 120);//B相电流相位
		UDC.inB9 = -Ctrl_Para.Kp_in * UDC.deltB9 * cosVal;
		UDC.inB10 = -Ctrl_Para.Kp_in * UDC.deltB10 * cosVal;
		Sin_Value(THETA_IA + 120);//C相电流相位
		UDC.inC9 = -Ctrl_Para.Kp_in * UDC.deltC9 * cosVal;
		UDC.inC10 = -Ctrl_Para.Kp_in * UDC.deltC10 * cosVal;

		M_out.Pa9 =(M_out.Pa + UDC.inA9) * Avg_N / UDC_AR9;
		M_out.Pb9 =(M_out.Pb + UDC.inB9) * Avg_N / UDC_BR9;
		M_out.Pc9 =(M_out.Pc + UDC.inC9) * Avg_N / UDC_CR9;
		M_out.Pa10 =(M_out.Pa + UDC.inA10) * Avg_N / UDC_AR10;
		M_out.Pb10 =(M_out.Pb + UDC.inB10) * Avg_N / UDC_BR10;
		M_out.Pc10 =(M_out.Pc + UDC.inC10) * Avg_N / UDC_CR10;

		M_out.Oa9 = (unsigned int)(M_out.Pa9 + Half_period);
		M_out.Ob9 = (unsigned int)(M_out.Pb9 + Half_period);
		M_out.Oc9 = (unsigned int)(M_out.Pc9 + Half_period);
		M_out.Oa10 = (unsigned int)(M_out.Pa10 + Half_period);
		M_out.Ob10 = (unsigned int)(M_out.Pb10 + Half_period);
		M_out.Oc10 = (unsigned int)(M_out.Pc10 + Half_period);

		M_out.Oa9 = Out_Limit(M_out.Oa9, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob9 = Out_Limit(M_out.Ob9, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc9 = Out_Limit(M_out.Oc9, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oa10 = Out_Limit(M_out.Oa10, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob10 = Out_Limit(M_out.Ob10, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc10 = Out_Limit(M_out.Oc10, ModuWaveLimit_High, ModuWaveLimit_Low);

		*( DRAM_data + Modulation_A9_Addr ) = M_out.Oa9 ;
		*( DRAM_data + Modulation_B9_Addr ) = M_out.Ob9 ;
		*( DRAM_data + Modulation_C9_Addr ) = M_out.Oc9 ;
		*( DRAM_data + Modulation_A10_Addr ) = M_out.Oa10 ;
		*( DRAM_data + Modulation_B10_Addr ) = M_out.Ob10 ;
		*( DRAM_data + Modulation_C10_Addr ) = M_out.Oc10 ;

		break;
	}
	case 6:
	case 12:
	case 18:
	case 24:
	{
		UDC.deltA11 = UDC_averA - UDC_AS11;
		UDC.deltA12 = UDC_averA - UDC_AS12;
		UDC.deltB11 = UDC_averB - UDC_BS11;
		UDC.deltB12 = UDC_averB - UDC_BS12;
		UDC.deltC11 = UDC_averC - UDC_CS11;
		UDC.deltC12 = UDC_averC - UDC_CS12;
		Sin_Value(THETA_IA);
		UDC.inA11 = -Ctrl_Para.Kp_in * UDC.deltA11 * cosVal;
		UDC.inA12 = -Ctrl_Para.Kp_in * UDC.deltA12 * cosVal;
		Sin_Value(THETA_IA - 120);//B相电流相位
		UDC.inB11 = -Ctrl_Para.Kp_in * UDC.deltB11 * cosVal;
		UDC.inB12 = -Ctrl_Para.Kp_in * UDC.deltB12 * cosVal;
		Sin_Value(THETA_IA + 120);//C相电流相位
		UDC.inC11 = -Ctrl_Para.Kp_in * UDC.deltC11 * cosVal;
		UDC.inC12 = -Ctrl_Para.Kp_in * UDC.deltC12 * cosVal;

		M_out.Pa11 =(M_out.Pa + UDC.inA11) * Avg_N / UDC_AR11;
		M_out.Pb11 =(M_out.Pb + UDC.inB11) * Avg_N / UDC_BR11;
		M_out.Pc11 =(M_out.Pc + UDC.inC11) * Avg_N / UDC_CR11;
		M_out.Pa12 =(M_out.Pa + UDC.inA12) * Avg_N / UDC_AR12;
		M_out.Pb12 =(M_out.Pb + UDC.inB12) * Avg_N / UDC_BR12;
		M_out.Pc12 =(M_out.Pc + UDC.inC12) * Avg_N / UDC_CR12;

		M_out.Oa11 = (unsigned int)(M_out.Pa11 + Half_period);
		M_out.Ob11 = (unsigned int)(M_out.Pb11 + Half_period);
		M_out.Oc11 = (unsigned int)(M_out.Pc11 + Half_period);
		M_out.Oa12 = (unsigned int)(M_out.Pa12 + Half_period);
		M_out.Ob12 = (unsigned int)(M_out.Pb12 + Half_period);
		M_out.Oc12 = (unsigned int)(M_out.Pc12 + Half_period);

		M_out.Oa11 = Out_Limit(M_out.Oa11, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob11 = Out_Limit(M_out.Ob11, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc11 = Out_Limit(M_out.Oc11, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oa12 = Out_Limit(M_out.Oa12, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Ob12 = Out_Limit(M_out.Ob12, ModuWaveLimit_High, ModuWaveLimit_Low);
		M_out.Oc12 = Out_Limit(M_out.Oc12, ModuWaveLimit_High, ModuWaveLimit_Low);

		*( DRAM_data + Modulation_A11_Addr ) = M_out.Oa11 ;
		*( DRAM_data + Modulation_B11_Addr ) = M_out.Ob11 ;
		*( DRAM_data + Modulation_C11_Addr ) = M_out.Oc11 ;
		*( DRAM_data + Modulation_A12_Addr ) = M_out.Oa12 ;
		*( DRAM_data + Modulation_B12_Addr ) = M_out.Ob12 ;
		*( DRAM_data + Modulation_C12_Addr ) = M_out.Oc12 ;
		break;
	}
	default:
	{
		CpuTimer0Count = 0;
		break;
	}
	}
//-------------------------PWM调制波输出结束---------------------------
	if(CpuTimer0Count == 24)
	{
		CpuTimer0Count = 0;
	}
//-----------------------------设置主控板DA保护阈值---------------------------//
//------------------------------DA output start--------------------------//
	if(Main_DA[0] != (Ipcc_Amp * 408))
	{
		Main_DA[0] = Ipcc_Amp * 408; //A  并网电流阈值 （DA最大输出5V时为 Ipcc_Amp * 816）
		*( DRAM_data + DA_Main_Addr )  = (((int)Main_DA[0])&0xFFFF) ;
	}
	else
	{
		switch(DA_Switch)
		{
			case 1:{ DA0 = THETA_C * 8 + 1000; break;}
			case 2:{ DA0 = Usq_PLL * 50 + 2000;	break;}
			case 3:{ DA0 = UDC_PI_integ * 10 + 2000; break;}
			case 4:{ DA0 = UDC_PI_out * 10 + 2000; break;}
			case 5:{ DA0 = Curr_PI1_integ * 10 + 2000; break;}
			case 6:{ DA0 = Curr_PI2_integ * 10 + 2000; break;}
			case 7:{ DA0 = U_qout * 10 + 2000; break;}
			case 8:{ DA0 = U_dout * 10 + 2000; break;}
			case 9:{ DA0 = I_d * 100 + 2000; break;}
			case 10:{ DA0 = I_q * 100 + 2000; break;}
			case 11:{ DA0 = Idref * 100 + 2000; break;}
			case 12:{ DA0 = UDC.DA1 * 3; break;}
			case 13:{ DA0 = UDC.DB1 * 3; break;}
			case 14:{ DA0 = UDC.DC1 * 3; break;}
			case 15:{ DA0 = (UDC.DA1 - Ctrl_Para.UDCMIN) * 20; break;}
			case 16:{ DA0 = (UDC.DB1 - Ctrl_Para.UDCMIN) * 20; break;}
			case 17:{ DA0 = (UDC.DC1 - Ctrl_Para.UDCMIN) * 20; break;}
			case 18:{ DA0 = (UDC.DA2 - Ctrl_Para.UDCMIN) * 20; break;}
			case 19:{ DA0 = (UDC.DA5 - Ctrl_Para.UDCMIN) * 20; break;}
			case 20:{ DA0 = (UDC.DA8 - Ctrl_Para.UDCMIN) * 20; break;}
			case 21:{ DA0 = (UDC.DA11 - Ctrl_Para.UDCMIN) * 20; break;}
			case 22:{ DA0 = (UDC.DA12 - Ctrl_Para.UDCMIN) * 20; break;}
			case 23:{ DA0 = (UDC.DB2 - Ctrl_Para.UDCMIN) * 20; break;}
			case 24:{ DA0 = (UDC.DB5 - Ctrl_Para.UDCMIN) * 20; break;}
			case 25:{ DA0 = (UDC.DB8 - Ctrl_Para.UDCMIN) * 20; break;}
			case 26:{ DA0 = (UDC.DB11 - Ctrl_Para.UDCMIN) * 20; break;}
			case 27:{ DA0 = (UDC.DB12 - Ctrl_Para.UDCMIN) * 20; break;}
			case 28:{ DA0 = (UDC.DC2 - Ctrl_Para.UDCMIN) * 20; break;}
			case 29:{ DA0 = (UDC.DC5 - Ctrl_Para.UDCMIN) * 20; break;}
			case 30:{ DA0 = (UDC.DC8 - Ctrl_Para.UDCMIN) * 20; break;}
			case 31:{ DA0 = (UDC.DC11 - Ctrl_Para.UDCMIN) * 20; break;}
			case 32:{ DA0 = (UDC.DC12 - Ctrl_Para.UDCMIN) * 20; break;}
			default:
				break;
		}
		if(DA0 > 4000) DA0 = 4000; //DA数字量最大为4080
		if(DA0 < 0) DA0 = 10;
		Main_DA[1] = DA0 + 4096;
		*( DRAM_data + DA_Main_Addr )  = (((int)Main_DA[1])&0xFFFF) ;
		delay_t(13);

//	Main_DA_OUT();
	}

//--------------------------------DA操作结束-------------------------------//
	GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;  //统计中断时间

//配合EINT使用Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//需要外部定义 Curr_PI1_integ = 0;Curr_PI1_out = 0;
//定义积分限幅和输出限幅：
void Current_PI1( float Kp, float Ki )
{
	Curr_PI1_integ = Curr_PI1_integ + I_derror * Delta_T * Ki;
	Curr_PI1_integ = Out_Limit(Curr_PI1_integ, Curr_PI_Limit, (-Curr_PI_Limit));
	Curr_PI1_out = Kp * I_derror + Curr_PI1_integ;
	U_dout = Out_Limit(Curr_PI1_out, Curr_PI_Limit, (-Curr_PI_Limit) );
}

void Current_PI2( float Kp, float Ki )
{
	Curr_PI2_integ = Curr_PI2_integ + I_qerror * Delta_T * Ki;
	Curr_PI2_integ = Out_Limit(Curr_PI2_integ, Curr_PI_Limit, (-Curr_PI_Limit));
	Curr_PI2_out = Kp * I_qerror + Curr_PI2_integ;
	U_qout = Out_Limit(Curr_PI2_out, Curr_PI_Limit, (-Curr_PI_Limit));
}

//稳压环PI控制
void UDC_PI( float Kp, float Ki )
{
	UDC_PI_integ = UDC_PI_integ + UDC_err * Delta_T * Ki;
	UDC_PI_integ = Out_Limit(UDC_PI_integ, UDC_PI_Limit, (-UDC_PI_Limit));
	UDC_PI_out = Kp * UDC_err + UDC_PI_integ;
	Idref = Out_Limit(UDC_PI_out, Idref_Limit,(-Idref_Limit));
}

float ARCTAN(float Ua, float Ub)  // ARCTAN(Ub/Ua)
{
	int temp;
	float temp1;
	float temp2;  //float不能用“==”
	float temp3;
	float theta;
	if ( Ua<0 )
		temp1= -Ua;
	else
		temp1 = Ua;
	if ( Ub < 0 )
		temp2 = -Ub;
	else
		temp2 = Ub;

	//计算tan(theta)值，查表得到角度
	if((Ua==0)&&(Ub==0))
	{
		theta = 0;
	}
	else if((Ua>0)&&(Ub==0))			//0
	{
	   theta=0;
	}
	else if((Ua==0)&&(Ub>0))		//90
	{
	   theta=90;
	}
	else if((Ua<0)&&(Ub==0))		//180
	{
	   theta=180;
	}
	else if((Ua==0)&&(Ub<0))		//270
	{
			theta=270;
	}
	else
	{
	  if(Ua>0)			//1,4象限
	  {
		if(Ub>0)			//1象限
		{
			if(temp1>=temp2)	//0-45
			{
				temp3=256*temp2/temp1;
				temp=(int)(temp3);
				theta=ARCTG_TABLE[temp];												//不璨逯
			}
			else						//45-90
			{
				temp3=256*temp1/temp2;
				temp=(int)(temp3);
				theta=90-ARCTG_TABLE[temp];												//不需插值
			}
		}
		 else							//4象限
		{
			if(temp1>=temp2)	//315-360
			{
				temp3=256*temp2/temp1;
					temp=(int)(temp3);
					theta=360-ARCTG_TABLE[temp];												//不需插值
			}
			else						//270-315
			{
				temp3=256*temp1/temp2;
					temp=(int)(temp3);
					theta=270+ARCTG_TABLE[temp];												//不需插值
			}
		}
	  }
	  else				//2,3象限
	  {
		if(Ub>0)	//2象限
		{
			if(temp1>=temp2)	//135-180
			{
				temp3=256*temp2/temp1;
					temp=(int)(temp3);
					theta=180-ARCTG_TABLE[temp];												//不需插值
			}
			else						//90-135
			{
				temp3=256*temp1/temp2;
					temp=(int)(temp3);
					theta=90+ARCTG_TABLE[temp];												//不需插值
			}
		}
		 else		//3象限
		{
			if(temp1>=temp2)	//180-225
			{
				temp3=256*temp2/temp1;
					temp=(int)(temp3);
					theta=180+ARCTG_TABLE[temp];												//不需插�
			}
			else						//225-270
			{
				temp3=256*temp1/temp2;
					temp=(int)(temp3);
					theta=270-ARCTG_TABLE[temp];												//不需插�
			}
		}
	  }
	}
	return theta;
}

//静止坐标变换
void CON3S2S(float U,float V,float W)
{
	float temp;  //基于功率相等的变换,幅值扩大了sqrt(2/3)倍

	temp = U - ( V + W ) * C_COS60;
	LS1 = temp * C_32COEF;				//A=sqrt(2/3)*(U-(V+W)*cos60)

	temp = ( V - W ) * C_SIN60;
	LS2 = temp * C_32COEF;				//B=sqrt(2/3)*((V-W)*sin60)
}

void CON2S3S( float Ua, float Ub )
{
	LS1 = Ua * C_32COEF;				//A=sqrt(2/3)*Ualpha
	LS2 = (-C_COS60 * Ua + Ub * C_SIN60) * C_32COEF;				//B=sqrt(2/3)*(-cos60*Ualpha+sin60*Ubeta)
	LS3 = (-C_COS60 * Ua - Ub * C_SIN60) * C_32COEF;				//C=sqrt(2/3)*(-cos60*Ualpha-sin60*Ubeta)
}
//旋转坐标变换

//---------------------------------变换-----------------------------------//
void CON2S2R_W( float U_alpha, float U_beta)
{
//  以cos方式锁相
//	坐标变换     cos   sin
//	       -sin   cos

	LS1 =  U_alpha*cosVal + U_beta*sinVal;
	LS2 = -U_alpha*sinVal + U_beta*cosVal;
}

void CON2R2S_W(float Um,float Ut)
{
//  以cos方式锁相
//	坐标变换        cos   -sin
//	         sin   cos

	LS1 = Um * cosVal - Ut * sinVal;
	LS2 = Um * sinVal + Ut * cosVal ;
}

//求sin 和 cos 值
void Sin_Value( float theta )
{
	int temp;
	float AB[2]={0,0};

	while (theta>=360)
		theta-=360;
	while (theta<0)
		theta+=360;

	if(theta==0)				//sin(0)
	{
		AB[0]=0;
		AB[1]=1;
	}
	else if(theta==90)		//sin(90)
	{
		AB[0]=1;
		AB[1]=0;
	}
	else if(theta==180)		//sin(180)
	{
		AB[0]=0;
		AB[1]=-1;
	}
	else if(theta==270)		//sin(270)
	{
		AB[0]=-1;
		AB[1]=0;
	}
	else
	{
		if(theta<90)
		{
			temp=(int)(theta*Theta_DB);		//sin
			AB[0]=SIN_TABLE[temp];
			temp=(int)((90-theta)*Theta_DB);	//cos=sin(90-theta)
			AB[1]=SIN_TABLE[temp];
		}
		else if(theta<180)
		{
			temp=(int)((180-theta)*Theta_DB);		//sin
			AB[0]=SIN_TABLE[temp];
			temp=(int)((theta-90)*Theta_DB);	//cos=sin(90-theta)
			AB[1]=-SIN_TABLE[temp];
		}
		else if(theta<270)
		{
			temp=(int)((theta-180)*Theta_DB);		//sin
			AB[0]=-SIN_TABLE[temp];
			temp=(int)((270-theta)*Theta_DB);	//cos=sin(90-theta)
			AB[1]=-SIN_TABLE[temp];
		}
		else if(theta<360)
		{
			temp=(int)((360-theta)*Theta_DB);		//sin
			AB[0]=-SIN_TABLE[temp];
			temp=(int)((theta-270)*Theta_DB);	//cos=sin(90-theta)
			AB[1]=SIN_TABLE[temp];
		}
	}
	sinVal=AB[0];
	cosVal=AB[1];
}

void P2P_Balance_Neg_U(void)
{
	float UA_THETA_neg;
//	--------------------------负序电压相角--------------------------
	THETA_neg = ARCTAN(  UDC.delt_beta, -UDC.delt_alpha );  //这是正确的求负序相角的公式,对应的Kp_P2P为负数,所以下面U_neg_amp要添负号
	UA_THETA_neg = THETA_C + THETA_neg + THETA_Neg_Com;
	Sin_Value(UA_THETA_neg);
//	--------------------------负序电压幅值--------------------------
	U_neg_amp = -UDC.delta_amp * Ctrl_Para.Kp_P2P_Neg;  //此处添负号详见万的论文和仿真模型分析
	if ( U_neg_amp < U_neg_amp_lim ) //限幅
		U_neg_amp = U_neg_amp_lim;
	UA_P2P = U_neg_amp * sinVal;  //注意是sinVal !!!
	Sin_Value( UA_THETA_neg + 120 );  //负序,B相超前A相120度
	UB_P2P = U_neg_amp * sinVal;  //cosVal不行
	Sin_Value(UA_THETA_neg - 120);
	UC_P2P = U_neg_amp * sinVal;
	return;
}

void P2P_Balance_Z(void)
{
	float THETA_Z;
//	-------------------------- 计算零序电压相角  --------------------------
	THETA_Z = THETA_C + Gama + ARCTAN( UDC.delt_alpha, -UDC.delt_beta );
	Sin_Value(THETA_Z);
//	-------------------------- 计算零序电压幅值  --------------------------
	U_Z_amp =  UDC.delta_amp * Ctrl_Para.Kp_P2P_Z;  //此处取正值,注意！！U_Z_amp在200V平衡时就已经60多
	if ( U_Z_amp > U_Z_amp_lim )
		U_Z_amp = U_Z_amp_lim;
	UA_P2P = U_Z_amp * cosVal; //电网电压是以cos锁相的,所以此处是cos
	UB_P2P = U_Z_amp * cosVal;
	UC_P2P = U_Z_amp * cosVal;
	return;
}

void Local_DC_Process(void)
{
//----------------------------分控电容电压采样-------------------------------//
//分散式控制需要一对一地呼叫应答;集中式控制只需一个全局指令即可。
//读取各module电容电压,对应实际值
	UDC.DC1 = *( DRAM_data + AD_C1_DC_Addr );
	UDC.DC2 = *( DRAM_data + AD_C2_DC_Addr );
	UDC.DC3 = *( DRAM_data + AD_C3_DC_Addr );
	UDC.DC4 = *( DRAM_data + AD_C4_DC_Addr );
	UDC.DC5 = *( DRAM_data + AD_C5_DC_Addr );
	UDC.DC6 = *( DRAM_data + AD_C6_DC_Addr );
	UDC.DC7 = *( DRAM_data + AD_C7_DC_Addr );
	UDC.DC8 = *( DRAM_data + AD_C8_DC_Addr );
	UDC.DC9 = *( DRAM_data + AD_C9_DC_Addr );
	UDC.DC10 = *( DRAM_data + AD_C10_DC_Addr );
	UDC.DC11 = *( DRAM_data + AD_C11_DC_Addr );
	UDC.DC12 = *( DRAM_data + AD_C12_DC_Addr );

	UDC.DB1 = *( DRAM_data + AD_B1_DC_Addr );
	UDC.DB2 = *( DRAM_data + AD_B2_DC_Addr );
	UDC.DB3 = *( DRAM_data + AD_B3_DC_Addr );
	UDC.DB4 = *( DRAM_data + AD_B4_DC_Addr );
	UDC.DB5 = *( DRAM_data + AD_B5_DC_Addr );
	UDC.DB6 = *( DRAM_data + AD_B6_DC_Addr );
	UDC.DB7 = *( DRAM_data + AD_B7_DC_Addr );
	UDC.DB8 = *( DRAM_data + AD_B8_DC_Addr );
	UDC.DB9 = *( DRAM_data + AD_B9_DC_Addr );
	UDC.DB10 = *( DRAM_data + AD_B10_DC_Addr );
	UDC.DB11 = *( DRAM_data + AD_B11_DC_Addr );
	UDC.DB12 = *( DRAM_data + AD_B12_DC_Addr );

	UDC.DA1 = *( DRAM_data + AD_A1_DC_Addr );
	UDC.DA2 = *( DRAM_data + AD_A2_DC_Addr );
	UDC.DA3 = *( DRAM_data + AD_A3_DC_Addr );
	UDC.DA4 = *( DRAM_data + AD_A4_DC_Addr );
	UDC.DA5 = *( DRAM_data + AD_A5_DC_Addr );
	UDC.DA6 = *( DRAM_data + AD_A6_DC_Addr );
	UDC.DA7 = *( DRAM_data + AD_A7_DC_Addr );
	UDC.DA8 = *( DRAM_data + AD_A8_DC_Addr );
	UDC.DA9 = *( DRAM_data + AD_A9_DC_Addr );
	UDC.DA10 = *( DRAM_data + AD_A10_DC_Addr );
	UDC.DA11 = *( DRAM_data + AD_A11_DC_Addr );
	UDC.DA12 = *( DRAM_data + AD_A12_DC_Addr );

	if(Run_Stop_Flag == 1)
	{
		if(UDC.DA1>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.A1 = UDC.DA1;
			ERROR_module |= 0x1001;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA2>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.A2 = UDC.DA2;
			ERROR_module |= 0x1002;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA3>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.A3 = UDC.DA3;
			ERROR_module |= 0x1004;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA4>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.A4 = UDC.DA4;
			ERROR_module |= 0x1008;
			Run_Stop_Flag = 0;

		}
		if(UDC.DA5>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.A5 = UDC.DA5;
			ERROR_module |= 0x1010;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA6>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.A6 = UDC.DA6;
			ERROR_module |= 0x1020;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA7>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x1040;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA8>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x1080;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA9>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x1100;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA10>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x1200;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA11>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x1400;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA12>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x1800;
			Run_Stop_Flag = 0;
		}

		if(UDC.DB1>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.B1 = UDC.DB1;
			ERROR_module |= 0x2001;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB2>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.B2 = UDC.DB2;
			ERROR_module |= 0x2002;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB3>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.B3 = UDC.DB3;
			ERROR_module |= 0x2004;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB4>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.B4 = UDC.DB4;
			ERROR_module |= 0x2008;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB5>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.B5 = UDC.DB5;
			ERROR_module |= 0x2010;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB6>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.B6 = UDC.DB6;
			ERROR_module |= 0x2020;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB7>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x2040;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB8>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x2080;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB9>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x2100;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB10>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x2200;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB11>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x2400;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB12>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x2800;
			Run_Stop_Flag = 0;
		}

		if(UDC.DC1>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.C1 = UDC.DC1;
			ERROR_module |= 0x4001;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC2>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.C2 = UDC.DC2;
			ERROR_module |= 0x4002;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC3>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.C3 = UDC.DC3;
			ERROR_module |= 0x4004;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC4>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.C4 = UDC.DC4;
			ERROR_module |= 0x4008;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC5>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.C5 = UDC.DC5;
			ERROR_module |= 0x4010;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC6>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;  //F_State.C6 = UDC.DC6;
			ERROR_module |= 0x4020;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC7>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x4040;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC8>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x4080;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC9>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x4100;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC10>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x4200;
			Run_Stop_Flag = 0;

		}
		if(UDC.DC11>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x4400;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC12>Ctrl_Para.UDCMAX)
		{
			ERROR_flag |= LocalOverMAX;
			ERROR_module |= 0x4800;
			Run_Stop_Flag = 0;
		}

		if(UDC.DA1<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.A1 = UDC.DA1;
			ERROR_module |= 0x1001;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA2<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.A2 = UDC.DA2;
			ERROR_module |= 0x1002;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA3<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.A3 = UDC.DA3;
			ERROR_module |= 0x1004;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA4<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.A4 = UDC.DA4;
			ERROR_module |= 0x1008;
			Run_Stop_Flag = 0;

		}
		if(UDC.DA5<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.A5 = UDC.DA5;
			ERROR_module |= 0x1010;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA6<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.A6 = UDC.DA6;
			ERROR_module |= 0x1020;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA7<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x1040;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA8<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x1080;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA9<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x1100;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA10<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x1200;
			Run_Stop_Flag = 0;

		}
		if(UDC.DA11<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x1400;
			Run_Stop_Flag = 0;
		}
		if(UDC.DA12<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x1800;
			Run_Stop_Flag = 0;
		}

		if(UDC.DB1<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.B1 = UDC.DB1;
			ERROR_module |= 0x2001;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB2<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.B2 = UDC.DB2;
			ERROR_module |= 0x2002;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB3<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.B3 = UDC.DB3;
			ERROR_module |= 0x2004;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB4<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.B4 = UDC.DB4;
			ERROR_module |= 0x2008;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB5<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.B5 = UDC.DB5;
			ERROR_module |= 0x2010;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB6<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.B6 = UDC.DB6;
			ERROR_module |= 0x2020;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB7<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x2040;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB8<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x2080;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB9<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x2100;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB10<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x2200;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB11<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x2400;
			Run_Stop_Flag = 0;
		}
		if(UDC.DB12<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x2800;
			Run_Stop_Flag = 0;
		}

		if(UDC.DC1<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.C1 = UDC.DC1;
			ERROR_module |= 0x4001;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC2<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.C2 = UDC.DC2;
			ERROR_module |= 0x4002;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC3<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.C3 = UDC.DC3;
			ERROR_module |= 0x4004;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC4<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.C4 = UDC.DC4;
			ERROR_module |= 0x4008;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC5<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.C5 = UDC.DC5;
			ERROR_module |= 0x4010;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC6<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN; //F_State.C6 = UDC.DC6;
			ERROR_module |= 0x4020;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC7<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x4040;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC8<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x4080;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC9<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x4100;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC10<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x4200;
			Run_Stop_Flag = 0;

		}
		if(UDC.DC11<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x4400;
			Run_Stop_Flag = 0;
		}
		if(UDC.DC12<Ctrl_Para.UDCMIN)
		{
			ERROR_flag |= LocalUnderMIN;
			ERROR_module |= 0x4800;
			Run_Stop_Flag = 0;
		}
	}
//直流母线电压在分控改进后无需定标，只用滤波,Ts_UDC为带宽,50Hz,滤除电容上的二倍频
	switch(CpuTimer0Count)
	{
	case 1:
	case 7:
	case 13:
	case 19:
	{
//		UDC.DA1 = *( DRAM_data + AD_A1_DC_Addr );
//		UDC.DA2 = *( DRAM_data + AD_A2_DC_Addr );
//		UDC.DB1 = *( DRAM_data + AD_B1_DC_Addr );
//		UDC.DB2 = *( DRAM_data + AD_B2_DC_Addr );
//		UDC.DC1 = *( DRAM_data + AD_C1_DC_Addr );
//		UDC.DC2 = *( DRAM_data + AD_C2_DC_Addr );
//
//		if(Run_Stop_Flag == 1)
//		{
//			if(UDC.DA1>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.A1 = UDC.DA1;
//				ERROR_module |= 0x1001;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA2>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.A2 = UDC.DA2;
//				ERROR_module |= 0x1002;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB1>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.B1 = UDC.DB1;
//				ERROR_module |= 0x2001;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB2>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.B2 = UDC.DB2;
//				ERROR_module |= 0x2002;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC1>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.C1 = UDC.DC1;
//				ERROR_module |= 0x4001;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC2>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.C2 = UDC.DC2;
//				ERROR_module |= 0x4002;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA1<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.A1 = UDC.DA1;
//				ERROR_module |= 0x1001;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA2<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.A2 = UDC.DA2;
//				ERROR_module |= 0x1002;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB1<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.B1 = UDC.DB1;
//				ERROR_module |= 0x2001;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB2<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.B2 = UDC.DB2;
//				ERROR_module |= 0x2002;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC1<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.C1 = UDC.DC1;
//				ERROR_module |= 0x4001;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC2<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.C2 = UDC.DC2;
//				ERROR_module |= 0x4002;
//				Run_Stop_Flag = 0;
//			}
//		}
		UDC_AR1 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR1 + UDC.DA1 * Delta_Tdc * Ts_UDC; 	 //差分方程，实现滤波
		UDC_BR1 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR1 + UDC.DB1 * Delta_Tdc * Ts_UDC;
		UDC_CR1 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR1 + UDC.DC1 * Delta_Tdc * Ts_UDC;
		UDC_AS1 = filter_Band_Stop(UDC_AR1, filter_band_stop_A1);
		UDC_BS1 = filter_Band_Stop(UDC_BR1, filter_band_stop_B1);
		UDC_CS1 = filter_Band_Stop(UDC_CR1, filter_band_stop_C1);

		UDC_AR2 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR2 + UDC.DA2 * Delta_Tdc * Ts_UDC;
		UDC_BR2 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR2 + UDC.DB2 * Delta_Tdc * Ts_UDC;
		UDC_CR2 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR2 + UDC.DC2 * Delta_Tdc * Ts_UDC;
		UDC_AS2 = filter_Band_Stop(UDC_AR2, filter_band_stop_A2);
		UDC_BS2 = filter_Band_Stop(UDC_BR2, filter_band_stop_B2);
		UDC_CS2 = filter_Band_Stop(UDC_CR2, filter_band_stop_C2);
		break;
	}
	case 2:
	case 8:
	case 14:
	case 20:
	{
//		UDC.DA3 = *( DRAM_data + AD_A3_DC_Addr );
//		UDC.DA4 = *( DRAM_data + AD_A4_DC_Addr );
//		UDC.DB3 = *( DRAM_data + AD_B3_DC_Addr );
//		UDC.DB4 = *( DRAM_data + AD_B4_DC_Addr );
//		UDC.DC3 = *( DRAM_data + AD_C3_DC_Addr );
//		UDC.DC4 = *( DRAM_data + AD_C4_DC_Addr );
//		if(Run_Stop_Flag == 1)
//		{
//			if(UDC.DA3>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.A3 = UDC.DA3;
//				ERROR_module |= 0x1004;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA4>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.A4 = UDC.DA4;
//				ERROR_module |= 0x1008;
//				Run_Stop_Flag = 0;
//
//			}
//			if(UDC.DB3>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.B3 = UDC.DB3;
//				ERROR_module |= 0x2004;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB4>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.B4 = UDC.DB4;
//				ERROR_module |= 0x2008;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC3>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.C3 = UDC.DC3;
//				ERROR_module |= 0x4004;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC4>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.C4 = UDC.DC4;
//				ERROR_module |= 0x4008;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA3<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.A3 = UDC.DA3;
//				ERROR_module |= 0x1004;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA4<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.A4 = UDC.DA4;
//				ERROR_module |= 0x1008;
//				Run_Stop_Flag = 0;
//
//			}
//			if(UDC.DB3<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.B3 = UDC.DB3;
//				ERROR_module |= 0x2004;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB4<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.B4 = UDC.DB4;
//				ERROR_module |= 0x2008;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC3<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.C3 = UDC.DC3;
//				ERROR_module |= 0x4004;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC4<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.C4 = UDC.DC4;
//				ERROR_module |= 0x4008;
//				Run_Stop_Flag = 0;
//			}
//		}

		UDC_AR3 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR3 + UDC.DA3 * Delta_Tdc * Ts_UDC;
		UDC_BR3 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR3 + UDC.DB3 * Delta_Tdc * Ts_UDC;
		UDC_CR3 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR3 + UDC.DC3 * Delta_Tdc * Ts_UDC;
		UDC_AS3 = filter_Band_Stop(UDC_AR3, filter_band_stop_A3);
		UDC_BS3 = filter_Band_Stop(UDC_BR3, filter_band_stop_B3);
		UDC_CS3 = filter_Band_Stop(UDC_CR3, filter_band_stop_C3);

		UDC_AR4 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR4 + UDC.DA4 * Delta_Tdc * Ts_UDC;
		UDC_BR4 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR4 + UDC.DB4 * Delta_Tdc * Ts_UDC;
		UDC_CR4 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR4 + UDC.DC4 * Delta_Tdc * Ts_UDC;
		UDC_AS4 = filter_Band_Stop(UDC_AR4, filter_band_stop_A4);
		UDC_BS4 = filter_Band_Stop(UDC_BR4, filter_band_stop_B4);
		UDC_CS4 = filter_Band_Stop(UDC_CR4, filter_band_stop_C4);
		break;
	}
	case 3:
	case 9:
	case 15:
	case 21:
	{
//		UDC.DA5 = *( DRAM_data + AD_A5_DC_Addr );
//		UDC.DA6 = *( DRAM_data + AD_A6_DC_Addr );
//		UDC.DB5 = *( DRAM_data + AD_B5_DC_Addr );
//		UDC.DB6 = *( DRAM_data + AD_B6_DC_Addr );
//		UDC.DC5 = *( DRAM_data + AD_C5_DC_Addr );
//		UDC.DC6 = *( DRAM_data + AD_C6_DC_Addr );
//		if(Run_Stop_Flag == 1)
//		{
//			if(UDC.DA5>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.A5 = UDC.DA5;
//				ERROR_module |= 0x1010;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA6>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.A6 = UDC.DA6;
//				ERROR_module |= 0x1020;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB5>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.B5 = UDC.DB5;
//				ERROR_module |= 0x2010;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB6>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.B6 = UDC.DB6;
//				ERROR_module |= 0x2020;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC5>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.C5 = UDC.DC5;
//				ERROR_module |= 0x4010;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC6>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;  //F_State.C6 = UDC.DC6;
//				ERROR_module |= 0x4020;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA5<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.A5 = UDC.DA5;
//				ERROR_module |= 0x1010;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA6<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.A6 = UDC.DA6;
//				ERROR_module |= 0x1020;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB5<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.B5 = UDC.DB5;
//				ERROR_module |= 0x2010;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB6<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.B6 = UDC.DB6;
//				ERROR_module |= 0x2020;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC5<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.C5 = UDC.DC5;
//				ERROR_module |= 0x4010;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC6<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN; //F_State.C6 = UDC.DC6;
//				ERROR_module |= 0x4020;
//				Run_Stop_Flag = 0;
//			}
//		}

		UDC_AR5 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR5 + UDC.DA5 * Delta_Tdc * Ts_UDC;
		UDC_BR5 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR5 + UDC.DB5 * Delta_Tdc * Ts_UDC;
		UDC_CR5 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR5 + UDC.DC5 * Delta_Tdc * Ts_UDC;
		UDC_AS5 = filter_Band_Stop(UDC_AR5, filter_band_stop_A5);
		UDC_BS5 = filter_Band_Stop(UDC_BR5, filter_band_stop_B5);
		UDC_CS5 = filter_Band_Stop(UDC_CR5, filter_band_stop_C5);

		UDC_AR6 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR6 + UDC.DA6 * Delta_Tdc * Ts_UDC;
		UDC_BR6 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR6 + UDC.DB6 * Delta_Tdc * Ts_UDC;
		UDC_CR6 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR6 + UDC.DC6 * Delta_Tdc * Ts_UDC;
		UDC_AS6 = filter_Band_Stop(UDC_AR6, filter_band_stop_A6);
		UDC_BS6 = filter_Band_Stop(UDC_BR6, filter_band_stop_B6);
		UDC_CS6 = filter_Band_Stop(UDC_CR6, filter_band_stop_C6);
		break;
	}
	case 4:
	case 10:
	case 16:
	case 22:
	{
//		UDC.DA7 = *( DRAM_data + AD_A7_DC_Addr );
//		UDC.DA8 = *( DRAM_data + AD_A8_DC_Addr );
//		UDC.DB7 = *( DRAM_data + AD_B7_DC_Addr );
//		UDC.DB8 = *( DRAM_data + AD_B8_DC_Addr );
//		UDC.DC7 = *( DRAM_data + AD_C7_DC_Addr );
//		UDC.DC8 = *( DRAM_data + AD_C8_DC_Addr );
//		if(Run_Stop_Flag == 1)
//		{
//			if(UDC.DA7>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x1040;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA8>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x1080;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB7>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x2040;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB8>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x2080;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC7>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x4040;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC8>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x4080;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA7<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x1040;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA8<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x1080;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB7<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x2040;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB8<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x2080;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC7<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x4040;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC8<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x4080;
//				Run_Stop_Flag = 0;
//			}
//		}

		UDC_AR7 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR7 + UDC.DA7 * Delta_Tdc * Ts_UDC;
		UDC_BR7 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR7 + UDC.DB7 * Delta_Tdc * Ts_UDC;
		UDC_CR7 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR7 + UDC.DC7 * Delta_Tdc * Ts_UDC;
		UDC_AS7 = filter_Band_Stop(UDC_AR7, filter_band_stop_A7);
		UDC_BS7 = filter_Band_Stop(UDC_BR7, filter_band_stop_B7);
		UDC_CS7 = filter_Band_Stop(UDC_CR7, filter_band_stop_C7);

		UDC_AR8 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR8 + UDC.DA8 * Delta_Tdc * Ts_UDC;
		UDC_BR8 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR8 + UDC.DB8 * Delta_Tdc * Ts_UDC;
		UDC_CR8 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR8 + UDC.DC8 * Delta_Tdc * Ts_UDC;
		UDC_AS8 = filter_Band_Stop(UDC_AR8, filter_band_stop_A8);
		UDC_BS8 = filter_Band_Stop(UDC_BR8, filter_band_stop_B8);
		UDC_CS8 = filter_Band_Stop(UDC_CR8, filter_band_stop_C8);
		break;
	}
	case 5:
	case 11:
	case 17:
	case 23:
	{
//		UDC.DC9 = *( DRAM_data + AD_C9_DC_Addr );
//		UDC.DC10 = *( DRAM_data + AD_C10_DC_Addr );
//		UDC.DB9 = *( DRAM_data + AD_B9_DC_Addr );
//		UDC.DB10 = *( DRAM_data + AD_B10_DC_Addr );
//		UDC.DA9 = *( DRAM_data + AD_A9_DC_Addr );
//		UDC.DA10 = *( DRAM_data + AD_A10_DC_Addr );
//		if(Run_Stop_Flag == 1)
//		{
//			if(UDC.DA9>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x1100;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA10>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x1200;
//				Run_Stop_Flag = 0;
//
//			}
//			if(UDC.DB9>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x2100;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB10>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x2200;
//				Run_Stop_Flag = 0;
//
//			}
//			if(UDC.DC9>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x4100;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC10>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x4200;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA9<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x1100;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA10<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x1200;
//				Run_Stop_Flag = 0;
//
//			}
//			if(UDC.DB9<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x2100;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB10<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x2200;
//				Run_Stop_Flag = 0;
//
//			}
//			if(UDC.DC9<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x4100;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC10<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x4200;
//				Run_Stop_Flag = 0;
//
//			}
//		}

		UDC_AR9 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR9 + UDC.DA9 * Delta_Tdc * Ts_UDC;
		UDC_BR9 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR9 + UDC.DB9 * Delta_Tdc * Ts_UDC;
		UDC_CR9 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR9 + UDC.DC9 * Delta_Tdc * Ts_UDC;
		UDC_AS9 = filter_Band_Stop(UDC_AR9, filter_band_stop_A9);
		UDC_BS9 = filter_Band_Stop(UDC_BR9, filter_band_stop_B9);
		UDC_CS9 = filter_Band_Stop(UDC_CR9, filter_band_stop_C9);

		UDC_AR10 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR10 + UDC.DA10 * Delta_Tdc * Ts_UDC;
		UDC_BR10 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR10 + UDC.DB10 * Delta_Tdc * Ts_UDC;
		UDC_CR10 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR10 + UDC.DC10 * Delta_Tdc * Ts_UDC;
		UDC_AS10 = filter_Band_Stop(UDC_AR10, filter_band_stop_A10);
		UDC_BS10 = filter_Band_Stop(UDC_BR10, filter_band_stop_B10);
		UDC_CS10 = filter_Band_Stop(UDC_CR10, filter_band_stop_C10);
		break;
	}
	case 6:
	case 12:
	case 18:
	case 24:
	{
//		UDC.DA11 = *( DRAM_data + AD_A11_DC_Addr );
//		UDC.DA12 = *( DRAM_data + AD_A12_DC_Addr );
//		UDC.DB11 = *( DRAM_data + AD_B11_DC_Addr );
//		UDC.DB12 = *( DRAM_data + AD_B12_DC_Addr );
//		UDC.DC11 = *( DRAM_data + AD_C11_DC_Addr );
//		UDC.DC12 = *( DRAM_data + AD_C12_DC_Addr );
//		if(Run_Stop_Flag == 1)
//		{
//			if(UDC.DA11>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x1400;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA12>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x1800;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB11>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x2400;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB12>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x2800;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC11>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x4400;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC12>Ctrl_Para.UDCMAX)
//			{
//				ERROR_flag |= LocalOverMAX;
//				ERROR_module |= 0x4800;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA11<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x1400;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DA12<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x1800;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB11<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x2400;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DB12<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x2800;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC11<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x4400;
//				Run_Stop_Flag = 0;
//			}
//			if(UDC.DC12<Ctrl_Para.UDCMIN)
//			{
//				ERROR_flag |= LocalUnderMIN;
//				ERROR_module |= 0x4800;
//				Run_Stop_Flag = 0;
//			}
//		}

		UDC_AR11 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR11 + UDC.DA11 * Delta_Tdc * Ts_UDC;
		UDC_BR11 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR11 + UDC.DB11 * Delta_Tdc * Ts_UDC;
		UDC_CR11 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR11 + UDC.DC11 * Delta_Tdc * Ts_UDC;
		UDC_AS11 = filter_Band_Stop(UDC_AR11, filter_band_stop_A11);
		UDC_BS11 = filter_Band_Stop(UDC_BR11, filter_band_stop_B11);
		UDC_CS11 = filter_Band_Stop(UDC_CR11, filter_band_stop_C11);

		UDC_AR12 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_AR12 + UDC.DA12 * Delta_Tdc * Ts_UDC;
		UDC_BR12 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_BR12 + UDC.DB12 * Delta_Tdc * Ts_UDC;
		UDC_CR12 = ( 1- Delta_Tdc * Ts_UDC ) * UDC_CR12 + UDC.DC12 * Delta_Tdc * Ts_UDC;
		UDC_AS12 = filter_Band_Stop(UDC_AR12, filter_band_stop_A12);
		UDC_BS12 = filter_Band_Stop(UDC_BR12, filter_band_stop_B12);
		UDC_CS12 = filter_Band_Stop(UDC_CR12, filter_band_stop_C12);
		break;
	}
	default:{ break; }
	}
		return;
}

void Phase_Lock(void)
{
	if(PLL_Switch == 0)
	{
	//利用Usq=0的PLL方法求电网电压相角
		Sin_Value(Omiga_PLL);
		CON2S2R_W(U_alpha_open_pre, U_beta_open_pre);
		Usd_PLL = SQRT2_3 * LS1; //此处应之前采用功率不变导致电压值放大了sqrt(3/2)倍,故在此处缩小
		Usq_PLL = SQRT2_3 * LS2;
		integral_PLL = integral_PLL + Usq_PLL * Delta_T; //历史积分值，稳定时约为81.8, 若为(-Usq),则锁相值相差180°
		Omiga = Kp_PLL * Usq_PLL + Ki_PLL * integral_PLL ; //P + I; 90 + 220
		Omiga_PLL = Omiga_PLL + Omiga * Delta_T;  //角速度的积分
		while(Omiga_PLL < 0)	Omiga_PLL += 360;
		while(Omiga_PLL >= 360) Omiga_PLL -= 360;
		THETA_C = Omiga_PLL + THETA_COM_Usq; //THETA_C为电网电压相角  + THETA_COM_Usq
		if(THETA_C >= 360)
			THETA_C = THETA_C - 360;
	}
	else if(PLL_Switch == 1)
	{//SOGI-FLL正负序电压分离，锁正序A相的相位
		FLL_err1 = (U_alpha_open_pre*0.5 - Vd1);
		Vd1 = Vd1 + (FLL_err1*1.414 - Vq1) * Omiga_FLL * Delta_T;
		integral_Vd1 = integral_Vd1 + Vd1 * Delta_T;
		Vq1 = integral_Vd1 * Omiga_FLL;

		FLL_err2 = (U_beta_open_pre*0.5 - Vd2);
		Vd2 = Vd2 + (FLL_err2*1.414 - Vq2) * Omiga_FLL * Delta_T;
		integral_Vd2 = integral_Vd2 + Vd2 * Delta_T;
		Vq2 = integral_Vd2 * Omiga_FLL;

		integral_Omiga = integral_Omiga - 50*FLL_err1*Vq1*Omiga_FLL*Delta_T/((Vd1*Vd1+Vq1*Vq1)*1.414);
		Omiga_FLL = integral_Omiga + 314.159;

		U_alpha_Pos = Vd1 - Vq2;
		U_beta_Pos  = Vq1 + Vd2;
		U_alpha_Neg = Vd1 + Vq2;
		U_beta_Neg  = Vd2 - Vq1;

		THETA_C = ARCTAN(U_alpha_Pos, U_beta_Pos);
		while(THETA_C >= 360)
			THETA_C = THETA_C - 360;
		while(THETA_C < 0)
			THETA_C = THETA_C + 360;

		THETA_C += THETA_COM_Usq;
		if(THETA_C >= 360)
			THETA_C = THETA_C - 360;
	}
	return;
//-----------------------------PLL锁相结束---------------------------------//
}

float filter_2nd_dq(const float In, float* buffer)
{
    double curr_out = 0;
    double curr_in = In;
////--------------------------1000HZ二阶滤波器系数160us----------------------//
//	static double znum_1000[3] = {0.145324, 0.290648, 0.145324};  //滤波器分子系数
//	static double zden_1000[2] = {0.252325, -0.671029};  //滤波器分母系数
//
////--------------------------1000HZ:采样周期80us-----------------------------//
//	static double znum_500[3] = {4.61318e-2, 9.22636e-2, 4.61318e-2};
//	static double zden_500[2] = {0.491812, -1.30729};
////-------------------------------dq轴电流二阶滤波器系数---------------------//
	static double znum_dq[3] = {4.61318e-2, 9.22636e-2, 4.61318e-2};  //1000Hz
	static double zden_dq[2] = {0.491812, -1.30729};

    curr_out = znum_dq[2] * curr_in + znum_dq[1] * buffer[1] + znum_dq[0] * buffer[0];
    curr_out -= (zden_dq[1] * buffer[3] + zden_dq[0] * buffer[2]);
    buffer[0] = buffer[1];  //X(n-2)
    buffer[1] = curr_in;  //X(n-1)
    buffer[2] = buffer[3]; //y(n-2)
    buffer[3] = curr_out; //y(n-1)

    return curr_out;
}

float filter_Band_Stop(const float In, float* buffer)
{
    double curr_out = 0;
    double curr_in = In;
//---------------------------直流母线电压 2阶陷波器系数-------------------------//
	static double znum_dq[3] = {0.9016973, -1.7221373, 0.9016973};  //采样周期480us(2083Hz)，中心频率100Hz，滤波器设置截止频率70Hz、142Hz
	static double zden_dq[2] = {0.8033945, -1.7221373};

    curr_out = znum_dq[2] * curr_in + znum_dq[1] * buffer[1] + znum_dq[0] * buffer[0];
    curr_out -= (zden_dq[1] * buffer[3] + zden_dq[0] * buffer[2]);
    buffer[0] = buffer[1];  //X(n-2)
    buffer[1] = curr_in;  //X(n-1)
    buffer[2] = buffer[3]; //y(n-2)
    buffer[3] = curr_out; //y(n-1)

    return curr_out;
}

//float Repetive_PI1( float Kp, float Ki )
//{
//	Rpt_PI1_integ = Rpt_PI1_integ + Repetive_d * Delta_T * Ki;
//	Rpt_PI1_integ = Out_Limit(Rpt_PI1_integ, Curr_PI_integ_uplim, Curr_PI_integ_lowlim);
//	Rpt_PI1_out = Kp * Repetive_d + Rpt_PI1_integ;
//	return Out_Limit(Rpt_PI1_out, Curr_PI_out_uplim, Curr_PI_out_lowlim);
//}
//
//float Repetive_PI2( float Kp, float Ki )
//{
//	Rpt_PI2_integ = Rpt_PI2_integ + Repetive_q * Delta_T * Ki;
//	Rpt_PI2_integ = Out_Limit(Rpt_PI2_integ, Curr_PI_integ_uplim, Curr_PI_integ_lowlim);
//	Rpt_PI2_out = Kp * Repetive_q + Rpt_PI2_integ;
//	return Out_Limit(Rpt_PI2_out, Curr_PI_out_uplim, Curr_PI_out_lowlim);
//}

void DataCheck(void)
{
//启动数据收发正确性检测
	D_Check.Data = D_Check.Data + 1;
//	*( DRAM_data + Datas_Check ) = D_Check.Data;  //全局检查数据链路
//分别检查数据链路
	*( DRAM_data + Send_Check_A1_Addr ) = D_Check.Data + 1;
	*( DRAM_data + Send_Check_A2_Addr ) = D_Check.Data + 2;
	*( DRAM_data + Send_Check_A3_Addr ) = D_Check.Data + 3;
	*( DRAM_data + Send_Check_A4_Addr ) = D_Check.Data + 4;
	*( DRAM_data + Send_Check_A5_Addr ) = D_Check.Data + 5;
	*( DRAM_data + Send_Check_A6_Addr ) = D_Check.Data + 6;
	*( DRAM_data + Send_Check_A7_Addr ) = D_Check.Data + 7;
	*( DRAM_data + Send_Check_A8_Addr ) = D_Check.Data + 8;
	*( DRAM_data + Send_Check_A9_Addr ) = D_Check.Data + 9;
	*( DRAM_data + Send_Check_A10_Addr) = D_Check.Data + 10;
	*( DRAM_data + Send_Check_A11_Addr) = D_Check.Data + 11;
	*( DRAM_data + Send_Check_A12_Addr) = D_Check.Data + 12;

	*( DRAM_data + Send_Check_B1_Addr ) = D_Check.Data + 31;
	*( DRAM_data + Send_Check_B2_Addr ) = D_Check.Data + 32;
	*( DRAM_data + Send_Check_B3_Addr ) = D_Check.Data + 33;
	*( DRAM_data + Send_Check_B4_Addr ) = D_Check.Data + 34;
	*( DRAM_data + Send_Check_B5_Addr ) = D_Check.Data + 35;
	*( DRAM_data + Send_Check_B6_Addr ) = D_Check.Data + 36;
	*( DRAM_data + Send_Check_B7_Addr ) = D_Check.Data + 37;
	*( DRAM_data + Send_Check_B8_Addr ) = D_Check.Data + 38;
	*( DRAM_data + Send_Check_B9_Addr ) = D_Check.Data + 39;
	*( DRAM_data + Send_Check_B10_Addr) = D_Check.Data + 40;
	*( DRAM_data + Send_Check_B11_Addr) = D_Check.Data + 41;
	*( DRAM_data + Send_Check_B12_Addr) = D_Check.Data + 42;

	*( DRAM_data + Send_Check_C1_Addr ) = D_Check.Data + 61;
	*( DRAM_data + Send_Check_C2_Addr ) = D_Check.Data + 62;
	*( DRAM_data + Send_Check_C3_Addr ) = D_Check.Data + 63;
	*( DRAM_data + Send_Check_C4_Addr ) = D_Check.Data + 64;
	*( DRAM_data + Send_Check_C5_Addr ) = D_Check.Data + 65;
	*( DRAM_data + Send_Check_C6_Addr ) = D_Check.Data + 66;
	*( DRAM_data + Send_Check_C7_Addr ) = D_Check.Data + 67;
	*( DRAM_data + Send_Check_C8_Addr ) = D_Check.Data + 68;
	*( DRAM_data + Send_Check_C9_Addr ) = D_Check.Data + 69;
	*( DRAM_data + Send_Check_C10_Addr) = D_Check.Data + 70;
	*( DRAM_data + Send_Check_C11_Addr) = D_Check.Data + 71;
	*( DRAM_data + Send_Check_C12_Addr) = D_Check.Data + 72;

	delay_t(400);	//写命令和读指令之间需要加延时，写命令之间因为有FIFO，不用加延时

	D_Check.A1 = *( DRAM_data + Data_Check_A1_Addr );
	D_Check.A2 = *( DRAM_data + Data_Check_A2_Addr );
	D_Check.A3 = *( DRAM_data + Data_Check_A3_Addr );
	D_Check.A4 = *( DRAM_data + Data_Check_A4_Addr );
	D_Check.A5 = *( DRAM_data + Data_Check_A5_Addr );
	D_Check.A6 = *( DRAM_data + Data_Check_A6_Addr );
	D_Check.A7 = *( DRAM_data + Data_Check_A7_Addr );
	D_Check.A8 = *( DRAM_data + Data_Check_A8_Addr );
	D_Check.A9 = *( DRAM_data + Data_Check_A9_Addr );
	D_Check.A10= *( DRAM_data + Data_Check_A10_Addr );
	D_Check.A11= *( DRAM_data + Data_Check_A11_Addr );
	D_Check.A12= *( DRAM_data + Data_Check_A12_Addr );

	D_Check.B1 = *( DRAM_data + Data_Check_B1_Addr );
	D_Check.B2 = *( DRAM_data + Data_Check_B2_Addr );
	D_Check.B3 = *( DRAM_data + Data_Check_B3_Addr );
	D_Check.B4 = *( DRAM_data + Data_Check_B4_Addr );
	D_Check.B5 = *( DRAM_data + Data_Check_B5_Addr );
	D_Check.B6 = *( DRAM_data + Data_Check_B6_Addr );
	D_Check.B7 = *( DRAM_data + Data_Check_B7_Addr );
	D_Check.B8 = *( DRAM_data + Data_Check_B8_Addr );
	D_Check.B9 = *( DRAM_data + Data_Check_B9_Addr );
	D_Check.B10= *( DRAM_data + Data_Check_B10_Addr );
	D_Check.B11= *( DRAM_data + Data_Check_B11_Addr );
	D_Check.B12= *( DRAM_data + Data_Check_B12_Addr );

	D_Check.C1 = *( DRAM_data + Data_Check_C1_Addr );
	D_Check.C2 = *( DRAM_data + Data_Check_C2_Addr );
	D_Check.C3 = *( DRAM_data + Data_Check_C3_Addr );
	D_Check.C4 = *( DRAM_data + Data_Check_C4_Addr );
	D_Check.C5 = *( DRAM_data + Data_Check_C5_Addr );
	D_Check.C6 = *( DRAM_data + Data_Check_C6_Addr );
	D_Check.C7 = *( DRAM_data + Data_Check_C7_Addr );
	D_Check.C8 = *( DRAM_data + Data_Check_C8_Addr );
	D_Check.C9 = *( DRAM_data + Data_Check_C9_Addr );
	D_Check.C10= *( DRAM_data + Data_Check_C10_Addr );
	D_Check.C11= *( DRAM_data + Data_Check_C11_Addr );
	D_Check.C12= *( DRAM_data + Data_Check_C12_Addr );

//	if((D_Check.A1 != D_Check.Data)||(D_Check.A2 != D_Check.Data)||(D_Check.A3 != D_Check.Data)
//		||(D_Check.B1 != D_Check.Data)||(D_Check.B2 != D_Check.Data)||(D_Check.B3 != D_Check.Data)
//		||(D_Check.C1 != D_Check.Data)||(D_Check.C2 != D_Check.Data)||(D_Check.C3 != D_Check.Data))
//	{
//		StopCpuTimer0();
//		Run_Stop_Flag = 0;  //某一节点失联了
//		ERROR_flag |= Node_Lost;  //失联
//	}
	return;
}

void Read_Fault_State(void)
{
	*( DRAM_data +  Req_State) = 100;  //集中式控制只需发一个全局指令即可
	delay_t(320);  //等待分控完成数据上传

	F_State.A1 = *( DRAM_data +  State_A1_Addr);
	F_State.A2 = *( DRAM_data +  State_A2_Addr);
	F_State.A3 = *( DRAM_data +  State_A3_Addr);
	F_State.A4 = *( DRAM_data +  State_A4_Addr);
	F_State.A5 = *( DRAM_data +  State_A5_Addr);
	F_State.A6 = *( DRAM_data +  State_A6_Addr);
	F_State.A7 = *( DRAM_data +  State_A7_Addr);
	F_State.A8 = *( DRAM_data +  State_A8_Addr);
	F_State.A9 = *( DRAM_data +  State_A9_Addr);
	F_State.A10 = *( DRAM_data + State_A10_Addr);
	F_State.A11 = *( DRAM_data + State_A11_Addr);
	F_State.A12 = *( DRAM_data + State_A12_Addr);

	F_State.B1 = *( DRAM_data +  State_B1_Addr);
	F_State.B2 = *( DRAM_data +  State_B2_Addr);
	F_State.B3 = *( DRAM_data +  State_B3_Addr);
	F_State.B4 = *( DRAM_data +  State_B4_Addr);
	F_State.B5 = *( DRAM_data +  State_B5_Addr);
	F_State.B6 = *( DRAM_data +  State_B6_Addr);
	F_State.B7 = *( DRAM_data +  State_B7_Addr);
	F_State.B8 = *( DRAM_data +  State_B8_Addr);
	F_State.B9 = *( DRAM_data +  State_B9_Addr);
	F_State.B10 = *( DRAM_data + State_B10_Addr);
	F_State.B11 = *( DRAM_data + State_B11_Addr);
	F_State.B12 = *( DRAM_data + State_B12_Addr);

	F_State.C1 = *( DRAM_data +  State_C1_Addr);
	F_State.C2 = *( DRAM_data +  State_C2_Addr);
	F_State.C3 = *( DRAM_data +  State_C3_Addr);
	F_State.C4 = *( DRAM_data +  State_C4_Addr);
	F_State.C5 = *( DRAM_data +  State_C5_Addr);
	F_State.C6 = *( DRAM_data +  State_C6_Addr);
	F_State.C7 = *( DRAM_data +  State_C7_Addr);
	F_State.C8 = *( DRAM_data +  State_C8_Addr);
	F_State.C9 = *( DRAM_data +  State_C9_Addr);
	F_State.C10 = *( DRAM_data + State_C10_Addr);
	F_State.C11 = *( DRAM_data + State_C11_Addr);
	F_State.C12 = *( DRAM_data + State_C12_Addr);

//	if((F_State.A1 != 0x108)||(F_State.A2 != 0x108)||(F_State.A3 != 0x108)
//	 ||(F_State.A4 != 0x108)||(F_State.A5 != 0x108)||(F_State.A6 != 0x108)
//	 ||(F_State.A7 != 0x108)||(F_State.A8 != 0x108)||(F_State.A9 != 0x108)
//	 ||(F_State.A10!= 0x108)||(F_State.A11!= 0x108)||(F_State.A12!= 0x108)
//	 ||(F_State.B1 != 0x108)||(F_State.B2 != 0x108)||(F_State.B3 != 0x108)
//	 ||(F_State.B4 != 0x108)||(F_State.B5 != 0x108)||(F_State.B6 != 0x108)
//	 ||(F_State.B7 != 0x108)||(F_State.B8 != 0x108)||(F_State.B9 != 0x108)
//	 ||(F_State.B10!= 0x108)||(F_State.B11!= 0x108)||(F_State.B12!= 0x108)
//	 ||(F_State.C1 != 0x108)||(F_State.C2 != 0x108)||(F_State.C3 != 0x108)
//	 ||(F_State.C4 != 0x108)||(F_State.C5 != 0x108)||(F_State.C6 != 0x108)
//	 ||(F_State.C7 != 0x108)||(F_State.C8 != 0x108)||(F_State.C9 != 0x108)
//	 ||(F_State.C10!= 0x108)||(F_State.C11!= 0x108)||(F_State.C12!= 0x108))
//	{
//		ERROR_flag |= Local_Fault;  //启动时故障检测不通过
//	}
	if(F_State.A1 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F1;
		Run_Stop_Flag = 0;
	}
	if(F_State.A2 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F2;
		Run_Stop_Flag = 0;
	}
	if(F_State.A3 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F3;
		Run_Stop_Flag = 0;
	}
	if(F_State.A4 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F4;
		Run_Stop_Flag = 0;

	}
	if(F_State.A5 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F5;
		Run_Stop_Flag = 0;
	}
	if(F_State.A6 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F6;
		Run_Stop_Flag = 0;
	}
	if(F_State.A7 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F7;
		Run_Stop_Flag = 0;
	}
	if(F_State.A8 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F8;
		Run_Stop_Flag = 0;
	}
	if(F_State.A9 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F9;
		Run_Stop_Flag = 0;
	}
	if(F_State.A10!= 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F10;
		Run_Stop_Flag = 0;

	}
	if(F_State.A11!= 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F11;
		Run_Stop_Flag = 0;
	}
	if(F_State.A12!= 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERROR_module |= F12;
		Run_Stop_Flag = 0;
	}

	if(F_State.B1 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F1;
		Run_Stop_Flag = 0;
	}
	if(F_State.B2 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F2;
		Run_Stop_Flag = 0;
	}
	if(F_State.B3 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F3;
		Run_Stop_Flag = 0;
	}
	if(F_State.B4 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F4;
		Run_Stop_Flag = 0;
	}
	if(F_State.B5 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F5;
		Run_Stop_Flag = 0;
	}
	if(F_State.B6 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F6;
		Run_Stop_Flag = 0;
	}
	if(F_State.B7 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F7;
		Run_Stop_Flag = 0;
	}
	if(F_State.B8 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F8;
		Run_Stop_Flag = 0;
	}
	if(F_State.B9 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F9;
		Run_Stop_Flag = 0;
	}
	if(F_State.B10 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F10;
		Run_Stop_Flag = 0;

	}
	if(F_State.B11 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F11;
		Run_Stop_Flag = 0;
	}
	if(F_State.B12 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_B |= F12;
		Run_Stop_Flag = 0;
	}

	if(F_State.C1 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F1;
		Run_Stop_Flag = 0;
	}
	if(F_State.C2 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F2;
		Run_Stop_Flag = 0;
	}
	if(F_State.C3 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F3;
		Run_Stop_Flag = 0;
	}
	if(F_State.C4 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F4;
		Run_Stop_Flag = 0;
	}
	if(F_State.C5 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F5;
		Run_Stop_Flag = 0;
	}
	if(F_State.C6 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F6;
		Run_Stop_Flag = 0;
	}
	if(F_State.C7 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F7;
		Run_Stop_Flag = 0;
	}
	if(F_State.C8 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F8;
		Run_Stop_Flag = 0;
	}
	if(F_State.C9 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F9;
		Run_Stop_Flag = 0;
	}
	if(F_State.C10 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F10;
		Run_Stop_Flag = 0;

	}
	if(F_State.C11 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F11;
		Run_Stop_Flag = 0;
	}
	if(F_State.C12 != 0x108)
	{
		ERROR_flag |= Local_Fault;
		ERR_M_C |= F12;
		Run_Stop_Flag = 0;
	}
}

//---------------------------------通用函数--------------------------------//
void delay_t(int a)
{
	int n=0;
  for(n=0;n<a;n++)
  {
  ;
  }
}
