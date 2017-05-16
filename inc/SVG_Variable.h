//###########################################################################
//--------------------------------------------------------------------------
// FILE:  Variable.c
// DESCRIPTION:
//         including those variables used in the 1140V SVG system
//--------------------------------------------------------------------------
// $Author: Chao  zhang $
// $Date: June  1, 2015$
// $Target System: DSP F28335 + FPGA   高压SVG $
// $Version: v2.1	LI Shuzhen $
//--------------------------------------------------------------------------
//###########################################################################
#ifndef SVG_VARIABLE_H
#define SVG_VARIABLE_H

#include "DSP2833x_Device.h"

Uint16 State_Machine = 0;
Uint16 delay_Count = 53333;
int Init_i = 0;
int delay_time_send = 150;//200;
int delay_time_rcvd = 150;//200;
int delay_time_AD = 12;//20;
int delay_time_DA = 12;//20;    //分控DA延时，有FIFO
int delay_time_Main_DA = 12; //150; //主控DA延时,130即可,不能再少了,主控的DA串行传输速率设置原来是2.34M,太低了。最高可支持30M
int delay_PWM = 400;//450;//500;
int S_Status = 0;  //合闸为0，分闸为1
int Reset_Flag = 1;
int NMICount = 0;
Uint16 CpuTimer0Count = 0;
Uint16 DA_Switch = 1;
int Conductor_i = 0;

float KPCC_VCT = 20;  //PCC点电压VT传感器变比,2000V对应100V
//PCC点电网电压定标
float K_UA = 0.5248; //(变比3000:100,2.5Kv可并网)0.1154;//(变比3000:100)0.06855; //0.0457;(变比2000:100) //0.046158;(变比2000:100)
float K_UB = 0.5285; //0.1162; //0.06937;//0.0463; //0.046248;
float K_UC = 0.5232; //0.1177; // 0.06825; //0.0455; //0.045706;
float UA_Z = -149.58; //-36.937; //-39.3795;//-26.253; //-18.462;
float UB_Z = 362.02; //87.762; //25.293; //16.862; //46.016;
float UC_Z = -444.73;//-91.23;//3.8589;  //2.5726; //-42.278;

float KLoad_ICT = 40;  //负载侧电流CT传感器变比,需要在触摸屏中显示
float K_IA = 0.00037787;  //负载侧电流定标
float K_IB = 0.00037672;
float K_IC = 0.00037637;
float IA_Z = -0.022955;
float IB_Z = 0.0030137;
float IC_Z = -0.027569;

float K_IA_INV = 0.015253;  //SVG变流器测电流定标
float K_IB_INV = 0.01533;
float K_IC_INV = 0.015472;
float IA_Z_INV = 0.18558;
float IB_Z_INV = 0.38324;
float IC_Z_INV = 0.46415;
/*
float K_UA1 = 0.033251;//0.032618;  //直流母线电压定标
float K_UA2 = 0.032772;
float K_UA3 = 0.032921;
float UDC_ZA1 = -5.8313;//1.4633;
float UDC_ZA2 = 9.9565;
float UDC_ZA3 = 8.3642;

float K_UB1 = 0.032919;
float K_UB2 = 0.032868;
float K_UB3 = 0.032683;
float UDC_ZB1 = 4.0159;
float UDC_ZB2 = 7.9605;
float UDC_ZB3 = 4.8421;

float K_UC1 = 0.032684;
float K_UC2 = 0.033085;
float K_UC3 = 0.033121;
float UDC_ZC1 = 6.7565;
float UDC_ZC2 = -2.2997;
float UDC_ZC3 = 2.3194;
*/

float UDC_Amp = 3.75;  //1.87V对应实际399V; 经同相放大器,所以3.5V对应750V
float Ipcc_Amp = 3;//4.5;  //80A RMS的峰值对应输入是1.19V,经同相放大器后,为2.4V;所以3V对应100A RMS
float General_Protect = 2;
int DA0 = 0;
int DA1 = 0;
int DA2 = 0;
int DA3 = 0;
int DA5 = 0;
int DA6 = 0;

int Chose_angle = 0;
float Balance=0;
float DA_Current = 0;   //DA输出电流
float DA_Voltage = 0;   //DA输出电压
float DA_Phase = 0;     //DA输出相角
float DA_OutPhase = 0;  //相间平衡
float DA_InterPhase = 0;//相内平衡
float DA_Current_dq = 0;//DQ轴上的电流与指令
float DA_Voltage_neg = 0;//负序电压指令
float DA_Voltage_A=0;
float DA_Voltage_B=0; 
float DA_Voltage_C=0;
float DA_Phase_PLL=0;
int Main_DA[8]={2048,2048,2048,2048,2048,2048,2048,2048};
int DA_Test[8]={0};

float CloseGrid = 0;    //闭环并网 稳压  默认状态
float InterPhase = 0;   //闭环并网 稳压 相内平衡控制
float OuterPhase = 0;   //闭环并网 稳压 相间平衡控制
float InOuterPhase = 0; //闭环并网 稳压 相内/间平衡控制
float OpenGrid = 0;     //开环并网

float Soft_Protect = 0; //直流侧母线电压保护软开关
int Negative = 0;

float Start_Balance = 0;

//------------------------------PLL相关变量--------------------------------//
Uint16 PLL_Switch = 100;
//PLL_Switch = 0，dq坐标轴下锁A相相角
float Usd_PLL = 0;
float Usq_PLL = 0;
float Ki_PLL = 50;//220;
float Kp_PLL = 15;//90;
float Omiga=0;
float Omiga_t=0;
float Omiga_PLL = 0;
float invented_integral=0;
float integral_PLL = 0;

//PLL_Switch = 1，SOGI-FLL正负序电压分离，锁A相正序电压
float Omiga_FLL = 0;
float integral_Omiga = 0;
float U_alpha_Pos = 0;
float U_beta_Pos = 0;
float U_alpha_Neg = 0;
float U_beta_Neg = 0;
//正序SOGI
float FLL_err1 = 0;
float Vd1 = 100;
float Vq1 = 0;
float integral_Vd1 = 0;
//负序SOGI
float FLL_err2 = 0;
float Vd2 = 100;
float Vq2 = 0;
float integral_Vd2 = 0;

//PLL_Switch = 6，DDSRF-SPLL
//float Vd_P = 0;
//float Vq_P = 0;
//float Vd_N = 0;
//float Vq_N = 0;
//float Vd_P1 = 0;
//float Vq_P1 = 0;
//float Vd_N1 = 0;
//float Vq_N1 = 0;
//float Vd_P_ave = 0;
//float Vq_P_ave = 0;
//float Vd_N_ave = 0;
//float Vq_N_ave = 0;
//float Vq_P1_integ = 0;
//float Vq_P1_out = 0;
//float theta_integ = 0;
//Uint16 Ts_SPLL = 250;
//float PLL_Ki = 1;
//float PLL_Kp = 1;
//------------------------------PLL相关结束--------------------------------//

//----------------------------重复控制用到的变量-----------------------------//
//float Repetive_d=0;
//float Repetive_q=0;
//float Repetive_out_d=0;
//float Repetive_out_q=0;
//Uint16 Ahead_Point=3;
//Uint16 ahead_n = 6;  //6;
//float Qr = 0.95;
//float new_out_d = 0;
//float new_out_q = 0;
//Uint16 ii = 0;
//float ahead_pwm_d[125] = {0};
//float ahead_pwm_q[125] = {0};
//float out_pwm_d[125] = {0};
//float out_pwm_q[125] = {0};
//float burrfer_error_d[4] = {0,0,0,0};
//float burrfer_error_q[4] = {0,0,0,0};
//float Rpt_PI1_integ = 0;
//float Rpt_PI2_integ = 0;
//float Rpt_PI1_out = 0;
//float Rpt_PI2_out = 0;
//-------------------------------重复控制结束--------------------------------//

//-------------------------------DA生成正弦波-------------------------------//
Uint16 Sine_m = 100;
//float Sine_t = 5.76;
//float Modu = 0;
//float Modu_AD = 0;
//float Modu_Filter = 0;
//float filter_band_stop_Modu_AD[4] = {0,0,0,0};
//Uint16 filter=0;
//------------------------------DA生成正弦波结束-----------------------------//

//--------------------------------graph显示--------------------------------//
//Uint16 graph_Switch=3;
Uint16 m = 0;
//Uint16 Control = 0;
//float graph_data1[400]={0};
//float graph_data2[400]={0};
//float graph_data3[400]={0};
//float graph_data4[400]={0};
//float graph_data5[400]={0};
//float graph_data6[400]={0};
//float graph_data7[400]={0};
//float graph_data8[400]={0};
//float graph_data9[400]={0};
//-------------------------------graph显示结束------------------------------//

//-------------------------------采样模块变量--------------------------------//
Uint16 Ts_US = 80;
Uint16 Ts_UDC = 200;	//滤除UDC上的二倍频波动,感性45Hz,容性200Hz
Uint16 Ts_Id = 2500;

float 	UDC_AR1=0;	//直流侧电压滤波
float 	UDC_BR1=0;
float	UDC_CR1=0;
float	UDC_AR2=0;
float 	UDC_BR2=0;
float 	UDC_CR2=0;
float	UDC_AR3=0;
float	UDC_BR3=0;
float	UDC_CR3=0;
float	UDC_AR4=0;
float	UDC_BR4=0;
float	UDC_CR4=0;
float	UDC_AR5=0;
float	UDC_BR5=0;
float	UDC_CR5=0;
float	UDC_AR6=0;
float	UDC_BR6=0;
float	UDC_CR6=0;
float 	UDC_AR7=0;
float 	UDC_BR7=0;
float	UDC_CR7=0;
float	UDC_AR8=0;
float 	UDC_BR8=0;
float 	UDC_CR8=0;
float	UDC_AR9=0;
float	UDC_BR9=0;
float	UDC_CR9=0;
float	UDC_AR10=0;
float	UDC_BR10=0;
float	UDC_CR10=0;
float	UDC_AR11=0;
float	UDC_BR11=0;
float	UDC_CR11=0;
float	UDC_AR12=0;
float	UDC_BR12=0;
float	UDC_CR12=0;

float   UDC_AS1=0;  //直流侧电压陷波滤波
float   UDC_BS1=0;
float	UDC_CS1=0;
float	UDC_AS2=0;
float   UDC_BS2=0;
float   UDC_CS2=0;
float	UDC_AS3=0;
float	UDC_BS3=0;
float	UDC_CS3=0;
float	UDC_AS4=0;
float	UDC_BS4=0;
float	UDC_CS4=0;
float	UDC_AS5=0;
float	UDC_BS5=0;
float	UDC_CS5=0;
float	UDC_AS6=0;
float	UDC_BS6=0;
float	UDC_CS6=0;
float 	UDC_AS7=0;
float 	UDC_BS7=0;
float	UDC_CS7=0;
float	UDC_AS8=0;
float 	UDC_BS8=0;
float 	UDC_CS8=0;
float	UDC_AS9=0;
float	UDC_BS9=0;
float	UDC_CS9=0;
float	UDC_AS10=0;
float	UDC_BS10=0;
float	UDC_CS10=0;
float	UDC_AS11=0;
float	UDC_BS11=0;
float	UDC_CS11=0;
float	UDC_AS12=0;
float	UDC_BS12=0;
float	UDC_CS12=0;

float 	UDC_averA = 0;   //直流侧电压平均值
float 	UDC_averB = 0;
float	UDC_averC = 0;
float 	UDC_aver= 0;     //直流侧电压平均值

float	ADC_IA = 0;		//负载AD采样电流 数字量
float	ADC_IB = 0;
float	ADC_IC = 0;

float	ADC_VAB = 0;	//电网AD采样电压 数字量
float	ADC_VBC = 0;
float	ADC_VCA = 0;

float	ADC_IA_INV = 0;	//变流器AD采样电流 数字量
float	ADC_IB_INV = 0;
float	ADC_IC_INV = 0;

float   IASVG=0;	//变流器A相电流 实际值
float   IBSVG=0;
float   ICSVG=0;

float   IA_Load=0;	//A相负载电流 实际值
float   IB_Load=0;
float   IC_Load=0;
float	IA_Load_Z = 0;
float	IB_Load_Z = 0;
float	IA_Load_Pre = 0;
float	IB_Load_Pre = 0;

float   UAB_DRF=0;	//电网AB线电压 实际值
float   UBC_DRF=0;
float   UCA_DRF=0;
float   U_ADRF=0;	//电网A相电压 实际值
float   U_BDRF=0;
float   U_CDRF=0;
//-----------------------------采样模块变量结束-------------------------------//

//-----------------------------开环并网电压变量-------------------------------//
float 	U_alpha_open_pre=0;	//相位补偿前，开环并网电压
float 	U_beta_open_pre=0;
float   US_Pre=0;			//相位补偿前，开环并网在极坐标下的幅值信息
float   US_Display = 0;
float   US = 0;
float	THETA_C=0;    		//相位补偿后的电网电压相角

float   Gama = 0;  			//相内平衡电流相角
float   THETA_d_Com = 5;  	//因为d轴分量的存在导致电流相角超前或滞后电压大于90度,且75AIq时,Id约为6A
float   THETA_Neg_Com = 0;
float   THETA_COM_Usq = 2;	//锁相环的补偿相角（度）
float   THETA_C_ab = 0;
float 	THETA_IA = 0; 		//相内电压平衡，A相相角

//相位补偿后，开环并网电压,相位补偿后,U_alpha_open_after = U_alpha_open_pre;
//相位不补偿，则存在0.1ms左右的相位差
float   U_alpha_open_after;  
float   U_beta_open_after;

//相位补偿后，开环并网电压
float   U_aout_open;
float   U_bout_open;
float   U_cout_open;

float	U_FtoI = 1;	//开环前馈电压系数,高压时需要增大，否则启动瞬间电容电压有较大冲击
//-------------------------------开环变量结束--------------------------------//

//-----------------------------闭环并网电流变量-------------------------------//
float	Q = 0;
float	U_dF = 0;
float	U_qF = 0;
float   I_derror=0;
float   I_qerror=0;

float	Curr_PI1_integ = 0;
float	Curr_PI2_integ = 0;
float	Curr_PI1_out = 0;
float	Curr_PI2_out = 0;
float   U_dout=0;
float   U_qout=0;

float   U_alpha_close=0;
float   U_beta_close=0;

float   U_aout_close=0;
float   U_bout_close=0;
float   U_cout_close=0;
//---------------------------闭环并网电流变量结束------------------------------//

//--------------------------------坐标变换变量-------------------------------//
float	LS1=0;		//三相静止坐标下的临时存储变量
float	LS2=0;
float	LS3=0;

float cosVal=0;		//正弦、余弦值
float sinVal=0;
//------------------------------坐标变换变量结束------------------------------//

//--------------------------------并网电流变量-------------------------------//
float   I_alpha;
float   I_beta;
float   P2P_alpha;
float   P2P_beta;
float   I_d_P2P = 0;
float   I_q_P2P = 0;
float   I_d_Pre = 0;
float   I_q_Pre = 0;
float   I_d=0;
float   I_q=0;
float   ISVG_Amp=5;
float	Q_REF = 10;	//无功电流指令
//-------------------------------并网电流变量结束-----------------------------//

//------------------------------直流侧稳压控制变量-----------------------------//
float   UDC_PI_integ = 0;    //I调节输出
float   UDC_PI_out = 0;      //PI输出
float   Idref=0;		     //稳压指令
float	UDC_err=0;           //PI输入
float	Udc_slip = 0;
//----------------------------直流侧稳压控制变量结束----------------------------//

//------------------------------相间电压平衡变量------------------------------//
float   UA_P2P=0;  //相间平衡量调制波
float   UB_P2P=0;
float   UC_P2P=0;
//负序电压注入法
float	THETA_neg=0;
float   U_neg_amp=0;
float   I_neg_amp = 0;
float   U_Z_amp = 0;
float	UA_THETA_neg = 0;
//负序电流注入法
//float   IA_P2P = 0;
//float   IB_P2P = 0;
//float   IC_P2P = 0;
//float   Ialpha_N = 0;
//float   Ibeta_N = 0;
//float   Id_N = 0;  //负序注入法加到电压环不给力，可参照仿真改加到电流环
//float   Iq_N = 0;

//零序电压注入法
float   THETA_Zero=0;
float   Kp_P2P_Zero=5;
float   U_Zero_amp=0;
//----------------------------相间电压平衡变量结束-----------------------------//

//-------------------------------ModBus通信--------------------------------//
int ReceivedChar=0;
int display_flag=0;
int com=0;
int com1=0;
int Run_Stop_Flag=0;  //初始未启动
int Para_SET=0;
int Fault_flag=0;
int Receive_date_error=0;
int Com_data[30]={0};
int Read_control[7]={0x1B,0x52,0x00,0x64,0x00,0x01,0xD3};
int display_data_one[14]={0x01,0x7C,0x01,0x7C,0x01,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int display_data_two[40]={0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x32,0x00,0x00};
int Para1_set[9]={0x1B,0x57,0x00,0xAA,0x00,0x01,0x00,0x01,0x8D};
int Para2_set[9]={0x1B,0x57,0x00,0xAA,0x00,0x01,0x00,0x02,0x8D};
int Para1_Intdata[18]={0x1B,0x57,0x00,0x96,0x00,0x05,0x00,0xC8,0x00,0xFA,0x00,0x02,0x00,0x3C,0x00,0x00,0x8D};
//int Para2_Intdata[15]={0x1B,0x57,0x00,0xC8,0x00,0x04,0x00,0x64,0x00,0x32,0x01,0x40,0x00,0x8C,0x8D};
int Current_IA[202]={0};
int Current_IB[202]={0};
int Current_IC[202]={0};
//int Current_I0[202]={0};
int C_data_count1=0;
int C_data_count2=0;
int C_data_count3=0;
int COM_UDCREF=0;		//150
int COM_Ih_max=0;		//151
int C_way=0;			//152  .1=1全补。1=0分次补  。3=1 补偿基波无功
int COM_ALL_rate=0;		//153
int COM_EXT_count=0;	//154
int COM_IHB_RATE=0;  	//155
int COM_UDC_Max=0;		//156
int COM_IH1=0;			//157
int COM_IH1_RATE=0;		//158
int COM_IH2=0;			//159
int COM_IH2_RATE=0;		//160
int COM_IH3=0;			//161
int COM_IH3_RATE=0;		//162
int COM_Kp=0;			//200
int COM_ki=0;			//201
int COM_Udb=0;			//202
int COM_Ua=0;			//203
int COM_Tu=0;			//204
int COM_Ti=0; 			//205
int COM_Iz=0;			//206
int COM_Uz=0; 			//207
int COM_U=0;    		//UABC
int COM_UAB=0;   
int COM_UBC=0;
int COM_UCA=0;
int COM_IA=0;
int COM_IB=0;
int COM_IC=0;
int COM_I0=0;
int THD_IA=0;
int THD_IB=0;
int THD_IC=0;
//int COM_Udc=0;
float COM_Udc1=0;
int ERROR_flag=0;
Uint16 ERROR_module=0;
Uint16 ERR_M_B=0;
Uint16 ERR_M_C=0;
int Run_Flag = 0;
int INRL_data=0;
int Current_flag=0;
int Init_para_flag=0;
//-----------------------------ModBus通信结束-------------------------------//

//------------------------------APF参数数据库-------------------------------//
struct SystemPara 
{
   const Uint16  DramAddr;//参数在Dram中的地址
   const Uint16  Rom_P_Addr;//参数在E2PROM的页地址
   const Uint16  Rom_O_Addr;//参数在E2PROM的页内偏移地址
   const Uint16  ScreenAddr;//参数在触摸屏中的地址
   const int  FactoryDefault;//参数出厂值
   const int  Max;//参数的最大值
   const int  Min;//参数的最小值
};
struct SystemPara  UDCREF_Ref = {1,0,0x10,150,650,600,800}; //声明UDCREF_Ref为结构体类型
//-----------------------------APF参数数据库结束-----------------------------//

//-------------------------------EEPROM变量--------------------------------//
int Factory_Flag = 0;	//恢复出厂设置
int EEPROM_Compare = 0;	//判断是否将参数写入EEPROM
//-----------------------------EEPROM变量结束-------------------------------//

//float WL = 0.52;
//-------------------------------滤波器变量---------------------------------//
//并网电流 二阶滤波器
float filter_2nd_d[4] = {0,0,0,0};
float filter_2nd_q[4] = {0,0,0,0};
//float filter_2nd_US[4] = {0,0,0,0};

//直流母线电压 二阶陷波器
float filter_band_stop_A1[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B1[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C1[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A2[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B2[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C2[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A3[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B3[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C3[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A4[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B4[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C4[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A5[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B5[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C5[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A6[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B6[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C6[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A7[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B7[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C7[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A8[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B8[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C8[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A9[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B9[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C9[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A10[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B10[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C10[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A11[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B11[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C11[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_A12[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_B12[8] = {0,0,0,0,0,0,0,0};
float filter_band_stop_C12[8] = {0,0,0,0,0,0,0,0};

//ZTM有功无功解耦DDACSM
//Uint16 ZTM_Switch = 0;
//float buffer1[4]={0,0,0,0};
//float buffer2[4]={0,0,0,0};
//float filter_2rd1[2][4]={{0,0,0,0},{0,0,0,0}};
//float filter_2rd2[2][4]={{0,0,0,0},{0,0,0,0}};
//float Td_pqDcpl = 300;  //解耦网络法一阶滤波器带宽BandWidth
//float alpha_result[4]={0,0,0,0};
//float belta_result[4]={0,0,0,0};
//float Com_Rate = 1;  //补偿率

//测试DDACSM
//float I_alpha1 = 0;
//float I_beta1 = 0;
//float I_d1 = 0;
//float I_q1 = 0;
//------------------------------滤波器变量结束-------------------------------//

struct PI_Ctrlers_Param
{
	float   UDCMAX;
	float   UDCMIN;
	float   UDCREF;
	float   UDC_SLIP;
	float   UDC_Kp;		//直流侧稳压控制器
	float   UDC_Ki;
	float	Ud_Kp;		//Id、Iq电流环
	float   Ud_Ki;
	Uint16  P2P_Flag;
	Uint16  In_Flag;
	float   Kp_P2P_Neg;	//相间平衡P控制器 负序电压注入法
	float   Kp_P2P_Z;	//相间平衡P控制器 零序电压注入法
	float   Kp_in;		//相内平衡P控制器
	Uint16  Cloop;		//闭环or开环or多电平发正弦波or固定占空比
	int		UDC_ORF;	//开环并网时的UDC期望值,此时一定要串联电阻
	float   Rpt_Kp;		//重复控制Kp系数
} Ctrl_Para;

struct Error_Locate
{
	Uint16 Flag;

	Uint16 A;

	Uint16 B;

	Uint16 C;
} CommuError;

struct Fault_State
{
	Uint16 Flag;

	Uint16 A1;
	Uint16 A2;
	Uint16 A3;
	Uint16 A4;
	Uint16 A5;
	Uint16 A6;
	Uint16 A7;
	Uint16 A8;
	Uint16 A9;
	Uint16 A10;
	Uint16 A11;
	Uint16 A12;

	Uint16 B1;
	Uint16 B2;
	Uint16 B3;
	Uint16 B4;
	Uint16 B5;
	Uint16 B6;
	Uint16 B7;
	Uint16 B8;
	Uint16 B9;
	Uint16 B10;
	Uint16 B11;
	Uint16 B12;

	Uint16 C1;
	Uint16 C2;
	Uint16 C3;
	Uint16 C4;
	Uint16 C5;
	Uint16 C6;
	Uint16 C7;
	Uint16 C8;
	Uint16 C9;
	Uint16 C10;
	Uint16 C11;
	Uint16 C12;
} F_State;

struct Data_Check
{
	Uint16 Flag;
	Uint16 Data;

	int16 A1;
	int16 A2;
	int16 A3;
	int16 A4;
	int16 A5;
	int16 A6;
	int16 A7;
	int16 A8;
	int16 A9;
	int16 A10;
	int16 A11;
	int16 A12;

	int16 B1;
	int16 B2;
	int16 B3;
	int16 B4;
	int16 B5;
	int16 B6;
	int16 B7;
	int16 B8;
	int16 B9;
	int16 B10;
	int16 B11;
	int16 B12;

	int16 C1;
	int16 C2;
	int16 C3;
	int16 C4;
	int16 C5;
	int16 C6;
	int16 C7;
	int16 C8;
	int16 C9;
	int16 C10;
	int16 C11;
	int16 C12;
} D_Check;

struct Module_Modulation
{
	//Positive, 此处Uint肯定不行!!!
	float  Pa;
	float  Pa1;
	float  Pa2;
	float  Pa3;
	float  Pa4;
	float  Pa5;
	float  Pa6;
	float  Pa7;
	float  Pa8;
	float  Pa9;
	float  Pa10;
	float  Pa11;
	float  Pa12;

	float  Pb;
	float  Pb1;
	float  Pb2;
	float  Pb3;
	float  Pb4;
	float  Pb5;
	float  Pb6;
	float  Pb7;
	float  Pb8;
	float  Pb9;
	float  Pb10;
	float  Pb11;
	float  Pb12;

	float  Pc;
	float  Pc1;
	float  Pc2;
	float  Pc3;
	float  Pc4;
	float  Pc5;
	float  Pc6;
	float  Pc7;
	float  Pc8;
	float  Pc9;
	float  Pc10;
	float  Pc11;
	float  Pc12;

	//Output
	Uint16 Oa1;
	Uint16 Oa2;
	Uint16 Oa3;
	Uint16 Oa4;
	Uint16 Oa5;
	Uint16 Oa6;
	Uint16 Oa7;
	Uint16 Oa8;
	Uint16 Oa9;
	Uint16 Oa10;
	Uint16 Oa11;
	Uint16 Oa12;

	Uint16 Ob1;
	Uint16 Ob2;
	Uint16 Ob3;
	Uint16 Ob4;
	Uint16 Ob5;
	Uint16 Ob6;
	Uint16 Ob7;
	Uint16 Ob8;
	Uint16 Ob9;
	Uint16 Ob10;
	Uint16 Ob11;
	Uint16 Ob12;

	Uint16 Oc1;
	Uint16 Oc2;
	Uint16 Oc3;
	Uint16 Oc4;
	Uint16 Oc5;
	Uint16 Oc6;
	Uint16 Oc7;
	Uint16 Oc8;
	Uint16 Oc9;
	Uint16 Oc10;
	Uint16 Oc11;
	Uint16 Oc12;
} M_out;

struct DC_Modules
{
	//AD采样的数字量,有符号位
	int16 DA1;
	int16 DA2;
	int16 DA3;
	int16 DA4;
	int16 DA5;
	int16 DA6;
	int16 DA7;
	int16 DA8;
	int16 DA9;
	int16 DA10;
	int16 DA11;
	int16 DA12;

	int16 DB1;
	int16 DB2;
	int16 DB3;
	int16 DB4;
	int16 DB5;
	int16 DB6;
	int16 DB7;
	int16 DB8;
	int16 DB9;
	int16 DB10;
	int16 DB11;
	int16 DB12;

	int16 DC1;
	int16 DC2;
	int16 DC3;
	int16 DC4;
	int16 DC5;
	int16 DC6;
	int16 DC7;
	int16 DC8;
	int16 DC9;
	int16 DC10;
	int16 DC11;
	int16 DC12;

	//相内平衡
	float inA1;
	float inA2;
	float inA3;
	float inA4;
	float inA5;
	float inA6;
	float inA7;
	float inA8;
	float inA9;
	float inA10;
	float inA11;
	float inA12;

	float inB1;
	float inB2;
	float inB3;
	float inB4;
	float inB5;
	float inB6;
	float inB7;
	float inB8;
	float inB9;
	float inB10;
	float inB11;
	float inB12;

	float inC1;
	float inC2;
	float inC3;
	float inC4;
	float inC5;
	float inC6;
	float inC7;
	float inC8;
	float inC9;
	float inC10;
	float inC11;
	float inC12;

	float deltA1;
	float deltA2;
	float deltA3;
	float deltA4;
	float deltA5;
	float deltA6;
	float deltA7;
	float deltA8;
	float deltA9;
	float deltA10;
	float deltA11;
	float deltA12;

	float deltB1;
	float deltB2;
	float deltB3;
	float deltB4;
	float deltB5;
	float deltB6;
	float deltB7;
	float deltB8;
	float deltB9;
	float deltB10;
	float deltB11;
	float deltB12;

	float deltC1;
	float deltC2;
	float deltC3;
	float deltC4;
	float deltC5;
	float deltC6;
	float deltC7;
	float deltC8;
	float deltC9;
	float deltC10;
	float deltC11;
	float deltC12;

	//相间平衡
	float deltA;
	float deltB;
	float deltC;
	float delt_alpha;
	float delt_beta;
	float delta_amp;
} UDC;

#endif   // end of SVG_VARIABLE_H definition
