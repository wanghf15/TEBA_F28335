// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:25 $
//###########################################################################
//
// FILE:	DSP2833x_Gpio.c
//
// TITLE:	DSP2833x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example. 
void InitGpio(void)
{
	//参考：
	//TMS320F28332, TMS320F28334, TMS320F28335 Digital Signal Controllers (DSCs)
	//Table4-14 GPIO Registers
/*   EALLOW;
   
   // Each GPIO pin can be: 
   // a) a GPIO input/output
   // b) peripheral function 1
   // c) peripheral function 2
   // d) peripheral function 3
   // By default, all are GPIO Inputs 
   GpioCtrlRegs.GPAMUX1.all = 0x0000;     // GPIO functionality GPIO0-GPIO15
   GpioCtrlRegs.GPAMUX2.all = 0x0000;     // GPIO functionality GPIO16-GPIO31
   GpioCtrlRegs.GPBMUX1.all = 0x0000;     // GPIO functionality GPIO32-GPIO39
   GpioCtrlRegs.GPBMUX2.all = 0x0000;     // GPIO functionality GPIO48-GPIO63
   GpioCtrlRegs.GPCMUX1.all = 0x0000;     // GPIO functionality GPIO64-GPIO79
   GpioCtrlRegs.GPCMUX2.all = 0x0000;     // GPIO functionality GPIO80-GPIO95

   GpioCtrlRegs.GPADIR.all = 0x0000;      // GPIO0-GPIO31 are inputs
   GpioCtrlRegs.GPBDIR.all = 0x0000;      // GPIO32-GPIO63 are inputs   
   GpioCtrlRegs.GPCDIR.all = 0x0000;      // GPI064-GPIO95 are inputs

   // Each input can have different qualification
   // a) input synchronized to SYSCLKOUT
   // b) input qualified by a sampling window
   // c) input sent asynchronously (valid for peripheral inputs only)
   GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // GPIO0-GPIO15 Synch to SYSCLKOUT 
   GpioCtrlRegs.GPAQSEL2.all = 0x0000;    // GPIO16-GPIO31 Synch to SYSCLKOUT
   GpioCtrlRegs.GPBQSEL1.all = 0x0000;    // GPIO32-GPIO39 Synch to SYSCLKOUT 
   GpioCtrlRegs.GPBQSEL2.all = 0x0000;    // GPIO48-GPIO63 Synch to SYSCLKOUT 

   // Pull-ups can be enabled or disabled. 
   GpioCtrlRegs.GPAPUD.all = 0x0000;      // Pullup's enabled GPIO0-GPIO31
   GpioCtrlRegs.GPBPUD.all = 0x0000;      // Pullup's enabled GPIO32-GPIO63
   GpioCtrlRegs.GPCPUD.all = 0x0000;      // Pullup's enabled GPIO64-GPIO79

   //GpioCtrlRegs.GPAPUD.all = 0xFFFF;    // Pullup's disabled GPIO0-GPIO31
   //GpioCtrlRegs.GPBPUD.all = 0xFFFF;    // Pullup's disabled GPIO32-GPIO34
   //GpioCtrlRegs.GPCPUD.all = 0xFFFF     // Pullup's disabled GPIO64-GPIO79

   EDIS;
*/
	EALLOW;

	//通用IO配置信息 均采用默认配置：不复用 不上拉 输入

	//GPIO30
	GpioCtrlRegs.GPAMUX2.bit.GPIO30=0;         	
    GpioCtrlRegs.GPAPUD.bit.GPIO30=1;   //0:enable(default) 1:disable
	GpioCtrlRegs.GPADIR.bit.GPIO30=0;   //0:INPUT(default)  1:OUTPUT
	//GPIO35   设置为输出 用于测试中断时间的长短
	GpioCtrlRegs.GPBMUX1.bit.GPIO35=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO35=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO35=1;
	//GPIO48  设置为输出 进入一次XINT1n故障中断则翻转一次
	GpioCtrlRegs.GPBMUX2.bit.GPIO48=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO48=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO48=1; //作输出口
	//GPIO49 观察触摸屏是否可向DSP写数据
	GpioCtrlRegs.GPBMUX2.bit.GPIO49=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO49=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO49=1;
	//GPIO50
	GpioCtrlRegs.GPBMUX2.bit.GPIO50=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO50=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO50=0;
	//GPIO51
	GpioCtrlRegs.GPBMUX2.bit.GPIO51=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO51=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO51=0;
	//GPIO52
	GpioCtrlRegs.GPBMUX2.bit.GPIO52=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO52=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO52=0;
	//GPIO53
	GpioCtrlRegs.GPBMUX2.bit.GPIO53=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO53=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO53=0;

	GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1;  
    GpioCtrlRegs.GPBDIR.bit.GPIO55 = 1;  
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1;  
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;

	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;  
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;  
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1;  
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 1;   

	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0; 
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0; 
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0; 

	//GPIO58
	GpioCtrlRegs.GPBMUX2.bit.GPIO58=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO58=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO58=0;
	//GPIO59
	GpioCtrlRegs.GPBMUX2.bit.GPIO59=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO59=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO59=0;
	//GPIO60 CON_C
	GpioCtrlRegs.GPBMUX2.bit.GPIO60=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO60=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO60=1;
	//GPIO61
	GpioCtrlRegs.GPBMUX2.bit.GPIO61=0;         	
    GpioCtrlRegs.GPBPUD.bit.GPIO61=1;
	GpioCtrlRegs.GPBDIR.bit.GPIO61=0;
	
	//暂时将TZ也设置为普通IO  不复用 不上拉 输入
	//GPIO12 用于FPGA的使能   低电平使能
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO12=1;
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;
	
	//GPIO13 response
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO13=1;
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;

	//GPIO16 request
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO16=1;
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;

	//GPIO17 XINT1n中断输入引脚 响应FPGA_ADC上传的故障信号
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO17=1;
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 2;    // XINT1 Qual using 6 samples
    GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xFF;// Each sampling window is 510*SYSCLKOUT

	//暂时将EQEP也设置为普通IO  不复用 上拉 输入
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO20=1;
	GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;

	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO21=1;
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;

	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO22=1;
	GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;

	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO23=1;
	GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;

	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO24=1;
	GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;

	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO25=1;
	GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;

	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO26=1;
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;

	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO27=1;
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;

/**/

	EDIS;
}	
	
//===========================================================================
// End of file.
//===========================================================================
