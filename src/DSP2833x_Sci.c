// TI File $Revision: /main/2 $
// Checkin $Date: March 1, 2007   16:06:07 $
//###########################################################################
//
// FILE:	DSP2833x_Sci.c
//
// TITLE:	DSP2833x SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File



//---------------------------------------------------------------------------
// Example: InitSciGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SCI pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 
// Caution: 
// Only one GPIO pin should be enabled for SCITXDA/B operation.
// Only one GPIO pin shoudl be enabled for SCIRXDA/B operation. 
// Comment out other unwanted lines.

void InitSciGpio()
{
   InitSciaGpio();
#if DSP28_SCIB   
   InitScibGpio();
#endif // if DSP28_SCIB  
#if DSP28_SCIC
   InitScicGpio();
#endif // if DSP28_SCIC
}

void InitSciaGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.

//	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
	GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;    // Enable pull-up for GPIO36 (SCIRXDA)
	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	   // Enable pull-up for GPIO29 (SCITXDA)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.

//	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
	GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 3;  // Asynch input GPIO36 (SCIRXDA)

/* Configure SCI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

//	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   
	// Configure GPIO28 for SCIRXDA operation
	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 1;
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
	
    EDIS;
}

#if DSP28_SCIB 
void InitScibGpio()
{
   EALLOW;
	
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;     // Enable pull-up for GPIO9  (SCITXDB)
  GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up for GPIO14 (SCITXDB)
//	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	   // Enable pull-up for GPIO18 (SCITXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;    // Enable pull-up for GPIO22 (SCITXDB)

	
//  GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up for GPIO11 (SCIRXDB)
  GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up for GPIO15 (SCIRXDB)

//    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	   
    // Enable pull-up for GPIO19 (SCIRXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up for GPIO23 (SCIRXDB)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3;  // Asynch input GPIO11 (SCIRXDB)
  GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (SCIRXDB)
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  
	// Asynch input GPIO19 (SCIRXDB)
//  GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)

/* Configure SCI-B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;    // Configure GPIO9 for SCITXDB operation
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;   // Configure GPIO14 for SCITXDB operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   
	// Configure GPIO18 for SCITXDB operation
   //GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;   
   // Configure GPIO22 for SCITXDB operation
	
//  GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;   // Configure GPIO11 for SCIRXDB operation
  GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;   // Configure GPIO15 for SCIRXDB operation
//    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   
    // Configure GPIO19 for SCIRXDB operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO23 for SCIRXDB operation
	
    EDIS;
}
#endif // if DSP28_SCIB 

#if DSP28_SCIC
void InitScicGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

/* Configure SCI-C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation
	
    EDIS;
}
#endif // if DSP28_SCIC 

//---------------------------------------------------------------------------
// InitSci: 
//---------------------------------------------------------------------------
// This function initializes the SCI(s) to a known state.
//
void InitSci(void)
{
	InitSciGpio();
	// Initialize SCI-A:
	SciaRegs.SCICCR.all =0x0007;   	// 1 stop bit,  No loopback
                                   	// No parity,8 char bits,
                                   	// async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  	// enable TX, RX, internal SCICLK,
                                   	// Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0x0003;   //初始使能Tx/Rx中断,但紧接着又禁止Tx/Rx中断
	SciaRegs.SCICTL2.bit.TXINTENA =0;
	SciaRegs.SCICTL2.bit.RXBKINTENA =0;  //1：使能接收满中断
    SciaRegs.SCIHBAUD=0x0001;   //9600 baud @LSPCLK37.5MHz	//0x0003;
    SciaRegs.SCILBAUD=0x00E7;   //0x00B6;
	SciaRegs.SCICCR.bit.LOOPBKENA =0;	// Disable loop back
	ScicRegs.SCICTL1.bit.RXENA = 1;
	ScicRegs.SCICTL1.bit.TXENA = 1;
	ScicRegs.SCICTL1.bit.SWRESET = 1;  //配置完参数后,软件重启SCI(bit5:SW_RST)
	SciaRegs.SCIFFRX.bit.RXFFIENA=0;
	//tbd...
 	
	#if DSP28_SCIB 
	// Initialize SCI-B:
	ScibRegs.SCICCR.all =0x0007;   	// 1 stop bit,  No loopback
                                   	// No parity,8 char bits,
                                   	// async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  	// enable TX, RX, internal SCICLK,
                                   	// Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all =0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA =0;
	ScibRegs.SCICTL2.bit.RXBKINTENA =0;
    ScibRegs.SCIHBAUD=0x0001;  	//3;
    ScibRegs.SCILBAUD=0x00E7;  	//B6;
	ScibRegs.SCICCR.bit.LOOPBKENA =0;	// Disable loop back
	ScicRegs.SCICTL1.bit.RXENA = 1;
	ScicRegs.SCICTL1.bit.TXENA = 1;
	ScicRegs.SCICTL1.bit.SWRESET = 1;  //配置完参数后,软件重启SCI(bit5:SW_RST)
	ScibRegs.SCIFFRX.bit.RXFFIENA=0;
	#endif
	//tbd...
 
      // Initialize SCI-C:
	#if DSP28_SCIC 
	ScicRegs.SCICCR.all =0x0007;   	// 1 stop bit,  No loopback 1位停止位,禁止回送测试功能
                                   	// No parity,8 char bits,禁止奇偶校验
                                   	// async mode, idle-line protocol空闲线模式
	ScicRegs.SCICTL1.all =0x0003;  	// enable TX, RX, internal SCICLK,
                                   	// Disable RX ERR, SLEEP, TXWAKE
	ScicRegs.SCICTL2.all =0x0003;
	ScicRegs.SCICTL2.bit.TXINTENA =0;
	ScicRegs.SCICTL2.bit.RXBKINTENA =0;  //1: 接收中断使能
    ScicRegs.SCIHBAUD=0x0001;  	//0x03;
    ScicRegs.SCILBAUD=0x00E7;  	//0xB6;
	ScicRegs.SCICCR.bit.LOOPBKENA =0;	// Disable loop back
	ScicRegs.SCICTL1.bit.RXENA = 1;
	ScicRegs.SCICTL1.bit.TXENA = 1;
	ScicRegs.SCICTL1.bit.SWRESET = 1;  //配置完参数后,软件重启SCI(bit5:SW_RST)

	ScicRegs.SCIFFTX.bit.SCIFFENA = 1;  //使能SCI FIFO增强型的功能
	ScicRegs.SCIFFRX.bit.RXFFOVRCLR = 1; //清除FIFO溢出标志位RXFFOVF
	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;  //重新使能接收FIFO的操作
	ScicRegs.SCIFFRX.bit.RXFFIENA =0;  //禁止FIFO接收中断
//	ScicRegs.SCIFFRX.bit.RXFFIL = 16;  //FIFO接收中断级别为16,即当接收FIFO中有16个字符时发送中断
	#endif
}		
//===========================================================================
// End of file.
//===========================================================================
