#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
/******************************初始化***************************************/
//系统时钟及外设时钟初始化
void InitSysCtrl(void)
{

   // Disable the watchdog
   DisableDog();

   // Initialize the PLL control: PLLCR and DIVSEL
   // DSP28_PLLCR and DSP28_DIVSEL are defined in DSP2833x_Examples.h
   //#define DSP28_PLLCR    10
	//#define DSP28_DIVSEL   2
   //InitPll(DSP28_PLLCR,DSP28_DIVSEL);
   InitPll(10,2);

   // Initialize the peripheral clocks
   InitPeripheralClocks();
}
void ServiceDog(void)
{
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;
}
void DisableDog(void)
{
    EALLOW;
    SysCtrlRegs.WDCR= 0x0068;
    EDIS;
}
void InitPll(Uint16 val, Uint16 divsel)
{

   // Make sure the PLL is not running in limp mode
   if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
   {
      // Missing external clock has been detected
      // Replace this line with a call to an appropriate
      // SystemShutdown(); function.
      asm("        ESTOP0");
   }

   // DIVSEL MUST be 0 before PLLCR can be changed from
   // 0x0000. It is set to 0 by an external reset XRSn
   // This puts us in 1/4
   if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
   {
       EALLOW;
       SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
       EDIS;
   }

   // Change the PLLCR
   if (SysCtrlRegs.PLLCR.bit.DIV != val)
   {

      EALLOW;
      // Before setting PLLCR turn off missing clock detect logic
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
      SysCtrlRegs.PLLCR.bit.DIV = val;
      EDIS;

      // Optional: Wait for PLL to lock.
      // During this time the CPU will switch to OSCCLK/2 until
      // the PLL is stable.  Once the PLL is stable the CPU will
      // switch to the new PLL value.
      //
      // This time-to-lock is monitored by a PLL lock counter.
      //
      // Code is not required to sit and wait for the PLL to lock.
      // However, if the code does anything that is timing critical,
      // and requires the correct clock be locked, then it is best to
      // wait until this switching has completed.

      // Wait for the PLL lock bit to be set.

      // The watchdog should be disabled before this loop, or fed within
      // the loop via ServiceDog().

	  // Uncomment to disable the watchdog
      DisableDog();

      while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)
      {
	      // Uncomment to service the watchdog
          // ServiceDog();
      }

      EALLOW;
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
      EDIS;
    }

	EALLOW;
	SysCtrlRegs.PLLSTS.bit.DIVSEL = divsel;
	EDIS;
}

void InitPeripheralClocks(void)
{
	EALLOW;

// HISPCP/LOSPCP prescale register settings, normally it will be set to default values
	SysCtrlRegs.HISPCP.all = 0x0001;
	SysCtrlRegs.LOSPCP.all = 0x0002;

// XCLKOUT to SYSCLKOUT ratio.  By default XCLKOUT = 1/4 SYSCLKOUT
// XTIMCLK = SYSCLKOUT/2
//	XintfRegs.XINTCNF2.all=0x03;
	XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
// XCLKOUT = XTIMCLK/2
	XintfRegs.XINTCNF2.bit.CLKMODE = 1;
// Enable XCLKOUT
	XintfRegs.XINTCNF2.bit.CLKOFF = 0;

// Peripheral clock enables set for the selected peripherals.
// If you are not using a peripheral leave the clock off
// to save on power.
//
// Note: not all peripherals are available on all 2833x derivates.
// Refer to the datasheet for your particular device.
//
// This function is not written to be an example of efficient code.

	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0;    // ADC

// *IMPORTANT*
// The ADC_cal function, which  copies the ADC calibration values from TI reserved
// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
// Boot ROM. If the boot ROM code is bypassed during the debug process, the
// following function MUST be called for the ADC to function according
// to specification. The clocks to the ADC MUST be enabled before calling this
// function.
// See the device data manual and/or the ADC Reference
// Manual for more information.

	ADC_cal();

//	SysCtrlRegs.PCLKCR0.all=0x0028;
//	SysCtrlRegs.PCLKCR3.all=0xFFFF;

	SysCtrlRegs.PCLKCR0.bit.I2CAENCLK = 1;   // I2C
	SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;   // SCI-A
	SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 1;   // SCI-B
	SysCtrlRegs.PCLKCR0.bit.SCICENCLK = 1;   // SCI-C
	SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 0;   // SPI-A
	SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK = 1; // McBSP-A
	SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK = 1; // McBSP-B
	SysCtrlRegs.PCLKCR0.bit.ECANAENCLK = 0;    // eCAN-A
	SysCtrlRegs.PCLKCR0.bit.ECANBENCLK = 0;    // eCAN-B

	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Disable TBCLK within the ePWM
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 0;  // ePWM1
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 0;  // ePWM2
	SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 0;  // ePWM3
	SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0;  // ePWM4
	SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 0;  // ePWM5
	SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 0;  // ePWM6
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // Enable TBCLK within the ePWM

	SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK = 0;  // eCAP3
	SysCtrlRegs.PCLKCR1.bit.ECAP4ENCLK = 0;  // eCAP4
	SysCtrlRegs.PCLKCR1.bit.ECAP5ENCLK = 0;  // eCAP5
	SysCtrlRegs.PCLKCR1.bit.ECAP6ENCLK = 0;  // eCAP6
	SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 0;  // eCAP1
	SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK = 0;  // eCAP2
	SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 0;  // eQEP1
	SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK = 0;  // eQEP2

	SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = 0; // CPU Timer 1
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 0; // CPU Timer 2

	SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 0;       // DMA Clock
	SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;     // XTIMCLK
	SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;    // GPIO input clock

	EDIS;
}


/*------------------------------------------------------------------------------------**
** 	                                                                                  **
** void InitTIMER1(void)                                                              **
** {                                                                                  **
** 	CpuTimer1.RegsAddr = &CpuTimer1Regs;                                              **
** 	// Initialize timer period to maximum:                                            **
** 	CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;                                              **
**                                                                                    **
**         // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):                **
** 	CpuTimer1Regs.TPR.all  = 0;                                                       **
** 	CpuTimer1Regs.TPRH.all = 0;                                                       **
**                                                                                    **
**         // Make sure timers are stopped:                                           **
** 	CpuTimer1Regs.TCR.bit.TSS = 1;                                                    **
**                                                                                    **
** 	// Reload all counter register with period value:                                 **
** 	CpuTimer1Regs.TCR.bit.TRB = 1;                                                    **
**                                                                                    **
** 	// Reset interrupt counters:                                                      **
** 	CpuTimer1.InterruptCount = 0;                                                     **
** }                                                                                  **
** void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period)         **
** {                                                                                  **
** 	Uint32 	temp;                                                                     **
**                                                                                    **
** 	// Initialize timer period:                                                       **
** 	Timer->CPUFreqInMHz = Freq;                                                       **
** 	Timer->PeriodInUSec = Period;                                                     **
** 	temp = (long) (Freq * Period);                                                    **
** 	Timer->RegsAddr->PRD.all = temp;                                                  **
**                                                                                    **
** 	// Set pre-scale counter to divide by 1 (SYSCLKOUT):                              **
** 	Timer->RegsAddr->TPR.all  = 0;                                                    **
** 	Timer->RegsAddr->TPRH.all  = 0;                                                   **
**                                                                                    **
** 	// Initialize timer control register:                                             **
** 	Timer->RegsAddr->TCR.bit.TSS = 1;      // 1 = Stop timer, 0 = Start/Restart Timer **
** 	Timer->RegsAddr->TCR.bit.TRB = 1;      // 1 = reload timer                        **
** 	Timer->RegsAddr->TCR.bit.SOFT = 0;                                                **
** 	Timer->RegsAddr->TCR.bit.FREE = 0;     // Timer Free Run Disabled                 **
** 	Timer->RegsAddr->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt **
**                                                                                    **
** 	// Reset interrupt counter:                                                       **
** 	Timer->InterruptCount = 0;                                                        **
** }                                                                                  **
**------------------------------------------------------------------------------------*/

