/*
// TI File $Revision: /main/9 $
// Checkin $Date: August 28, 2007   11:23:31 $
//###########################################################################
//
// FILE:    28335_RAM_lnk.cmd
//
// TITLE:   Linker Command File For 28335 examples that run out of RAM
//
//          This ONLY includes all SARAM blocks on the 28335 device.
//          This does not include flash or OTP. 
//
//          Keep in mind that L0 and L1 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to 
//          another memory map file which has more memory defined.  
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################
*/

/* ======================================================
// For Code Composer Studio V2.2 and later
// ---------------------------------------
// In addition to this memory linker command file, 
// add the header linker command file directly to the project. 
// The header linker command file is required to link the
// peripheral structures to the proper locations within 
// the memory map.
//
// The header linker files are found in <base>\DSP2833x_Headers\cmd
//   
// For BIOS applications add:      DSP2833x_Headers_BIOS.cmd
// For nonBIOS applications add:   DSP2833x_Headers_nonBIOS.cmd    
========================================================= */

/* ======================================================
// For Code Composer Studio prior to V2.2
// --------------------------------------
// 1) Use one of the following -l statements to include the 
// header linker command file in the project. The header linker
// file is required to link the peripheral structures to the proper 
// locations within the memory map                                    */

/* Uncomment this line to include file only for non-BIOS applications */
/* -l DSP2833x_Headers_nonBIOS.cmd */

/* Uncomment this line to include file only for BIOS applications */
/* -l DSP2833x_Headers_BIOS.cmd */

/* 2) In your project add the path to <base>\DSP2833x_headers\cmd to the
   library search path under project->build options, linker tab, 
   library search path (-i).
/*========================================================= */

/* Define the memory block start/length for the F28335  
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes: 
         Memory blocks on F28335 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.  
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program 
         and/or data. 
         
         L0/L1/L2 and L3 memory blocks are mirrored - that is
         they can be accessed in high memory or low memory.
         For simplicity only one instance is used in this
         linker file. 
         
         Contiguous SARAM memory blocks can be combined 
         if required to create a larger memory block. 
*/


MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to SARAM" bootloader mode      */
   /* BOOT_RSVD is used by the boot ROM for stack.               */
   /* This section is only reserved to keep the BOOT ROM from    */
   /* corrupting this area during the debug process              */
   
   BEGIN      : origin = 0x33FFF6, length = 0x000002     /* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
   RAMM0      : origin = 0x000050, length = 0x0003B0

   RAML       : origin = 0x008000, length = 0x004000    


 /*  ZONE6A     : origin = 0x100000, length = 0x00FC00    /* XINTF zone 6 - program space */ 
   FLASHC     : origin = 0x310000, length = 0x010000     /* on-chip FLASH */
   FLASHD     : origin = 0x320000, length = 0x008000     /* on-chip FLASH */
   FLASHA     : origin = 0x338000, length = 0x007F80     /* on-chip FLASH */     /* on-chip FLASH */
   CSM_RSVD   : origin = 0x33FF80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
   CSM_PWL    : origin = 0x33FFF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA            */
   OTP        : origin = 0x380400, length = 0x000400     /* on-chip OTP */
   ADC_CAL    : origin = 0x380080, length = 0x000009
   RESET      : origin = 0x3FFFC0, length = 0x000002
   IQTABLES   : origin = 0x3FE000, length = 0x000b50
   IQTABLES2  : origin = 0x3FEB50, length = 0x00008c
   FPUTABLES  : origin = 0x3FEBDC, length = 0x0006A0
   BOOTROM    : origin = 0x3FF27C, length = 0x000D44               
   VECTORS    : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM  */
         
PAGE 1 : 
   BOOT_RSVD  : origin = 0x000000, length = 0x000050  
   RAMM1      : origin = 0x000400,  length = 0x000400     /* on-chip RAM block M1 */
   RAML4      : origin = 0x00C000,  length = 0x004000  
  
 /*  ZONE6B     : origin = 0x10FC00,  length = 0x000400     /* XINTF zone 6 - data space */
   ZONE0      : origin = 0x004000,  length = 0x001000
   ZONE6      : origin = 0x0100000, length = 0x040000
   ZONE7      : origin = 0x0200000, length = 0x100000
   FLASHB     : origin = 0x300000,  length = 0x010000     /* on-chip FLASH */
}
 
 
SECTIONS
{
   /* Setup for "boot to SARAM" mode: 
      The codestart section (found in DSP28_CodeStartBranch.asm)
      re-directs execution to the start of user code.  */







   .cinit              : > FLASHA      PAGE = 0
   .pinit              : > FLASHA      PAGE = 0
   .text               : > FLASHA      PAGE = 0
   codestart           : > BEGIN       PAGE = 0
   ramfuncs            : LOAD = FLASHD, 
                         RUN = RAML, 
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
                         PAGE = 0
   csmpasswds          : > CSM_PWL     PAGE = 0
   csm_rsvd            : > CSM_RSVD    PAGE = 0

   .econst             : > FLASHA      PAGE = 0
   .switch             : > FLASHA      PAGE = 0  
   
   .stack           : > RAMM1,     PAGE = 1
   .ebss            : > RAML4,     PAGE = 1
   .esysmem         : > RAMM1,     PAGE = 1
   DARAML5			: > RAML4,     PAGE = 1
   DARAML6			: > RAML4,     PAGE = 1
   DARAML7			: > RAML4,     PAGE = 1

   IQmath           : > FLASHA,     PAGE = 0
   IQmathTables     : > IQTABLES,  PAGE = 0, TYPE = NOLOAD 
   IQmathTables2    : > IQTABLES2, PAGE = 0, TYPE = NOLOAD 
   FPUmathTables    : > FPUTABLES, PAGE = 0, TYPE = NOLOAD 
      
   DMARAML4         : > RAML4,     PAGE = 1
   DMARAML5         : > RAML4,     PAGE = 1
   DMARAML6         : > RAML4,     PAGE = 1
   DMARAML7         : > RAML4,     PAGE = 1
   
   ZONE6DATA        : > ZONE7,    PAGE = 1  

   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used                    */
   vectors          : > VECTORS    PAGE = 0, TYPE = DSECT
   /* Allocate ADC_cal function (pre-programmed by factory into TI reserved memory) */
   .adc_cal     : load = ADC_CAL,   PAGE = 0, TYPE = NOLOAD
    
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
