/*
// TI File $Revision: /main/3 $
// Checkin $Date: Agu 1, 2017   13:45:43 $
//
// FILE:    F280013x_FLASH_LIB.cmd
// TITLE:   Linker Command File For F2800137 examples that run out of Flash
//
//          Keep in mind that LS0, and LS1 are protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
*/

/*========================================================= */
/* Define the memory block start/length for the F280013x
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F2800137 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         Contiguous SARAM memory blocks can be combined
         if required to create a larger memory block.
*///#############################################################################
// This file belongs to the DCSM testing project for f280013x device.
// It is intended to be a part of the test directory only.
//#############################################################################
//                              DISCLAIMER
//#############################################################################
// RAM: 36KB/18KW
// TMS320F2800132:   64KB/100Hz       (0~31)
// TMS320F2800133:   64KB/120Hz       (0~31)
// TMS320F2800135:  128KB/120Hz       (0~31)+(32~63)
// TMS320F2800135V: 128KB/120Hz(Vreg) (0~31)+(32~63)
// TMS320F2800137:  256KB/120Hz       (0~31)+(32~63)+(64~127)

// FAST:   000015b6

MEMORY
{
   BEGIN                   : origin = 0x00080000, length = 0x00000002
   BOOT_RSVD               : origin = 0x00000002, length = 0x00000126

   RESET              	   : origin = 0x003FFFC0, length = 0x00000002

   /* Flash sectors */
/* FLASH_BANK0_SEC_0_3     : origin = 0x080002, length = 0x0FFE */ /* on-chip Flash */
   FLASH_BANK0_BOOT	       : origin = 0x080002, length = 0x0FFE	 /* remote update */

/* FLASH_BANK0_SEC_4_5     : origin = 0x081000, length = 0x0800 */ /* on-chip Flash */
   FLASH_BANK0_INDEX1	   : origin = 0x081000, length = 0x0040	    /* constant data */
   FLASH_BANK0_DATAS1	   : origin = 0x081040, length = 0x07C0	    /* constant data */

/* FLASH_BANK0_SEC_6_7     : origin = 0x081800, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_8_9     : origin = 0x082000, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_10_11   : origin = 0x082800, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_FAST	       : origin = 0x081800, length = 0x1800 */

/* FLASH_BANK0_SEC_12_13   : origin = 0x083000, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_14_15   : origin = 0x083800, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_16_17   : origin = 0x084000, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_18_19   : origin = 0x084800, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_20_21   : origin = 0x085000, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_22_23   : origin = 0x085800, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_24_25   : origin = 0x086000, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_26_27   : origin = 0x086800, length = 0x0800 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_28_29   : origin = 0x087000, length = 0x0800 */  /* on-chip Flash */
/* FLASH_BANK0_SEC_30_31   : origin = 0x087800, length = 0x0800 */  /* on-chip Flash */
/* FLASH_BANK0_SEC_32_35   : origin = 0x088000, length = 0x1000 */  /* on-chip Flash */
/* FLASH_BANK0_SEC_36_39   : origin = 0x089000, length = 0x1000 */  /* on-chip Flash */
   FLASH_BANK0_CODES1	   : origin = 0x081800, length = 0x8800    /* motor control code */

/* FLASH_BANK0_SEC_40_43   : origin = 0x08A000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_44_47   : origin = 0x08B000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_48_51   : origin = 0x08C000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_52_55   : origin = 0x08D000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_56_59   : origin = 0x08E000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_60_61   : origin = 0x08F000, length = 0x0800 */ /* on-chip Flash */
   FLASH_BANK0_CODES2 	   : origin = 0x08A000, length = 0x5800    /* control code */

/* FLASH_BANK0_SEC_62_63   : origin = 0x08F800, length = 0x0800 */ /* on-chip Flash */
   FLASH_BANK0_INDEX2	   : origin = 0x08F800, length = 0x0040	    /* constant data */
   FLASH_BANK0_DATAS2	   : origin = 0x08F840, length = 0x07C0	    /* constant data */

/* FLASH_BANK0_SEC_64_67   : origin = 0x090000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_64_71   : origin = 0x091000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_72_79   : origin = 0x092000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_72_79   : origin = 0x093000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_80_87   : origin = 0x094000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_80_87   : origin = 0x095000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_88_91   : origin = 0x096000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_92_95   : origin = 0x097000, length = 0x1000 */ /* on-chip Flash */
   FLASH_BANK0_CODES3 	   : origin = 0x090000, length = 0x8000    /* control code */

/* FLASH_BANK0_SEC_96_99   : origin = 0x098000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_100_103 : origin = 0x099000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_104_107 : origin = 0x09A000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_107_111 : origin = 0x09B000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_112_115 : origin = 0x09C000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_116_119 : origin = 0x09D000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_120_123 : origin = 0x09E000, length = 0x1000 */ /* on-chip Flash */
/* FLASH_BANK0_SEC_124_127 : origin = 0x09F000, length = 0x0FF0 */ /* on-chip Flash */
   FLASH_BANK0_CODES4	   : origin = 0x098000, length = 0x7FF0    /* control code */


/* FLASH_BANK0_SEC_127_RSVD : origin = 0x0A0FF0, length = 0x0010 */  /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RAMM0DP        	  : origin = 0x00000128, length = 0x00000018	/* pointer */
   RAMM0S          	  : origin = 0x00000140, length = 0x00000340	/* stack */
   RAMM1D         	  : origin = 0x00000480, length = 0x00000378	/* on-chip RAM block M1 */
   RAMM1_RSVD         : origin = 0x000007F8, length = 0x00000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */


   RAMLS0F        	  : origin = 0x00008000, length = 0x00000600	/* Reserve for FAST */
   RAMLS0D        	  : origin = 0x00008600, length = 0x00000A00

/* RAMLS1             : origin = 0x0000A000, length = 0x00001FF8 */
   RAMLS1P            : origin = 0x00009000, length = 0x00002FF8
// RAMLS1_RSVD        : origin = 0x0000BFF8, length = 0x00000008 /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
}


SECTIONS
{
   .reset           : > RESET,  		    TYPE = DSECT /* not used, */
   codestart		: > BEGIN,     			ALIGN(4)

   GROUP
   {
   	.TI.ramfunc {
#if defined(SFRA_ENABLE)
       -l sfra_f32_tmu_eabi.lib<sfra_f32_tmu_collect.obj> (.text)
       -l sfra_f32_tmu_eabi.lib<sfra_f32_tmu_inject.obj>  (.text)
#endif
   			   }
    ramfuncs
   	dclfuncs
    dcl32funcs
   }                    LOAD >  FLASH_BANK0_CODES1,
                        RUN  >  RAMLS1P,
		                LOAD_START(RamfuncsLoadStart),
		                LOAD_SIZE(RamfuncsLoadSize),
		                LOAD_END(RamfuncsLoadEnd),
		                RUN_START(RamfuncsRunStart),
		                RUN_SIZE(RamfuncsRunSize),
		                RUN_END(RamfuncsRunEnd),
                        ALIGN(8)

    ctrlfuncs 		:   LOAD >  FLASH_BANK0_CODES1,
                        RUN  >  RAMLS1P,
					    LOAD_START(loadStart_ctrlfuncs),
					    LOAD_END(loadEnd_ctrlfuncs),
					    LOAD_SIZE(loadSize_ctrlfuncs),
					    RUN_START(runStart_ctrlfuncs),
					    RUN_END(runEnd_ctrlfuncs),
					    RUN_SIZE(runSize_ctrlfuncs),
					    ALIGN(8)
/* *
    lfufuncs {
#if defined(_LFU_ENABLE)
			 --library=FAPI_F280013x_EABI_v2.00.10.lib (.text)
#endif
			  }         LOAD >  FLASH_BANK0_BOOT,
                        RUN  >  RAMLS1P,
	                	LOAD_START(loadStart_lfufuncs),
	              		LOAD_END(loadEnd_lfufuncs),
	              		LOAD_SIZE(loadSize_lfufuncs),
	                 	RUN_START(runStart_lfufuncs),
	                    RUN_END(runEnd_lfufuncs),
	                    RUN_SIZE(runSize_lfufuncs),
	                    ALIGN(8)
* */

/* */
    lfufuncs 		    LOAD >  FLASH_BANK0_BOOT,
                        RUN  >  RAMLS1P,
	                	LOAD_START(loadStart_lfufuncs),
	              		LOAD_END(loadEnd_lfufuncs),
	              		LOAD_SIZE(loadSize_lfufuncs),
	                 	RUN_START(runStart_lfufuncs),
	                    RUN_END(runEnd_lfufuncs),
	                    RUN_SIZE(runSize_lfufuncs),
	                    ALIGN(8)
/* */

    fastfuncs 		: > FLASH_BANK0_CODES2
    {
#if defined(_FULL_FAST_LIB)
         --library=fast_full_lib.lib (.text)
#endif
#if defined(_SIMPLE_FAST_LIB)
         --library=fast_simple_lib.lib (.text)
#endif
    }

   .text            : > FLASH_BANK0_CODES1,		ALIGN(8)
   .cinit           : > FLASH_BANK0_CODES1,		ALIGN(8)
   .switch          : > FLASH_BANK0_CODES1,		ALIGN(8)
   .binit       	: > FLASH_BANK0_CODES1,		ALIGN(8)
   .ovly       		: > FLASH_BANK0_CODES1,		ALIGN(8)
   .cio				: > FLASH_BANK0_CODES1,		ALIGN(8)
   .pinit           : > FLASH_BANK0_CODES1,		ALIGN(8)
   .const           : > FLASH_BANK0_CODES1,  	ALIGN(8)
   .init_array      : > FLASH_BANK0_CODES1,		ALIGN(8)

   .stack           : > RAMM0S
   .bss             : > RAMM1D
   .bss:output      : > RAMM1D
   .bss:cio         : > RAMM1D
   .data            : > RAMM1D
   .sysmem          : > RAMM1D

	/*  Allocate IQ math areas: */
    IQmath          : > FLASH_BANK0_CODES1, 	ALIGN(8)
    IQmathTables    : > FLASH_BANK0_CODES1,		ALIGN(8)

    guifuncs        : > FLASH_BANK0_BOOT,		ALIGN(8)

   	prms_index1		: > FLASH_BANK0_INDEX1
   	prms_datas1 	: > FLASH_BANK0_DATAS1
    prms_index2     : > FLASH_BANK0_INDEX2
    prms_datas2     : > FLASH_BANK0_DATAS2

    sysfuncs        : > FLASH_BANK0_CODES3,		ALIGN(8)
    safefuncs       : > FLASH_BANK0_CODES4,		ALIGN(8)
    aicfuncs        : > FLASH_BANK0_CODES4,	    ALIGN(8)

    ptr_data            : >  RAMM0DP		/* Will not be cleared to zero */

	sta_data            : >  RAMM0S

    est_data            : >  RAMLS0F,
                             LOAD_START(loadStart_est_data),
                             LOAD_END(loadEnd_est_data),
                             LOAD_SIZE(loadSize_est_data)

//#####BEGIN_GUIDEMO#####
    gui_data            : >  RAMM1D,
                             LOAD_START(loadStart_gui_data),
                             LOAD_END(loadEnd_gui_data),
                             LOAD_SIZE(loadSize_gui_data)
//#####END_GUIDEMO#####
    hal_data            : >  RAMM1D,
                             LOAD_START(loadStart_hal_data),
                             LOAD_END(loadEnd_hal_data),
                             LOAD_SIZE(loadSize_hal_data)

    user_data           : >  RAMM1D,
                             LOAD_START(loadStart_user_data),
                             LOAD_END(loadEnd_user_data),
                             LOAD_SIZE(loadSize_user_data)

    foc_data            : >  RAMLS0D,
                             LOAD_START(loadStart_foc_data),
                             LOAD_END(loadEnd_foc_data),
                             LOAD_SIZE(loadSize_foc_data)

    sys_data            : >  RAMLS0D,
                             LOAD_START(loadStart_sys_data),
                             LOAD_END(loadEnd_sys_data),
                             LOAD_SIZE(loadSize_sys_data)

    ctrl_data           : >  RAMLS0D,
                             LOAD_START(loadStart_ctrl_data),
                             LOAD_END(loadEnd_ctrl_data),
                             LOAD_SIZE(loadSize_ctrl_data)

    SFRA_F32_Data       : >  RAMLS0D,
                             LOAD_START(loadStart_SFRA_F32_Data),
                             LOAD_END(loadEnd_SFRA_F32_Data),
                             LOAD_SIZE(loadSize_SFRA_F32_Data)

    datalog_data        : >  RAMLS0D,
                             LOAD_START(loadStart_datalog_data),
                             LOAD_END(loadEnd_datalog_data),
                             LOAD_SIZE(loadSize_datalog_data)
//#####BEGIN_AICTEST#####
    aic_data            : >  RAMLS0D,
                             LOAD_START(loadStart_aic_data),
                             LOAD_END(loadEnd_aic_data),
                             LOAD_SIZE(loadSize_aic_data)
//#####END_AICTEST#####
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
