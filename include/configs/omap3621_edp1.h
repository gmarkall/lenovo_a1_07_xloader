/*
 * (C) Copyright 2009
 * Texas Instruments <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * X-Loader Configuration settings for the OMAP3621-EDP1 board.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* serial printf facility takes about 3.5K */
#define CFG_PRINTF

/*
 * High Level Configuration Options
 */
#define CONFIG_ARMCORTEXA8       1    /* This is an ARM V7 CPU core */
#define CONFIG_OMAP              1    /* in a TI OMAP core */
#define CONFIG_OMAP36XX          1    /* which is a 36XX */
#define CONFIG_OMAP34XX          1    /* reuse 34XX */
#define CONFIG_OMAP3430          1    /* reusde 3430 */
#define CONFIG_3630ZOOM3	 1    /* 512 Memory on zoom2 */
#define CONFIG_3621EDP1		 1    /* 128/256 Memory on EDP1 */

/*
  Use in config CONFIG_SDRAM_MT46H32M32LFCM for 1Gb Micron Memory
  #define	CONFIG_SDRAM_MT46H32M32LFCM	 1
  Use in config CONFIG_SDRAM_MT46H64M32LF   for 2Gb Micron Memory
  #define	CONFIG_SDRAM_MT46H64M32LF	 1
*/

#include <asm/arch/cpu.h>        /* get chip and board defs */

/* uncomment it if you need timer based udelay(). it takes about 250 bytes */
#undef CFG_UDELAY

/* Clock Defines */
#define V_OSCK                   26000000  /* Clock output from T2 */

#if (V_OSCK > 19200000)
#define V_SCLK                   (V_OSCK >> 1)
#else
#define V_SCLK                   V_OSCK
#endif

#undef PRCM_CLK_CFG2_400MHZ           
#undef PRCM_CLK_CFG2_332MHZ           
#undef PRCM_CLK_CFG2_266MHZ           

#define PRCM_CLK_CFG2_400MHZ	1	/* VDD2=1.2v - 200MHz DDR */
//#define PRCM_CLK_CFG2_332MHZ	1    /* VDD2=1.15v - 166MHz DDR */
//#define PRCM_CLK_CFG2_266MHZ	1	/* 133MHz DDR */
#define PRCM_PCLK_OPP2           1    /* ARM=500MHz - VDD1=1.20v */

/* Memory type */
#define CFG_3430SDRAM_DDR        1

/* The actual register values are defined in u-boot- mem.h */
/* SDRAM Bank Allocation method */
#undef SDRC_B_R_C
#undef SDRC_B1_R_B0_C
#define SDRC_R_B_C               1

//&*&*&*SJ1_20100825, Add NAND flash boot.
/* Boot type */
//#define CFG_NAND 1

#define NAND_BASE_ADR           NAND_BASE  /* NAND flash */
#ifdef CFG_NAND
/* Use the 512 byte ecc */
#define CFG_SW_ECC_512							1
#define NAND_HW_ROMCODE_ECC_LAYOUT	1
#define CFG_HW_ECC_ROMCODE					1

#define OMAP34XX_GPMC_CS0_SIZE GPMC_SIZE_128M  /* u = ofdon't need so much for nand port */
#define OMAP34XX_GPMC_CS0_MAP NAND_BASE_ADR
#endif
//&*&*&*SJ2_20100825, Add NAND flash boot.

#ifdef CFG_PRINTF

#define CFG_NS16550
#define CFG_NS16550_SERIAL
#define CFG_NS16550_REG_SIZE     (-4)
#define CFG_NS16550_CLK          (48000000)	/* MHz */
#define CFG_NS16550_COM3         OMAP34XX_UART3
#define CFG_NS16550_COM1         OMAP34XX_UART1

/*
 * Select serial console configuration. UART 1 is used as UART3 is muxed
 * with USB and a GPIO already used for EPD screen
 */

#define CONFIG_SERIAL3           1    /* UART3 on board */
#define CONFIG_CONS_INDEX        1

#define CONFIG_BAUDRATE          115200
#define CFG_PBSIZE               256

#endif /* CFG_PRINTF */

/*
 * Miscellaneous configurable options
 */
#define CFG_LOADADDR             0x80008000

#undef	CFG_CLKS_IN_HZ		/* everything, incl board info, in Hz */

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE         (128*1024) /* regular stack */

//&*&*&*SJ1_20100825, Add NAND boot.
#ifdef CFG_NAND
/*-----------------------------------------------------------------------
 * Board NAND Info.
 */
#define CFG_NAND_K9F1G08R0A    /* Micron 16-bit 256MB chip large page NAND chip*/
#define NAND_16BIT


/* NAND is partitioned:
 * 0x00000000 - 0x0007FFFF  Booting Image
 * 0x00080000 - 0x000BFFFF  U-Boot Image
 * 0x000C0000 - 0x000FFFFF  U-Boot Env Data (X-loader doesn't care)
 * 0x00100000 - 0x002FFFFF  Kernel Image
 * 0x00300000 - 0x08000000  depends on application
 */
#define NAND_UBOOT_START         0x0080000 /* Leaving first 4 blocks for x-load */
#define NAND_UBOOT_END           0x00C0000 /* Giving a space of 2 blocks = 256KB */
#define NAND_BLOCK_SIZE          0x20000

#define GPMC_CONFIG              (OMAP34XX_GPMC_BASE+0x50)
#define GPMC_NAND_COMMAND_0      (OMAP34XX_GPMC_BASE+0x7C)
#define GPMC_NAND_ADDRESS_0      (OMAP34XX_GPMC_BASE+0x80)
#define GPMC_NAND_DATA_0         (OMAP34XX_GPMC_BASE+0x84)

#ifdef NAND_16BIT
#define WRITE_NAND_COMMAND(d, adr) \
        do {*(volatile u16 *)GPMC_NAND_COMMAND_0 = d;} while(0)
#define WRITE_NAND_ADDRESS(d, adr) \
        do {*(volatile u16 *)GPMC_NAND_ADDRESS_0 = d;} while(0)
#define WRITE_NAND(d, adr) \
        do {*(volatile u16 *)GPMC_NAND_DATA_0 = d;} while(0)
#define READ_NAND(adr) \
        (*(volatile u16 *)GPMC_NAND_DATA_0)
#define NAND_WAIT_READY()
#define NAND_WP_OFF()  \
        do {*(volatile u32 *)(GPMC_CONFIG) |= 0x00000010;} while(0)
#define NAND_WP_ON()  \
        do {*(volatile u32 *)(GPMC_CONFIG) &= ~0x00000010;} while(0)

#else /* to support 8-bit NAND devices */
#define WRITE_NAND_COMMAND(d, adr) \
        do {*(volatile u8 *)GPMC_NAND_COMMAND_0 = d;} while(0)
#define WRITE_NAND_ADDRESS(d, adr) \
        do {*(volatile u8 *)GPMC_NAND_ADDRESS_0 = d;} while(0)
#define WRITE_NAND(d, adr) \
        do {*(volatile u8 *)GPMC_NAND_DATA_0 = d;} while(0)
#define READ_NAND(adr) \
        (*(volatile u8 *)GPMC_NAND_DATA_0);
#define NAND_WAIT_READY()
#define NAND_WP_OFF()  \
        do {*(volatile u32 *)(GPMC_CONFIG) |= 0x00000010;} while(0)
#define NAND_WP_ON()  \
        do {*(volatile u32 *)(GPMC_CONFIG) &= ~0x00000010;} while(0)

#endif

#define NAND_CTL_CLRALE(adr) 
#define NAND_CTL_SETALE(adr) 
#define NAND_CTL_CLRCLE(adr) 
#define NAND_CTL_SETCLE(adr) 
#define NAND_DISABLE_CE() 
#define NAND_ENABLE_CE() 


#endif  /* CFG_NAND */
//&*&*&*SJ2_20100825, Add NAND flash boot.

/* Enable CONFIG_MMC macro if MMC boot support is required */
/* if loadb and MMC support are enabled together, the size of x-loader
   (code + data) becomes greater than 32K - the size of SRAM. So don't enable
   them together.
 */
#if !defined(START_LOADB_DOWNLOAD)
#define CONFIG_MMC		1
#endif
#if defined(CONFIG_MMC)
	#define CFG_CMD_MMC	1
	#define CFG_CMD_FAT	1
	#define CFG_CMD_MMC_RAW	1
#endif

/* eMMC is Partition as in NAND */
#if defined(CFG_CMD_MMC_RAW)
	#define EMMC_UBOOT_START 0x0080000 /* Leaving first 4 blocks for x-load*/
	#define EMMC_UBOOT_END	 0x00C0000 /* Giving a space of 2 blocks=256KB */
	#define EMMC_BLOCK_SIZE	 0x20000
#endif



#endif /* __CONFIG_H */

