/*
 * (C) Copyright 2004 Texas Instruments
 * Jian Zhang <jzhang@ti.com>
 *
 *  Samsung K9F1G08R0AQ0C NAND chip driver for an OMAP2420 board
 * 
 * This file is based on the following u-boot file:
 *	common/cmd_nand.c
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>

#ifdef CFG_NAND_K9F1G08R0A

#define K9F1G08R0A_MFR		0xec	/* Samsung */
#define K9F1G08R0A_ID		0xa1	/* part # */

/* Since Micron and Samsung parts are similar in geometry and bus width
 * we can use the same driver. Need to revisit to make this file independent
 * of part/manufacturer
 */
#define MT29F1G_MFR		0x2c	/* Micron */
#define MT29F1G_ID		0xa1	/* x8, 1GiB */
#define MT29F2G_ID		0xba	/* x16, 2GiB */
#define MT29F4G_ID		0xbc	/* x16, 4GiB */

#define ADDR_COLUMN		1
#define ADDR_PAGE		2
#define ADDR_COLUMN_PAGE	(ADDR_COLUMN | ADDR_PAGE)

#define ADDR_OOB		(0x4 | ADDR_COLUMN_PAGE)

#define PAGE_SIZE		2048
#define OOB_SIZE		64
#define MAX_NUM_PAGES		64

#ifdef NAND_HW_ROMCODE_ECC_LAYOUT
#define __raw_ecc_readl(a)	(*(volatile unsigned int *)(a))
#define __raw_ecc_writel(v, a)	(*(volatile unsigned int *)(a) = (v))
/* Register definitions */
#define GPMC_BASE_ADDR		OMAP34XX_GPMC_BASE

#define ECCCLEAR		(0x1 << 8)
#define ECCRESULTREG1		(0x1 << 0)
#define ECCSIZE512BYTE		0xFF
#define ECCSIZE1		(ECCSIZE512BYTE << 22)
#define ECCSIZE0		(ECCSIZE512BYTE << 12)
#define ECCSIZE0SEL		(0x000 << 0)

#define ECC_BLOCK_SIZE		512
#ifdef NAND_16BIT
#define DEV_WIDTH		1
static u_char ecc_pos[] = { 2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13
};
#else
#define DEV_WIDTH		0
static u_char ecc_pos[] = { 1, 2, 3, 4, 5, 6, 7, 8,
	9, 10, 11, 12
};
#endif

#else
#ifdef NAND_16BIT
#define DEV_WIDTH		1
#else
#define DEV_WIDTH		0
#endif

#ifdef CFG_SW_ECC_512
#define ECC_BLOCK_SIZE 512
#else
#define ECC_BLOCK_SIZE 256
#endif
/* JFFS2 large page layout for 3-byte ECC per 256 bytes ECC layout */
/* This is the only SW ECC supported by u-boot. So to load u-boot
 * this should be supported */
static u_char ecc_pos[] = {40, 41, 42, 43, 44, 45, 46, 47,
			   48, 49, 50, 51, 52, 53, 54, 55,
			   56, 57, 58, 59, 60, 61, 62, 63};

#endif

#define ECC_SIZE	(sizeof(ecc_pos))
#define ECC_STEPS	3
#define ECC_CHECK_ENABLE

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay (unsigned long loops)
{
	__asm__ volatile ("1:\n"
					"subs %0, %0, #1\n"
					"bne 1b" : "=r" (loops) : "0" (loops));
}

static int nand_read_page(u_char *buf, ulong page_addr);
static int nand_read_oob(u_char *buf, ulong page_addr);

static unsigned long chipsize = (256 << 20);

/* NanD_Command: Send a flash command to the flash chip */
static int NanD_Command(unsigned char command)
{
	NAND_CTL_SETCLE(NAND_ADDR);

	WRITE_NAND_COMMAND(command, NAND_ADDR);
	NAND_CTL_CLRCLE(NAND_ADDR);

	if (command == NAND_CMD_RESET) {
		unsigned char ret_val;
		NanD_Command(NAND_CMD_STATUS);
		do {
			ret_val = READ_NAND(NAND_ADDR);	/* wait till ready */
		} while ((ret_val & 0x40) != 0x40);
	}

	NAND_WAIT_READY();
	return 0;
}


/* NanD_Address: Set the current address for the flash chip */
static int NanD_Address(unsigned int numbytes, unsigned long ofs)
{
	uchar u;

	NAND_CTL_SETALE(NAND_ADDR);

	if (numbytes == ADDR_COLUMN || numbytes == ADDR_COLUMN_PAGE
				|| numbytes == ADDR_OOB)
	{
		ushort col = ofs;

		u = col & 0xff;
		WRITE_NAND_ADDRESS(u, NAND_ADDR);

		u = (col >> 8) & 0x07;
		if (numbytes == ADDR_OOB)
			u = u | ((DEV_WIDTH == 1) ? (1 << 2) : (1 << 3));
		WRITE_NAND_ADDRESS(u, NAND_ADDR);
	}

	if (numbytes == ADDR_PAGE || numbytes == ADDR_COLUMN_PAGE
				|| numbytes == ADDR_OOB)
	{
		u = (ofs >> 11) & 0xff;
		WRITE_NAND_ADDRESS(u, NAND_ADDR);
		u = (ofs >> 19) & 0xff;
		WRITE_NAND_ADDRESS(u, NAND_ADDR);

		/* One more address cycle for devices > 128MiB */
		if (chipsize > (128 << 20)) {
			u = (ofs >> 27) & 0xff;
			WRITE_NAND_ADDRESS(u, NAND_ADDR);
		}
	}

	NAND_CTL_CLRALE(NAND_ADDR);

	NAND_WAIT_READY();
	return 0;
}

/* read chip mfr and id
 * return 0 if they match board config
 * return 1 if not
 */
int nand_chip()
{
	int mfr, id;

	NAND_ENABLE_CE();

	if (NanD_Command(NAND_CMD_RESET)) {
		printf("Err: RESET\n");
		NAND_DISABLE_CE();
		return 1;
	}
 
	if (NanD_Command(NAND_CMD_READID)) {
		printf("Err: READID\n");
		NAND_DISABLE_CE();
		return 1;
	}
 
	NanD_Address(ADDR_COLUMN, 0);

	mfr = READ_NAND(NAND_ADDR);
	id = READ_NAND(NAND_ADDR);

	NAND_DISABLE_CE();

	/* Check Micron part or Samsung part */
	if ((mfr == MT29F1G_MFR && (id == MT29F1G_ID ||
		id == MT29F2G_ID || id == MT29F4G_ID)) ||
		(mfr == K9F1G08R0A_MFR && (id == K9F1G08R0A_ID)))
		return 0;

	return 1;
}

/* read a block data to buf
 * return 1 if the block is bad or ECC error can't be corrected for any page
 * return 0 on sucess
 */
int nand_read_block(unsigned char *buf, ulong block_addr)
{
	int i, offset = 0;

#ifdef ECC_CHECK_ENABLE
	u8 oob_buf[OOB_SIZE];

	/* check bad block */
	/* 0th word in spare area needs be 0xff */
	if (nand_read_oob(oob_buf, block_addr) || (oob_buf[0] & 0xff) != 0xff) {
		printf("Skipped bad block at 0x%x\n", block_addr);
		return 1;	/* skip bad block */
	}
#endif
	/* read the block page by page*/
	for (i = 0; i < MAX_NUM_PAGES; i++) {
		if (nand_read_page(buf + offset, block_addr + offset))
			return 1;
		offset += PAGE_SIZE;
	}

	return 0;
}

#ifdef NAND_HW_ROMCODE_ECC_LAYOUT
/*
 * omap_hwecc_init - Initialize the Hardware ECC for NAND flash in
 *  GPMC controller
 *
 */
static void omap_hwecc_init(void)
{
	/*
	 * Init ECC Control Register
	 * Clear all ECC | Enable Reg1
	 */
	__raw_ecc_writel(ECCCLEAR | ECCRESULTREG1,
				GPMC_BASE_ADDR + GPMC_ECC_CONTROL);
	__raw_ecc_writel(ECCSIZE1 | ECCSIZE0 | ECCSIZE0SEL,
				GPMC_BASE_ADDR + GPMC_ECC_SIZE_CONFIG);
	__raw_ecc_writel(0x1 | (0 << 1) | (DEV_WIDTH << 7),
				GPMC_BASE_ADDR + GPMC_ECC_CONFIG);
}

/*
 * gen_true_ecc - This function will generate true ECC value, which
 * can be used when correcting data read from NAND flash memory core
 *
 * @ecc_buf:	buffer to store ecc code
 *
 * @return:	re-formatted ECC value
 */
static uint32_t gen_true_ecc(uint8_t *ecc_buf)
{
	return ecc_buf[0] | (ecc_buf[1] << 16) | ((ecc_buf[2] & 0xF0) << 20) |
		((ecc_buf[2] & 0x0F) << 8);
}

/*
 *  omap_calculate_ecc - Generate non-inverted ECC bytes.
 *
 *  Using noninverted ECC can be considered ugly since writing a blank
 *  page ie. padding will clear the ECC bytes. This is no problem as
 *  long nobody is trying to write data on the seemingly unused page.
 *  Reading an erased page will produce an ECC mismatch between
 *  generated and read ECC bytes that has to be dealt with separately.
 *  E.g. if page is 0xFF (fresh erased), and if HW ECC engine within GPMC
 *  is used, the result of read will be 0x0 while the ECC offsets of the
 *  spare area will be 0xFF which will result in an ECC mismatch.
 *  @dat:	unused
 *  @ecc_code:	ecc_code buffer
 */
static int omap_calculate_ecc(const uint8_t *dat, uint8_t *ecc_code)
{
	u_int32_t val;

	/* Start Reading from HW ECC1_Result = 0x200 */
	val = __raw_ecc_readl(GPMC_BASE_ADDR + GPMC_ECC1_RESULT);

	ecc_code[0] = val & 0xFF;
	ecc_code[1] = (val >> 16) & 0xFF;
	ecc_code[2] = ((val >> 8) & 0x0F) | ((val >> 20) & 0xF0);

	/*
	 * Stop reading anymore ECC vals and clear old results
	 * enable will be called if more reads are required
	 */
	__raw_ecc_writel(0x000, GPMC_BASE_ADDR + GPMC_ECC_CONFIG);

	return 0;
}

static int hweight32(unsigned int val)
{
	u_char count = 0;
	for ( ; val; count++)
		val &= (val - 1);
	return count;
}

/*
 * omap_correct_data - Compares the ecc read from nand spare area with ECC
 * registers values and corrects one bit error if it has occured
 * Further details can be had from OMAP TRM and the following selected links:
 * http://en.wikipedia.org/wiki/Hamming_code
 * http://www.cs.utexas.edu/users/plaxton/c/337/05f/slides/ErrorCorrection-4.pdf
 *
 * @dat:		page data
 * @read_ecc:		ecc read from nand flash
 * @calc_ecc:		ecc read from ECC registers
 *
 * @return 0 if data is OK or corrected, else returns -1
 */
static int omap_correct_data(uint8_t *dat,
			     uint8_t *read_ecc, uint8_t *calc_ecc)
{
	uint32_t orig_ecc, new_ecc, res, hm;
	uint16_t parity_bits, byte;
	uint8_t bit;

	/* Regenerate the orginal ECC */
	orig_ecc = gen_true_ecc(read_ecc);
	new_ecc = gen_true_ecc(calc_ecc);
	/* Get the XOR of real ecc */
	res = orig_ecc ^ new_ecc;
	if (res) {
		/* Get the hamming width */
		hm = hweight32(res);
		/* Single bit errors can be corrected! */
		if (hm == 12) {
			/* Correctable data! */
			parity_bits = res >> 16;
			bit = (parity_bits & 0x7);
			byte = (parity_bits >> 3) & 0x1FF;
			/* Flip the bit to correct */
			dat[byte] ^= (0x1 << bit);
		} else if (hm == 1) {
			printf("Error: Ecc is wrong\n");
			/* ECC itself is corrupted */
			return 2;
		} else {
			/*
			 * hm distance != parity pairs OR one, could mean 2 bit
			 * error OR potentially be on a blank page..
			 * orig_ecc: contains spare area data from nand flash.
			 * new_ecc: generated ecc while reading data area.
			 * Note: if the ecc = 0, all data bits from which it was
			 * generated are 0xFF.
			 * The 3 byte(24 bits) ecc is generated per 512byte
			 * chunk of a page. If orig_ecc(from spare area)
			 * is 0xFF && new_ecc(computed now from data area)=0x0,
			 * this means that data area is 0xFF and spare area is
			 * 0xFF. A sure sign of a erased page!
			 */
			if ((orig_ecc == 0x0FFF0FFF) && (new_ecc == 0x00000000))
				return 0;
			printf("Error: Bad compare! failed\n");
			/* detected 2 bit error */
			return -1;
		}
	}
	return 0;
}

#define NAND_ECC_INIT() omap_hwecc_init()
#define NAND_ECC_CALC(data_buf, ecc_buf) omap_calculate_ecc(data_buf, ecc_buf)
#define NAND_ECC_CORRECT(data_buf, ecc_gen, ecc_old)\
	omap_correct_data(data_buf, ecc_gen, ecc_old)
#else
/* Software ECC APIs */
#define NAND_ECC_INIT()
#define NAND_ECC_CALC(data_buf, ecc_buf) nand_calculate_ecc(data_buf, ecc_buf)
#define NAND_ECC_CORRECT(data_buf, ecc_gen, ecc_old)\
	nand_correct_data(data_buf, ecc_gen, ecc_old)
#endif /* NAND_HW_ROMCODE_ECC_LAYOUT */

/* read a page with ECC */
static int nand_read_page(u_char *buf, ulong page_addr)
{
#ifdef ECC_CHECK_ENABLE
	u_char ecc_code[ECC_SIZE];
	u_char ecc_calc[ECC_SIZE];
	u_char oob_buf[OOB_SIZE];
#endif
	u16 val;
	int cntr;
	int len;
	int count;

#ifdef NAND_16BIT
	u16 *p = (u16 *) buf;
#else
	u_char *p = (u_char *) buf;
#endif

	NAND_ENABLE_CE();
	NanD_Command(NAND_CMD_READ0);
	NanD_Address(ADDR_COLUMN_PAGE, page_addr);
	NanD_Command(NAND_CMD_READSTART);
	NAND_WAIT_READY();

	/* A delay seems to be helping here. needs more investigation */
	delay(10000);
	len = (DEV_WIDTH == 1) ? ECC_BLOCK_SIZE >> 1 : ECC_BLOCK_SIZE;

	/* Read in chunks of data. */
	for (count = 0; count < ECC_SIZE; count += ECC_STEPS) {
		NAND_ECC_INIT();
		for (cntr = 0; cntr < len; cntr++) {
			*p++ = READ_NAND(NAND_ADDR);
			delay(10);
		}
		NAND_ECC_CALC((p - ECC_BLOCK_SIZE), &ecc_calc[count]);
	}

#ifdef ECC_CHECK_ENABLE
#ifdef NAND_16BIT
	p = (u16 *) oob_buf;
#else
	p = (u_char *) oob_buf;
#endif
	len = (DEV_WIDTH == 1) ? OOB_SIZE >> 1 : OOB_SIZE;
	for (cntr = 0; cntr < len; cntr++) {
		*p++ = READ_NAND(NAND_ADDR);
		delay(10);
	}
	count = 0;
	NAND_DISABLE_CE();	/* set pin high */

	/* Pick the ECC bytes out of the oob data */
	for (cntr = 0; cntr < ECC_SIZE; cntr++)
		ecc_code[cntr] =  oob_buf[ecc_pos[cntr]];

	for (count = 0, cntr = 0; cntr < PAGE_SIZE / ECC_BLOCK_SIZE;
					cntr++, count += ECC_STEPS) {
		if (NAND_ECC_CORRECT(buf, &ecc_code[count], &ecc_calc[count])
			== -1) {
			printf("ECC Failed, page 0x%08x\n", page_addr);
			for (val = 0; val < ECC_BLOCK_SIZE; val++)
				printf("%x ", buf[val]);
			printf(" Hang!!!\n");
			for (;;);
			return 1;
		}
		buf += ECC_BLOCK_SIZE;
		page_addr += ECC_BLOCK_SIZE;
	}
#endif	
	return 0;
}

/* read from the 16 bytes of oob data that correspond to a 512 / 2048 byte page.
 */
static int nand_read_oob(u_char *buf, ulong page_addr)
{
	int cntr;
	int len;

#ifdef NAND_16BIT
	u16 *p = (u16 *) buf;
#else
	u_char *p = (u_char *) buf;
#endif
	len = (DEV_WIDTH == 1) ? OOB_SIZE >> 1 : OOB_SIZE;

	NAND_ENABLE_CE();	/* set pin low */
	NanD_Command(NAND_CMD_READ0);
	NanD_Address(ADDR_OOB, page_addr);
	NanD_Command(NAND_CMD_READSTART);
	NAND_WAIT_READY();

	/* A delay seems to be helping here. needs more investigation */
	delay(10000);
	for (cntr = 0; cntr < len; cntr++)
		*p++ = READ_NAND(NAND_ADDR);

	NAND_WAIT_READY();
	NAND_DISABLE_CE();	/* set pin high */

	return 0;
}

#endif
