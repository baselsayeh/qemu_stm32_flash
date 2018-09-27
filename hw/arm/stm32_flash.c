/*
 * STM32 Microcontroller Flash Memory
 * The STM32 family stores its Flash memory at some base address in memory
 * (0x08000000 for medium density devices), and then aliases it to the
 * boot memory space, which starts at 0x00000000 (the System Memory can also
 * be aliased to 0x00000000, but this is not implemented here).  The processor
 * executes the code in the aliased memory at 0x00000000, but we need to
 * implement the "real" flash memory as well.  This "real" flash memory will
 * pass reads through to the memory at 0x00000000, which is where QEMU loads
 * the executable image.  Note that this is opposite of real hardware, where the
 * memory at 0x00000000 passes reads through the "real" flash memory, but it
 * works the same either way.
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "hw/arm/stm32.h"
//#include "exec-memory.h"

uint32_t is_flash_locked = 1;
uint32_t flash_programming_bit = 0;

const char two_block_zeros[1024] = {0};

typedef struct Stm32Flash {
	SysBusDevice busdev;
	MemoryRegion iomem;
	uint32_t size;
} Stm32Flash;

FILE *fp;

static uint64_t stm32_flash_read(void *opaque, hwaddr offset, unsigned size)
{
	uint32_t v = 0;
	char data[8];

	//printf("%lu, %d\n", offset, size);

	fseek(fp, offset, SEEK_SET);

	v = fread(data, 1, size, fp);
	if ( v != size )
		printf("Warning: Error Reading!\n");

	return *((uint64_t *)&data);
}

static void stm32_flash_write(void *opaque, hwaddr offset, uint64_t value,
							  unsigned size)
{

	if (is_flash_locked == 1)
		hw_error("stm32_flash: Attempted to write flash memory while flash is locked!");

	if (flash_programming_bit == 0)
		hw_error("stm32_flash: Attempted to write flash memory while flash programming bit is locked!");

	if (size != 2)
		hw_error("stm32_flash: Attempted to write flash memory not as 2 bytes!");

	/*printf("OK!\n");
	printf("%02x %02x\n", ((unsigned char *)&value)[0], ((unsigned char *)&value)[7]);
	printf("OK!\n");*/

  	fseek(fp, offset, SEEK_SET);
	if (fwrite( (unsigned char *)&value , 1, size, fp) != size)
		printf("warning: error write!\n");

	return;
}

static const MemoryRegionOps stm32_flash_ops = {
	.read = stm32_flash_read,
	.write = stm32_flash_write,
	.endianness = DEVICE_NATIVE_ENDIAN
};

static int stm32_flash_init(SysBusDevice *dev)
{
	//Stm32Flash *s = FROM_SYSBUS(Stm32Flash, dev);
	Stm32Flash *s = STM32_FLASH(dev);

	fp = fopen("writable_area", "r+b");
	if (fp == NULL)
		printf("Warning: cant open file writable_area !\n");

	memory_region_init_io(
			&s->iomem,
			OBJECT(s),
			&stm32_flash_ops,
			&s,
			"stm32-flash",
			0x00010000);
	sysbus_init_mmio(dev, &s->iomem);
	return 0;
}

static Property stm32_flash_properties[] = {
	DEFINE_PROP_UINT32("size", Stm32Flash, size, 0),
	DEFINE_PROP_END_OF_LIST(),
};

static void stm32_flash_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

	k->init = stm32_flash_init;
	dc->props = stm32_flash_properties;
}

static TypeInfo stm32_flash_info = {
	.name          = "stm32-flash",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(Stm32Flash),
	.class_init    = stm32_flash_class_init,
};

static void stm32_flash_register_types(void)
{
	type_register_static(&stm32_flash_info);
}

type_init(stm32_flash_register_types);






//////regs

/*-
 * Copyright (c) 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
/*
 * QEMU crc emulation
 */
//#include "hw/sysbus.h"
#include "hw/arm/stm32.h"

#define R_FLASH_ACR            (0x00 / 4)
#define R_FLASH_KEYR           (0x04 / 4)
#define R_FLASH_OPTKEYR        (0x08 / 4)
#define R_FLASH_SR             (0x0c / 4)
#define R_FLASH_CR             (0x10 / 4)
#define R_FLASH_AR             (0x14 / 4)
#define R_FLASH_RESERVED       (0x18 / 4)
#define R_FLASH_OBR            (0x1c / 4)
#define R_FLASH_WRPR           (0x20 / 4)
#define R_FLASH_MAX            (0x24 / 4)




///
#define FLASH_CR_PG_Pos                     (0U)                               
#define FLASH_CR_PG_Msk                     (0x1U << FLASH_CR_PG_Pos)          /*!< 0x00000001 */
#define FLASH_CR_PG                         FLASH_CR_PG_Msk                    /*!< Programming */
#define FLASH_CR_PER_Pos                    (1U)                               
#define FLASH_CR_PER_Msk                    (0x1U << FLASH_CR_PER_Pos)         /*!< 0x00000002 */
#define FLASH_CR_PER                        FLASH_CR_PER_Msk                   /*!< Page Erase */
#define FLASH_CR_MER_Pos                    (2U)                               
#define FLASH_CR_MER_Msk                    (0x1U << FLASH_CR_MER_Pos)         /*!< 0x00000004 */
#define FLASH_CR_MER                        FLASH_CR_MER_Msk                   /*!< Mass Erase */
#define FLASH_CR_OPTPG_Pos                  (4U)                               
#define FLASH_CR_OPTPG_Msk                  (0x1U << FLASH_CR_OPTPG_Pos)       /*!< 0x00000010 */
#define FLASH_CR_OPTPG                      FLASH_CR_OPTPG_Msk                 /*!< Option Byte Programming */
#define FLASH_CR_OPTER_Pos                  (5U)                               
#define FLASH_CR_OPTER_Msk                  (0x1U << FLASH_CR_OPTER_Pos)       /*!< 0x00000020 */
#define FLASH_CR_OPTER                      FLASH_CR_OPTER_Msk                 /*!< Option Byte Erase */
#define FLASH_CR_STRT_Pos                   (6U)                               
#define FLASH_CR_STRT_Msk                   (0x1U << FLASH_CR_STRT_Pos)        /*!< 0x00000040 */
#define FLASH_CR_STRT                       FLASH_CR_STRT_Msk                  /*!< Start */
#define FLASH_CR_LOCK_Pos                   (7U)                               
#define FLASH_CR_LOCK_Msk                   (0x1U << FLASH_CR_LOCK_Pos)        /*!< 0x00000080 */
#define FLASH_CR_LOCK                       FLASH_CR_LOCK_Msk                  /*!< Lock */
#define FLASH_CR_OPTWRE_Pos                 (9U)                               
#define FLASH_CR_OPTWRE_Msk                 (0x1U << FLASH_CR_OPTWRE_Pos)      /*!< 0x00000200 */
#define FLASH_CR_OPTWRE                     FLASH_CR_OPTWRE_Msk                /*!< Option Bytes Write Enable */
#define FLASH_CR_ERRIE_Pos                  (10U)                              
#define FLASH_CR_ERRIE_Msk                  (0x1U << FLASH_CR_ERRIE_Pos)       /*!< 0x00000400 */
#define FLASH_CR_ERRIE                      FLASH_CR_ERRIE_Msk                 /*!< Error Interrupt Enable */
#define FLASH_CR_EOPIE_Pos                  (12U)                              
#define FLASH_CR_EOPIE_Msk                  (0x1U << FLASH_CR_EOPIE_Pos)       /*!< 0x00001000 */
#define FLASH_CR_EOPIE                      FLASH_CR_EOPIE_Msk                 /*!< End of operation interrupt enable */



#define FLASH_KEY1                          0x45670123U                     /*!< FPEC Key1 */
#define FLASH_KEY2                          0xCDEF89ABU                     /*!< FPEC Key2 */

#define  FLASH_OPTKEY1                       FLASH_KEY1                    /*!< Option Byte Key1 */
#define  FLASH_OPTKEY2                       FLASH_KEY2                    /*!< Option Byte Key2 */

///

typedef struct Stm32FlashRegs {
	SysBusDevice busdev;
	MemoryRegion iomem;

	uint32_t ACR;
	uint32_t KEYR;
	uint32_t OPTKEYR;
	uint32_t SR;
	uint32_t CR;
	uint32_t AR;
	uint32_t RESERVED;
	uint32_t OBR;
	uint32_t WRPR;
} Stm32FlashRegs;

static uint64_t
stm32_flash_regs_read(void *arg, hwaddr addr, unsigned int size)
{
	Stm32FlashRegs *s = arg;

	if (size != 4) {
		qemu_log_mask(LOG_UNIMP, "stm32 flash only supports 4-byte reads\n");
		return 0;
	}

	addr >>= 2;
	if (addr >= R_FLASH_MAX) {
		qemu_log_mask(LOG_GUEST_ERROR, "invalid read stm32 flash register 0x%x\n",
		  (unsigned int)addr << 2);
		return 0;
	}

	switch(addr) {
	case R_FLASH_ACR:
		return s->ACR;
	case R_FLASH_KEYR:
		return s->KEYR;
	case R_FLASH_OPTKEYR:
		return s->OPTKEYR;
	case R_FLASH_SR:
		return s->SR;
	case R_FLASH_CR:
		return s->CR;
	case R_FLASH_AR:
		return s->AR;
	case R_FLASH_RESERVED:
		return s->RESERVED;
	case R_FLASH_OBR:
		return s->OBR;
	case R_FLASH_WRPR:
		return s->WRPR;
	}
	return 0;
}


static void
stm32_flash_regs_write(void *arg, hwaddr addr, uint64_t data, unsigned int size)
{
	Stm32FlashRegs *s = arg;

	/* XXX Check periph clock enable. */
	if (size != 4) {
		qemu_log_mask(LOG_UNIMP, "stm32 flash only supports 4-byte writes\n");
		return;
	}

	addr >>= 2;
	if (addr >= R_FLASH_MAX) {
		qemu_log_mask(LOG_GUEST_ERROR, "invalid write stm32 flash register 0x%x\n",
		  (unsigned int)addr << 2);
		return;
	}
	switch(addr) {
	case R_FLASH_ACR:
		s->ACR = data;
		break;

	case R_FLASH_KEYR:
		if (s->KEYR == FLASH_OPTKEY1 && data == FLASH_OPTKEY2) {
			printf("Flash is unlocked!\n");
			s->CR &= ~FLASH_CR_LOCK;
			is_flash_locked = 0;
		}
		s->KEYR = data;
		break;

	case R_FLASH_OPTKEYR:
		s->OPTKEYR = data;
		break;

	case R_FLASH_SR:
		s->SR = data;
		break;

	case R_FLASH_CR:
		if (is_flash_locked == 0 && (data & FLASH_CR_LOCK)) {
			if (data & FLASH_CR_PG)
				hw_error("stm32_flash: Attempted to write flash lock while flash program is on!");
			printf("Flash is locked!\n");
			//s->CR &= ~FLASH_CR_LOCK;
			is_flash_locked = 1;

		} else if ( (s->CR & FLASH_CR_PER) && (data & FLASH_CR_STRT) ) { //erase
			if (data & FLASH_CR_PG || (data & FLASH_CR_LOCK))
				hw_error("stm32_flash: Attempted to erase flash block while flash program/flash lock is on!");

			printf("start erase\n");
			if ( (s->AR % 1024) == 0 && (s->AR >= 0x10000000) && (s->AR <= 0x10010000) ) {

				fseek(fp, (s->AR) - 0x10000000, SEEK_SET);
				if (fwrite( two_block_zeros , 1, 1024, fp) != 1024)
					printf("warning: error erase!\n");
			} else {
				printf("ADDRESS: %lu\n", s->AR);
				hw_error("stm32_flash: Attempted to erase flash memory page while address is not alligned!");
			}

		} else if (data & FLASH_CR_PG) {
			if (data & FLASH_CR_LOCK || data & FLASH_CR_PER)
				hw_error("stm32_flash: Attempted to write flash program while flash lock/flash erase is on!");
			flash_programming_bit = 1;

		} else if (data & ~FLASH_CR_PG) {
			flash_programming_bit = 0;
		}
			
		s->CR = data;
		break;

	case R_FLASH_AR:
		s->AR = data;
		break;

	case R_FLASH_RESERVED:
		s->RESERVED = data;
		break;

	case R_FLASH_OBR:
		s->OBR = data;
		break;

	case R_FLASH_WRPR:
		s->WRPR = data;
		break;
	
	}

	return;
}

static const MemoryRegionOps stm32_flash_regs_ops = {
	.read = stm32_flash_regs_read,
	.write = stm32_flash_regs_write,
	.endianness = DEVICE_NATIVE_ENDIAN,
	.impl = {
		.min_access_size = 1,
		.max_access_size = 4,
	}
};

static int
stm32_flash_regs_init(SysBusDevice *dev)
{
	Stm32FlashRegs *s = STM32_FLASH_REGS(dev);

	memory_region_init_io(&s->iomem, OBJECT(s), &stm32_flash_regs_ops, s, "flash-regs", 0x400);
	sysbus_init_mmio(dev, &s->iomem);

	return 0;
}

static void
stm32_flash_regs_reset(DeviceState *ds)
{
	Stm32FlashRegs *s = STM32_FLASH_REGS(ds);

	printf("reset is called!\n\n");

	s->ACR = 0;
	s->KEYR = 0;
	s->OPTKEYR = 0;
	s->SR = 0;
	s->CR = 0;
	s->AR = 0;
	s->RESERVED = 0;
	s->OBR = 0;
	s->WRPR = 0;

	is_flash_locked = 1;
}

/*static Property stm32_crc_properties[] = {
	DEFINE_PROP_END_OF_LIST(),
};*/

static void
stm32_flash_regs_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
	sc->init = stm32_flash_regs_init;
	dc->reset = stm32_flash_regs_reset;
	//TODO: fix this: dc->no_user = 1;
	//dc->props = stm32_crc_properties;
}

static const TypeInfo
stm32_crc_info = {
	.name          = "stm32-flash-regs",
	.parent        = TYPE_SYS_BUS_DEVICE,
	.instance_size = sizeof(Stm32FlashRegs),
	.class_init    = stm32_flash_regs_class_init,
};

static void
stm32_crc_register_types(void)
{
	type_register_static(&stm32_crc_info);
}

type_init(stm32_crc_register_types);

