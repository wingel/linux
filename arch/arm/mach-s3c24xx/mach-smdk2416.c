/* linux/arch/arm/mach-s3c2416/mach-hanlin_v3c.c
 *
 * Copyright (c) 2009 Yauhen Kharuzhy <jekhor@gmail.com>,
 *	as part of OpenInkpot project
 * Copyright (c) 2009 Promwad Innovation Company
 *	Yauhen Kharuzhy <yauhen.kharuzhy@promwad.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <video/samsung_fimd.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>
#include <mach/regs-s3c2443-clock.h>
#include <mach/gpio-samsung.h>

#include <linux/platform_data/leds-s3c24xx.h>
#include <linux/platform_data/i2c-s3c2410.h>

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <linux/platform_data/mtd-nand-s3c2410.h>
#include <plat/sdhci.h>
#include <linux/platform_data/usb-ohci-s3c2410.h>
#include <linux/platform_data/usb-s3c2410_udc.h>
#include <linux/platform_data/s3c-hsudc.h>
#include <plat/samsung-time.h>

#include <plat/fb.h>

#include "common.h"
#include "common-smdk.h"

static struct map_desc smdk2416_iodesc[] __initdata = {
#if 0
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
#endif
};

#define UCON (S3C2410_UCON_DEFAULT	| \
		S3C2440_UCON_PCLK	| \
		S3C2443_UCON_RXERR_IRQEN)

#define ULCON (S3C2410_LCON_CS8 | S3C2410_LCON_PNONE)

#define UFCON (S3C2410_UFCON_RXTRIG8	| \
		S3C2410_UFCON_FIFOMODE	| \
		S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg smdk2416_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
#if 0
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON | 0x50,
		.ufcon	     = UFCON,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	}
#endif
};

static struct s3c2410_hcd_info sds7102_usb_info __initdata = {
	.port[0]	= {
		.flags	= S3C_HCDFLG_USED,
	},
	.port[1]	= {
		.flags	= 0,
	},
};

static void smdk2416_hsudc_gpio_init(void)
{
#if 0
	s3c_gpio_setpull(S3C2410_GPH(14), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPF(2), S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(S3C2410_GPH(14), S3C_GPIO_SFN(1));
#endif
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 0);
}

static void smdk2416_hsudc_gpio_uninit(void)
{
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 1);
#if 0
	s3c_gpio_setpull(S3C2410_GPH(14), S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(S3C2410_GPH(14), S3C_GPIO_SFN(0));
#endif
}

static struct s3c24xx_hsudc_platdata smdk2416_hsudc_platdata = {
	.epnum = 9,
	.gpio_init = smdk2416_hsudc_gpio_init,
	.gpio_uninit = smdk2416_hsudc_gpio_uninit,
};

static struct s3c_fb_pd_win smdk2416_fb_win[] = {
	[0] = {
		.default_bpp	= 24,
		.max_bpp	= 32,
		.xres           = 800,
		.yres           = 600,
	},
};

static struct fb_videomode smdk2416_lcd_timing = {
	.pixclock	= 35045,
	.left_margin	= 0x2e,
	.right_margin	= 0x96,
	.upper_margin	= 0x0e,
	.lower_margin	= 0x0c,
	.hsync_len	= 0x01,
	.vsync_len	= 0x0a,
	.xres           = 800,	/* 0x320 */
	.yres           = 600,	/* 0x258 */
};

static void s3c2416_fb_gpio_setup_24bpp(void)
{
	unsigned int gpio;

	for (gpio = S3C2410_GPC(1); gpio <= S3C2410_GPC(4); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPC(8); gpio <= S3C2410_GPC(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPD(0); gpio <= S3C2410_GPD(7); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPD(9); gpio <= S3C2410_GPD(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}

static struct s3c_fb_platdata smdk2416_fb_platdata = {
	.win[0]		= &smdk2416_fb_win[0],
	.vtiming	= &smdk2416_lcd_timing,
	.setup_gpio	= s3c2416_fb_gpio_setup_24bpp,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
};

static struct spi_board_info sds7102_spi_board_info[] = {
	[0] = {
		.modalias	= "ks8851",
		.irq		= IRQ_EINT1,
		.max_speed_hz	= 40*1000*1000,
		.bus_num	= 1,
		.mode		= SPI_MODE_0,
		.chip_select	= 0,
		.controller_data = (void *)S3C2410_GPL(13),
	},
};

static struct spi_gpio_platform_data sds7102_spi = {
	.sck		= S3C2410_GPE(13),
	.mosi		= S3C2410_GPE(12),
	.miso		= S3C2410_GPE(11),
	.num_chipselect = 1,
};

static struct platform_device sds7102_device_spi = {
	.name		= "spi_gpio",
	.id		= 1,
	.dev.platform_data = &sds7102_spi,
};

static struct s3c_sdhci_platdata smdk2416_hsmmc0_pdata __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= S3C2410_GPF(1),
	.ext_cd_gpio_invert	= 1,
};

static struct s3c_sdhci_platdata smdk2416_hsmmc1_pdata __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_NONE,
};

static struct platform_device *smdk2416_devices[] __initdata = {
	&s3c_device_fb,
	&s3c_device_wdt,
	&s3c_device_ohci,
	&s3c_device_usb_hsudc,
#if 0
	&s3c_device_i2c0,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
#endif
	&sds7102_device_spi,
	&s3c2443_device_dma,
};

static void __init smdk2416_init_time(void)
{
	s3c2416_init_clocks(12000000);
	samsung_timer_init();
}

static void __init smdk2416_map_io(void)
{
	u32 epllcon;
	u32 clksrc;

	s3c24xx_init_io(smdk2416_iodesc, ARRAY_SIZE(smdk2416_iodesc));
	s3c24xx_init_uarts(smdk2416_uartcfgs, ARRAY_SIZE(smdk2416_uartcfgs));
	samsung_set_timer_source(SAMSUNG_PWM3, SAMSUNG_PWM4);

	/* The clocks should really be set up by the bootloader, but
	 * since we don't control the bootloder we have to do it here. */

	/* Turn on EPLL and set output divider to 48 MHz */
	epllcon = __raw_readl(S3C2443_EPLLCON);
	printk("%s: EPLLCON 0x%08x\n", __func__, (int)epllcon);
	epllcon &= ~(1<<24);
	epllcon = (epllcon & ~3) | 3;
	__raw_writel(epllcon, S3C2443_EPLLCON);
	epllcon = __raw_readl(S3C2443_EPLLCON);
	printk("%s: EPLLCON 0x%08x\n", __func__, (int)epllcon);

	/* Make esysclk use the EPLL output */
	clksrc = __raw_readl(S3C2443_CLKSRC);
	printk("%s: CLKSRC 0x%08x\n", __func__, (int)clksrc);
	clksrc |= (1<<6);	/* EsysClk selection = EPLL output */
	__raw_writel(clksrc, S3C2443_CLKSRC);
	clksrc = __raw_readl(S3C2443_CLKSRC);
	printk("%s: CLKSRC 0x%08x\n", __func__, (int)clksrc);
}

static void __init smdk2416_machine_init(void)
{
	s3c_i2c0_set_platdata(NULL);
	s3c_fb_set_platdata(&smdk2416_fb_platdata);

	/* FPGA config */
	s3c_gpio_setpull(S3C2410_GPK(5), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPK(7), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPK(13), S3C_GPIO_PULL_UP);

	s3c_ohci_set_platdata(&sds7102_usb_info);
	s3c24xx_hsudc_set_platdata(&smdk2416_hsudc_platdata);

#if 0
	s3c_sdhci0_set_platdata(&smdk2416_hsmmc0_pdata);
	s3c_sdhci1_set_platdata(&smdk2416_hsmmc1_pdata);

	gpio_request(S3C2410_GPB(4), "USBHost Power");
	gpio_direction_output(S3C2410_GPB(4), 1);
#endif

	/* TODO GPB3 can be a PWM signal and give more fine grained control */
	gpio_request(S3C2410_GPB(3), "Display Brightness");
	gpio_direction_output(S3C2410_GPB(3), 0);

	s3c_gpio_setpull(S3C2410_GPF(1), S3C_GPIO_PULL_UP); /* ETH IRQ */
	s3c_gpio_setpull(S3C2410_GPL(13), S3C_GPIO_PULL_UP); /* ETH CS */

	spi_register_board_info(sds7102_spi_board_info,
				ARRAY_SIZE(sds7102_spi_board_info));

	platform_add_devices(smdk2416_devices, ARRAY_SIZE(smdk2416_devices));

	smdk_machine_init();
}

MACHINE_START(SMDK2416, "SMDK2416")
	/* Maintainer: Yauhen Kharuzhy <jekhor@gmail.com> */
	.atag_offset	= 0x100,

	.init_irq	= s3c2416_init_irq,
	.map_io		= smdk2416_map_io,
	.init_machine	= smdk2416_machine_init,
	.init_time	= smdk2416_init_time,
MACHINE_END
