/*
 * Driver for OV5640 CMOS Image Sensor from Omnivision
 *
 * Copyright (C) 2011, Bastian Hecht <hechtb@gmail.com>
 *
 * Based on Sony IMX074 Camera Driver
 * Copyright (C) 2010, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>

#include <media/soc_camera.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>
#include <linux/io.h>


/* OV5640 registers */
#define REG_CHIP_ID_HIGH		0x300A
#define REG_CHIP_ID_LOW			0x300B

#define REG_WINDOW_START_X_HIGH		0x3800
#define REG_WINDOW_START_X_LOW		0x3801
#define REG_WINDOW_START_Y_HIGH		0x3802
#define REG_WINDOW_START_Y_LOW		0x3803
#define REG_WINDOW_WIDTH_HIGH		0x3804
#define REG_WINDOW_WIDTH_LOW		0x3805
#define REG_WINDOW_HEIGHT_HIGH		0x3806
#define REG_WINDOW_HEIGHT_LOW		0x3807
#define REG_OUT_WIDTH_HIGH		0x3808
#define REG_OUT_WIDTH_LOW		0x3809
#define REG_OUT_HEIGHT_HIGH		0x380A
#define REG_OUT_HEIGHT_LOW		0x380B
#define REG_OUT_TOTAL_WIDTH_HIGH	0x380C
#define REG_OUT_TOTAL_WIDTH_LOW		0x380D
#define REG_OUT_TOTAL_HEIGHT_HIGH	0x380E
#define REG_OUT_TOTAL_HEIGHT_LOW	0x380F
#define REG_OUTPUT_FORMAT		0x4300
#define REG_ISP_CTRL_01			0x5001
#define REG_AVG_WINDOW_END_X_HIGH	0x5682
#define REG_AVG_WINDOW_END_X_LOW	0x5683
#define REG_AVG_WINDOW_END_Y_HIGH	0x5686
#define REG_AVG_WINDOW_END_Y_LOW	0x5687


#define ANTIBANDING_AUTO        0
#define ANTIBANDING_50HZ        1
#define ANTIBANDING_60HZ        2
#define ANTIBANDINT_OFF         3
//#define DEFAULT_ANTIBANDING     ANTIBANDING_50HZ

struct regval_list {
	u16 reg_num;
	u8 value;
};

/**
 * 720p@30 v1
 */
static struct regval_list ov5640_default_regs_init[] = {
        {0x3103, 0x11}, {0x3008, 0x82}, // software reset, and delay 10ms
        {0x3008, 0x42},//Bit[7]: Software reset    Bit[6]: Software power down
        {0x3103, 0x03},//Select system input clock From PLL
        {0x4005, 0x1a},// BLC always update
        {0x4740, 0x20},  //VSYNC polarity active high
        {0x3017, 0xff},
        {0x3018, 0xff},
        {0x3034, 0x1a},
        {0x3035, 0x11},//set frame rate
        {0x3036, 0x69},// default:0x69          SC PLL CONTRL2
        {0x3037, 0x13},
        {0x3108, 0x01},
        {0x3630, 0x36},
        {0x3631, 0x0e},
        {0x3632, 0xe2},
        {0x3633, 0x12},
        {0x3621, 0xe0},
        {0x3704, 0xa0},
        {0x3703, 0x5a},
        {0x3715, 0x78},
        {0x3717, 0x01},
        {0x370b, 0x60},
        {0x3705, 0x1a},
        {0x3905, 0x02},
        {0x3906, 0x10},
        {0x3901, 0x0a},
        {0x3731, 0x12},
        {0x3600, 0x08},
        {0x3601, 0x33},
        {0x302d, 0x60},
        {0x3620, 0x52},
        {0x371b, 0x20},
        {0x471c, 0x50},
        {0x3a13, 0x43},
        {0x3a18, 0x00},
        {0x3a19, 0xF8},//0xb0
        {0x3635, 0x13},
        {0x3636, 0x03},
        {0x3634, 0x40},
        {0x3622, 0x01},// 50/60Hz detection     50/60Hz 灯光条纹过滤
        {0x3c01, 0x34}, // Band auto, bit[7]
        {0x3c00, 0x00},
        {0x3c04, 0x28},// threshold low sum
        {0x3c05, 0x98},// threshold high sum
        {0x3c06, 0x00},// light meter 1 threshold[15:8]
        {0x3c07, 0x07},// light meter 1 threshold[7:0]
        {0x3c08, 0x00},// light meter 2 threshold[15:8]
        {0x3c09, 0x1c},// light meter 2 threshold[7:0]
        {0x3c0a, 0x9c},// sample number[15:8]
        {0x3c0b, 0x40},// sample number[7:0]
        {0x3820, 0x43},// //Timing control  Bit[2]: ISP vflip    Bit[1]: Sensor vflip  Bit[7:3]: Debug mode
        {0x3821, 0x05},//Bit[7:6]: Debug mode Bit[5]: JPEG enable  Bit[4:3]: Debug mode Bit[2]: ISP mirror Bit[1]: Sensor mirror Bit[0]: Horizontal binning enable
        {0x3814, 0x31},// timing X inc
        {0x3815, 0x31},// timing Y inc
        {0x3800, 0x00},// HS
        {0x3801, 0x00},// HS
        {0x3802, 0x00},// VS
        {0x3803, 0xfa},// VS
        {0x3804, 0x0a},// HW (HE)
        {0x3805, 0x3f},// HW (HE)
        {0x3806, 0x06},// VH (VE)
        {0x3807, 0xa9},// VH (VE)
        {0x3808, 0x05}, // DVPHO
        {0x3809, 0x00},// DVPHO
        {0x380a, 0x02}, // DVPVO
        {0x380b, 0xd0},// DVPVO
        {0x3503, 0x00},// AEC/AGC on
        {0x380c, 0x07},// HTS
        {0x380d, 0x64},// HTS
        {0x380e, 0x02},// VTS
        {0x380f, 0xe4},// VTS
        {0x3810, 0x00},//Timing Hoffset[11:8]
        {0x3811, 0x10},// Timing Hoffset[7:0]
        {0x3812, 0x00},// Timing Voffset[10:8]
        {0x3813, 0x04},// timing V offset
        {0x3618, 0x00},
        {0x3612, 0x29},
        {0x3708, 0x64},
        {0x3709, 0x52},
        {0x370c, 0x03},
        {0x3a02, 0x02},// 60Hz max exposure,night mode 5 fps
        {0x3a03, 0xe4},// 60Hz max exposure // banding filters are calculated automatically in camera driver
    //    {0x3a08, 0x01},
    //    {0x3a09, 0x27},
    //    {0x3a0a, 0x00},
    //    {0x3a0b, 0xf6},
    //    {0x3a0e, 0x03},
    //    {0x3a0d, 0x04},
        {0x3a14, 0x02},// 50Hz max exposure ,night mode 5 fps
        {0x3a15, 0xe4},// 50Hz max exposure
        {0x3b07, 0x0a},//FREX strobe mode1
        {0x4001, 0x02},//BLC start from line 2
        {0x4004, 0x02},//BLC line number
        {0x3000, 0x00},//enable blocks
        {0x3002, 0x1c},//reset JFIFO, SFIFO, JPG
        {0x3004, 0xff},//enable clocks
        {0x3006, 0xc3},// disable clock of JPEG2x, JPEG
        {0x300e, 0x58},//MIPI power down, DVP enable
        {0x302e, 0x00},
        {0x4300, 0x03},//0x00->RGB格式选择,raw RGRG..,0x65:RGB565 :RAW style
        {0x501f, 0x02},//ISP RGB     0x02
     //   {0x503d, 0x82},
     //   {0x4741, 0x0},


        {0x3b00, 0x83},//STROBE CTRL: strobe request ON, Strobe mode: LED3
        {0x3b00, 0x00},
        {0x3016, 0x02},// Strobe output enable
        /*{0x301c, 0x02},//
        {0x3019, 0x02},// Strobe output on
        {0x3019, 0x00},// Strobe output off
        */
        {0x4713, 0x03},//JPEG mode 3
        {0x4407, 0x04},// Quantization scale
        {0x440e, 0x00},
        {0x460b, 0x37},
        {0x460c, 0x20},
        {0x4837, 0x16},// MIPI global timing
        {0x3824, 0x04},// PCLK manual divider
        {0x5000, 0xa7},// Lenc on, raw gamma on, BPC on, WPC on, CIP on // AEC target    自动曝光控制
        {0x5001, 0x83},// SDE on, CMX on, AWB on
        {0x501D, 0x40},// enable manual offset of contrast// CIP  锐化和降噪
        {0x5180, 0xff},// AWB B block
        {0x5181, 0xf2},// AWB control
        {0x5182, 0x00},// [7:4] max local counter, [3:0] max fast counter
        {0x5183, 0x14},// AWB advanced
        {0x5184, 0x25},
        {0x5185, 0x24},
        {0x5186, 0x10},
        {0x5187, 0x12},
        {0x5188, 0x10},
        {0x5189, 0x74},
        {0x518a, 0x5e},
        {0x518b, 0xac},
        {0x518c, 0x83},
        {0x518d, 0x3b},
        {0x518e, 0x35},
        {0x518f, 0x4f},
        {0x5190, 0x42},
        {0x5191, 0xf8},// AWB top limit
        {0x5192, 0x04},// AWB bottom limit
        {0x5193, 0xF0},// red limit  0x78
        {0x5194, 0xF0},// green limit  0xF0
        {0x5195, 0xF0},// blue limit   0xF0
        {0x5196, 0x03},// AWB control
        {0x5197, 0x01},// local limit
        {0x5198, 0x04},
        {0x5199, 0x87},
        {0x519a, 0x04},
        {0x519b, 0x00},
        {0x519c, 0x07},
        {0x519d, 0x56},
        {0x519e, 0x38},// AWB control // Gamma    伽玛曲线
        {0x5381, 0x1e},// CMX1 for Y
        {0x5382, 0x5b},// CMX2 for Y
        {0x5383, 0x08},// CMX3 for Y
        {0x5384, 0x0a},// CMX4 for U
        {0x5385, 0x7e},// CMX5 for U
        {0x5386, 0x88},// CMX6 for U
        {0x5387, 0x7c},// CMX7 for V
        {0x5388, 0x6c},// CMX8 for V
        {0x5389, 0x10},// CMX9 for V
        {0x538a, 0x01},// sign[9]
        {0x538b, 0x98},// sign[8:1] // UV adjust   UV色彩饱和度调整
        {0x5300, 0x08},// CIP sharpen MT threshold 1
        {0x5301, 0x30},// CIP sharpen MT threshold 2
        {0x5302, 0x10},// CIP sharpen MT offset 1
        {0x5303, 0x00},// CIP sharpen MT offset 2
        {0x5304, 0x08},// CIP DNS threshold 1
        {0x5305, 0x30},// CIP DNS threshold 2
        {0x5306, 0x08},// CIP DNS offset 1
        {0x5307, 0x16},// CIP DNS offset 2
        {0x5309, 0x08},// CIP sharpen TH threshold 1
        {0x530a, 0x30},// CIP sharpen TH threshold 2
        {0x530b, 0x04},// CIP sharpen TH offset 1
        {0x530c, 0x06},// CIP sharpen TH offset 2
        {0x5480, 0x01},// Gamma bias plus on, bit[0]
        {0x5481, 0x08},
        {0x5482, 0x14},
        {0x5483, 0x28},
        {0x5484, 0x51},
        {0x5485, 0x65},
        {0x5486, 0x71},
        {0x5487, 0x7d},
        {0x5488, 0x87},
        {0x5489, 0x91},
        {0x548a, 0x9a},
        {0x548b, 0xaa},
        {0x548c, 0xb8},
        {0x548d, 0xcd},
        {0x548e, 0xdd},
        {0x548f, 0xea},
        {0x5490, 0x1d},// color matrix   色彩矩阵
        {0x5580, 0x06},// saturation on, bit[1]
        {0x5583, 0x40},
        {0x5584, 0x10},
        {0x5589, 0x10},
        {0x558a, 0x00},
        {0x558b, 0xf8},
        {0x5800, 0x23},
        {0x5801, 0x15},
        {0x5802, 0x10},
        {0x5803, 0x10},
        {0x5804, 0x15},
        {0x5805, 0x23},
        {0x5806, 0x0c},
        {0x5807, 0x08},
        {0x5808, 0x05},
        {0x5809, 0x05},
        {0x580a, 0x08},
        {0x580b, 0x0c},
        {0x580c, 0x07},
        {0x580d, 0x03},
        {0x580e, 0x00},
        {0x580f, 0x00},
        {0x5810, 0x03},
        {0x5811, 0x07},
        {0x5812, 0x07},
        {0x5813, 0x03},
        {0x5814, 0x00},
        {0x5815, 0x00},
        {0x5816, 0x03},
        {0x5817, 0x07},
        {0x5818, 0x0b},
        {0x5819, 0x08},
        {0x581a, 0x05},
        {0x581b, 0x05},
        {0x581c, 0x07},
        {0x581d, 0x0b},
        {0x581e, 0x2a},
        {0x581f, 0x16},
        {0x5820, 0x11},
        {0x5821, 0x11},
        {0x5822, 0x15},
        {0x5823, 0x29},
        {0x5824, 0xbf},
        {0x5825, 0xaf},
        {0x5826, 0x9f},
        {0x5827, 0xaf},
        {0x5828, 0xdf},
        {0x5829, 0x6f},
        {0x582a, 0x8e},
        {0x582b, 0xab},
        {0x582c, 0x9e},
        {0x582d, 0x7f},
        {0x582e, 0x4f},
        {0x582f, 0x89},
        {0x5830, 0x86},
        {0x5831, 0x98},
        {0x5832, 0x6f},
        {0x5833, 0x4f},
        {0x5834, 0x6e},
        {0x5835, 0x7b},
        {0x5836, 0x7e},
        {0x5837, 0x6f},
        {0x5838, 0xde},
        {0x5839, 0xbf},
        {0x583a, 0x9f},
        {0x583b, 0xbf},
        {0x583c, 0xec},
        {0x583d, 0xce},// lenc BR offset // AWB   自动白平衡     0xdf
        {0x5025, 0x00},
        {0x3a0f, 0x30},// stable range in high
        {0x3a10, 0x28},// stable range in low
        {0x3a1b, 0x30},// stable range out high
        {0x3a1e, 0x26},// stable range out low
        {0x3a11, 0x60},// fast zone high
        {0x3a1f, 0x01},// fast zone low// Lens correction for ?   镜头补偿
        {0x3008, 0x02},
        //CIP denoise
        {0x5300, 0x08}, // CIP sharpen MT threshold 1
        {0x5301, 0x30}, // CIP sharpen MT threshold 2
        {0x5302, 0x10}, // CIP sharpen MT offset 1
        {0x5303, 0x00}, // CIP sharpen MT offset 2
        {0x5304, 0x08}, // CIP DNS threshold 1
        {0x5305, 0x30}, // CIP DNS threshold 2
        {0x5306, 0x08}, // CIP DNS offset 1
        {0x5307, 0x16}, // CIP DNS offset 2
        {0x5309, 0x08}, // CIP sharpen TH threshold 1
        {0x530a, 0x30}, // CIP sharpen TH threshold 2
        {0x530b, 0x04}, // CIP sharpen TH offset 1
        {0x530c, 0x06}  // CIP sharpen TH offset 2
};

#define APB_BASE_ADDR   0x43C00000
enum _REG_OFFSET {
    REG_WIDTH_OFFSET    = 0x0,
    REG_HEIGHT_OFFSET   = 0x4,
    REG_TMR_OFFSET      = 0x8,
    REG_CTRL_OFFSET     = 0xC,
    REG_DEST_OFFSET     = 0x10,
    REG_SETUP_OFFSET    = 0x14,
    REG_FPS_OFFSET      = 0x18,
    REG_VER_OFFSET      = 0x1C,
};

struct reg_addr {
    volatile int *reg_width;
    volatile int *reg_height;
    volatile int *reg_tmr;
    volatile int *reg_ctrl;
    volatile int *reg_dest;
    volatile int *reg_setup;
    volatile int *reg_fps;
//    volatile int *reg_ver;
};

struct ov5640 {
    struct i2c_client *i2c_client;
    struct reg_addr addr;
    int rst_gpio;
    int irq_gpio;
};

static struct ov5640 ov5640_priv;
//static volatile int *pic_addr;

static int ov5640_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		dev_err(&client->dev, "%s: i2c write error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int ov5640_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	/* We have 16-bit i2c addresses - care for endianness */
	unsigned char data[2] = { reg >> 8, reg & 0xff };

	ret = i2c_master_send(client, data, 2);
	if (ret < 2) {
		dev_err(&client->dev, "%s: i2c read error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(client, val, 1);
	if (ret < 1) {
		dev_err(&client->dev, "%s: i2c read error, reg: %x\n", __func__, reg);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int ov5640_write_array(struct i2c_client *client, struct regval_list *vals)
{
	while (vals->reg_num != 0xffff || vals->value != 0xff) {
		int ret = ov5640_write_reg(client, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	dev_info(&client->dev, "Register list loaded\n");
	return 0;
}

static inline void ov5640_reset(struct ov5640 *priv)
{
    /* camera reset */
    gpio_set_value(priv->rst_gpio, 1);
    msleep(1);
    gpio_set_value(priv->rst_gpio, 0);
    msleep(1);
    gpio_set_value(priv->rst_gpio, 1);
    msleep(5);
}

static inline void ov5640_init(struct ov5640 *priv, struct i2c_client *client)
{
    struct device *dev = &client->dev;
    int retval;

    priv->i2c_client = client;

    priv->addr.reg_width    = devm_ioremap(dev, APB_BASE_ADDR + REG_WIDTH_OFFSET, 4);
    priv->addr.reg_height   = devm_ioremap(dev, APB_BASE_ADDR + REG_HEIGHT_OFFSET, 4);
    priv->addr.reg_tmr      = devm_ioremap(dev, APB_BASE_ADDR + REG_TMR_OFFSET, 4);
    priv->addr.reg_ctrl     = devm_ioremap(dev, APB_BASE_ADDR + REG_CTRL_OFFSET, 4);
    priv->addr.reg_dest     = devm_ioremap(dev, APB_BASE_ADDR + REG_DEST_OFFSET, 4);
    priv->addr.reg_setup    = devm_ioremap(dev, APB_BASE_ADDR + REG_SETUP_OFFSET, 4);
    priv->addr.reg_fps      = devm_ioremap(dev, APB_BASE_ADDR + REG_FPS_OFFSET, 4);
//    priv->addr.reg_ver      = devm_ioremap(dev, APB_BASE_ADDR + REG_VER_OFFSET, 4);

//    dev_info(dev, "FPGA-Ver: 0x%8x\n", ioread32(priv->addr.reg_ver));

//    pic_addr = devm_ioremap(dev, 0x18000000, 0x4000000);
/*
    for (i = 0; i < 1024; i++) {
        iowrite32(0, pic_addr + i);
        dev_info(dev, "pic_addr + %d:0x%x, ", i, ioread32(pic_addr+i));
    }
    printk("\n");
*/
    /* ov5640 default init */
    retval = ov5640_write_array(client, ov5640_default_regs_init);
    if (retval < 0) {
        dev_err(dev, "ov5640_write_array error!\n");
    }

    /* setting auto exposure */
    ov5640_write_reg(client, 0x3503, 0x00);

    /* set para iso */
    ov5640_write_reg(client, 0x3A19, 0xF8);

    /* set para antibanding */
    ov5640_write_reg(client, 0x3C01, 0x80);
    ov5640_write_reg(client, 0x3C00, 0x04);

    /* set para exposure level */
    ov5640_write_reg(client, 0x3A0F, 0x28);
    ov5640_write_reg(client, 0x3A10, 0x20);
    ov5640_write_reg(client, 0x3A11, 0x51);
    ov5640_write_reg(client, 0x3A1B, 0x28);
    ov5640_write_reg(client, 0x3A1E, 0x20);
    ov5640_write_reg(client, 0x3A1F, 0x10);

    /* set para white balance */
    ov5640_write_reg(client, 0x3406, 0x00);

    /* do something for AutoFocus */

    *priv->addr.reg_ctrl    = 0x1;
    *priv->addr.reg_ctrl    = 0x0;
    *priv->addr.reg_width   = 0x01400280;
    *priv->addr.reg_height  = 0x007801E0;
    *priv->addr.reg_tmr     = 0x000003E8;
    *priv->addr.reg_dest    = 0x18000000;
    *priv->addr.reg_setup   = 0x0060F380;
    *priv->addr.reg_fps     = 0x18;
    *priv->addr.reg_ctrl    = 0x2;
/*
    msleep(100);
    printk("======================================================\n");
    for (i = 0; i < 1024; i++) {
        dev_info(dev, "pic_addr + %d:0x%x, ", i, ioread32(pic_addr+i));
    }
    printk("\n");
*/
}

#if 0
static irqreturn_t ov5640_isr(int irq, void *dev_id)
{
    struct ov5640 *priv = (struct ov5640 *)dev_id;

    dev_info(&priv->i2c_client->dev, "IRQ_HANDLED\n");

    return IRQ_HANDLED;
}
#endif

static int ov5640_probe(struct i2c_client *client, const struct i2c_device_id *did)
{
    struct device *dev = &client->dev;
    struct ov5640 *priv = &ov5640_priv;
    int retval;
    u8 chip_id_high, chip_id_low;
    u16 chip_id;

    dev_info(dev, "%s: called", __FUNCTION__);

    /* request reset pin */
    priv->rst_gpio = of_get_named_gpio(dev->of_node, "ov,rst-gpios", 0);
    if (!gpio_is_valid(priv->rst_gpio)) {
        goto ERR_INVAL;
    }
    retval = devm_gpio_request_one(dev, priv->rst_gpio, GPIOF_OUT_INIT_HIGH, "ov5640_reset");
    if (retval < 0) {
        return retval;
    }
    dev_info(dev, "%s: called, rst_gpio:%d", __FUNCTION__, priv->rst_gpio);

#if 0
    /* request irq pin */
    priv->irq_gpio = of_get_named_gpio(dev->of_node, "ov,irq-gpios", 0);
    if (!gpio_is_valid(priv->irq_gpio)) {
        goto ERR_INVAL;
    }
    retval = devm_gpio_request_one(dev, priv->irq_gpio, GPIOF_IN, "ov5640_irq");
    if (retval < 0) {
        return retval;
    }

    /* request irq handle */
    retval = devm_request_threaded_irq(dev, client->irq, NULL, ov5640_isr, IRQF_TRIGGER_RISING, client->name, priv);
    if (retval) {
        dev_err(dev, "Failed to request IRQ: %d\n", retval);
        return retval;
    }
#endif
    ov5640_reset(priv);
#if 0
    /* Read sensor Model ID */
    if ((retval = ov5640_read_reg(client, REG_CHIP_ID_HIGH, &chip_id_high)) < 0) {
        goto ERR_NODEV;
    }
    chip_id = chip_id_high << 8;
    if ((retval = ov5640_read_reg(client, REG_CHIP_ID_LOW, &chip_id_low)) < 0) {
        goto ERR_NODEV;
    }
    chip_id |= chip_id_low;

    dev_info(dev, "Chip ID 0x%04x detected\n", chip_id);

    if (chip_id != 0x5640) {
        goto ERR_NODEV;
    }

    ov5640_init(priv, client);

    dev_info(dev, "camera ov5640 is found\n");
#endif
    return 0;

#if 0
ERR_NODEV:
    dev_err(dev, "camera ov5640 is not found\n");
    return -ENODEV;
#endif
ERR_INVAL:
    dev_err(dev, "no sensor reset pin available\n");
    return -EINVAL;
}

static int ov5640_remove(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ov_match_table[] = {
    {.compatible = "omnivision,ov5640"},
    { }
};
#endif

static const struct i2c_device_id ov5640_id[] = {
	{ "ov5640", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5640_id);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		.name = "ov5640",
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = ov_match_table,
#endif
	},
	.probe		= ov5640_probe,
	.remove		= ov5640_remove,
	.id_table	= ov5640_id,
};

module_i2c_driver(ov5640_i2c_driver);

MODULE_DESCRIPTION("Omnivision OV5640 Camera driver");
MODULE_AUTHOR("Clive Liu <ftdstudio1990@gmail.com>");
MODULE_LICENSE("GPL v2");
