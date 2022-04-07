/*****************************************************************************
 *
 * Filename:
 * ---------
 *     bf2253Lmipiraw_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
    *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
//#include <linux/xlog.h>
   
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "bf2253Lmipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "bf2253L_camera_sensor"
#define LOG_1 printk( "bf2253L,MIPI 1LANE\n")
#define LOG_2 printk( "preview 1600*1200@15fps; video 1600*1200@15fps; capture 1600x1200@15fps\n")

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

/****************************   Modify end    *******************************************/

static DEFINE_SPINLOCK( imgsensor_drv_lock );

static imgsensor_info_struct imgsensor_info = {
	.sensor_id = BF2253L_SENSOR_ID,	//0x2253	//record sensor id defined in Kd_imgsensor.h

	.checksum_value = 0xb7c714d7,		//checksum value for Camera Auto Test

	.pre = {
		.pclk = 66000000, 				//record different mode's pclk
		.linelength = 1780,				//record different mode's linelength
		.framelength = 1236,			//record different mode's framelength
		.startx = 0,                    //record different mode's startx of grabwindow
		.starty = 0,                    //record different mode's starty of grabwindow
		.grabwindow_width = 1600,        //record different mode's width of grabwindow
		.grabwindow_height = 1200,        //record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
 		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 66000000,
		.linelength = 1780,
		.framelength = 1236,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 66000000,
		.linelength = 1780,
		.framelength = 1236,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 66000000,
		.linelength = 1780,
		.framelength = 1236,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 66000000,
		.linelength = 1780,
		.framelength = 1236,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},
	.slim_video = {
		.pclk = 66000000,
		.linelength = 1780,
		.framelength = 1236,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1600,
		.grabwindow_height = 1200,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
	},

	.margin = 0,						//sensor framelength & shutter margin
	.min_shutter = 1,					//min shutter
	.max_frame_length = 0xffff,			//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,			//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,	//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,		//isp gain delay frame for AE cycle
	.ihdr_support = 0,					//1, support; 0,not support
	.ihdr_le_firstline = 0,				//1, le first ; 0, se first
	.sensor_mode_num = 5,				//support sensor mode num

	.cap_delay_frame = 2,				//enter capture delay frame num
	.pre_delay_frame = 3,				//enter preview delay frame num
	.video_delay_frame = 3,				//enter video delay frame num
	.hs_video_delay_frame = 3,			//enter high speed video  delay frame num
	.slim_video_delay_frame = 3,		//enter slim video delay frame num

	.isp_driving_current		= ISP_DRIVING_6MA,				//mclk driving current
	.sensor_interface_type		= SENSOR_INTERFACE_TYPE_MIPI,	//sensor_interface_type
	.mipi_sensor_type			= MIPI_OPHY_NCSI2,				//0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode		= MIPI_SETTLEDELAY_AUTO,		//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat	= SENSOR_OUTPUT_FORMAT_RAW_B,	//sensor output first pixel color
	.mclk						= 24,							//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num				= SENSOR_MIPI_1_LANE,			//mipi lane num
	.i2c_addr_table				= {0xdc,0xff},			//record sensor support all write id addr, only supprt 4, must end with 0xff
};

// Gain Index
#define MaxGainIndex (65)  

static imgsensor_struct imgsensor = {
	.mirror				 = IMAGE_NORMAL,//IMAGE_HV_MIRROR,  			//mirrorflip information
	.sensor_mode		 = IMGSENSOR_MODE_INIT,				//IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter			 = 0x3D0,							//current shutter
	.gain				 = 0x0,								//current gain
	.dummy_pixel		 = 0,								//current dummypixel
	.dummy_line			 = 0,								//current dummyline
	.current_fps		 = 150,								//full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en		 = KAL_FALSE,						//auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern		 = KAL_FALSE,						//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,	//current scenario id
	.ihdr_en			 = 0,								//sensor need support LE, SE with HDR feature
	.i2c_write_id		 = 0xdc,							//record current sensor's i2c write id
};

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] = {
	{ 1600, 1200,	 0,    0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200, 	 0,    0, 1600,  1200}, // Preview 
	{ 1600, 1200,	 0,    0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200, 	 0,    0, 1600,  1200}, // capture 
	{ 1600, 1200,	 0,    0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200, 	 0,    0, 1600,  1200}, // video 
	{ 1600, 1200,	 0,    0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200, 	 0,    0, 1600,  1200}, // high speed video 
	{ 1600, 1200,	 0,    0, 1600, 1200, 1600,  1200, 0000, 0000, 1600,  1200, 	 0,    0, 1600,  1200}, // slim video 
};



kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[1] = {(char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 1, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
#if 1
		char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};
		iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
#else
		iWriteReg((u16)addr, (u32)para, 2, imgsensor.i2c_write_id);
#endif
}

static void set_dummy(void)
{
	kal_uint16	dummy_pixel = 0;
	kal_uint16	dummy_line = 0;
	kal_uint32 LineLength;

	printk( "set_dummy()\n" );
	printk( "dummyline=%d, dummypixels=%d\n", imgsensor.dummy_line, imgsensor.dummy_pixel );

	dummy_pixel = imgsensor.dummy_pixel;
	dummy_line = imgsensor.dummy_line;
	LineLength = dummy_pixel + 1780;

	// Set dummy pixel
	write_cmos_sensor( 0x26, (LineLength  >> 8) & 0x0f );
	write_cmos_sensor( 0x25, (LineLength      ) & 0xff );
	// Set dummy line
	write_cmos_sensor( 0x08, (dummy_line >> 8) & 0xff );
	write_cmos_sensor( 0x07, (dummy_line     ) & 0xff );
}

static kal_uint32 return_sensor_id(void)
{
	return( (read_cmos_sensor( 0xfc ) << 8) | read_cmos_sensor( 0xfd ) );
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	//kal_int16	dummy_line;
	kal_uint32	frame_length = imgsensor.frame_length;
	//unsigned long	flags;

	printk( "framerate=%d, min framelength should enable=%d\n", framerate, min_framelength_en );

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;

	spin_lock( &imgsensor_drv_lock );
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line   = imgsensor.frame_length - imgsensor.min_frame_length;

	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if( dummy_line < 0 )
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;

	if( imgsensor.frame_length > imgsensor_info.max_frame_length )
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line   = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if( min_framelength_en )
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock( &imgsensor_drv_lock );

	set_dummy();
}

/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long	flags;

	spin_lock_irqsave( &imgsensor_drv_lock, flags );
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore( &imgsensor_drv_lock, flags );

	spin_lock( &imgsensor_drv_lock );
	if( shutter > imgsensor.min_frame_length - imgsensor_info.margin )
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;

	if( imgsensor.frame_length > imgsensor_info.max_frame_length )
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock( &imgsensor_drv_lock );

	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

	// Update Shutter
	write_cmos_sensor( 0x6b, (shutter >> 8) & 0xff );
	write_cmos_sensor( 0x6c, (shutter     ) & 0xff );
	printk( "Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length );
}

static kal_uint16 sensorGainMapping[MaxGainIndex][2] ={
 {1024  ,15},
 {1088  ,16},
 {1150  ,17},
 {1214  ,18},
 {1277  ,19},
 {1342  ,20},
 {1403  ,21},
 {1468  ,22},
 {1527  ,23},
 {1594  ,24},
 {1652  ,25},
 {1716  ,26},
 {1775  ,27},
 {1839  ,28},
 {1897  ,29},
 {1960  ,30},
 {2023  ,31},
 {2145  ,32},
 {2273  ,33},
 {2395  ,34},
 {2519  ,35},
 {2641  ,36},
 {2765  ,37},
 {2885  ,38},
 {3006  ,39},
 {3131  ,40},
 {3249  ,41},
 {3373  ,42},
 {3493  ,43},
 {3610  ,44},
 {3730  ,45},
 {3846  ,46},
 {3962  ,47},
 {4206  ,48},
 {4441  ,49},
 {4678  ,50},
 {4906  ,51},
 {5146  ,52},
 {5371  ,53},
 {5601  ,54},
 {5831  ,55},
 {6046  ,56},
 {6259  ,57},
 {6493  ,58},
 {6713  ,59},
 {6928  ,60},
 {7149  ,61},
 {7359  ,62},
 {7564  ,63},
 {7789  ,64},
 {8000  ,65},
 {8216  ,66},
 {8439  ,67},
 {8648  ,68},
 {8859  ,69},
 {9065  ,70},
 {9280  ,71},
 {9482  ,72},
 {9665  ,73},
 {9891  ,74},
 {10073 ,75},
 {10273 ,76},
 {10474 ,77},
 {10663 ,78},
 {10856 ,79},
};


static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 iI;
    kal_uint16 ret = 0;

    for (iI = 0; iI < (MaxGainIndex-1); iI++) {
    	if((gain >= sensorGainMapping[iI][0])&&(gain < sensorGainMapping[iI+1][0]))
    	{
	    	if((gain-sensorGainMapping[iI][0]) <= (sensorGainMapping[iI+1][0]-gain))
	    		ret = sensorGainMapping[iI][1];
	    	else
	    		ret = sensorGainMapping[iI+1][1];
       break;
	}
    }
    return ret;
}

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain( base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16	reg_gain = 0;
	gain = gain << 4;
	if (gain < 1024)
	    gain = 1024;
	else if (gain > 10856)
	    gain = 10856;
	reg_gain = gain2reg( gain );
	spin_lock( &imgsensor_drv_lock );
	imgsensor.gain = reg_gain;
	spin_unlock( &imgsensor_drv_lock );
	LOG_INF( "gain=%d, reg_gain=0x%x\n ", gain, reg_gain );
	
	write_cmos_sensor( 0x6a, reg_gain & 0xff );


	return gain;
}

static void ihdr_write_shutter_gain( kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	printk( "le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain );
}
#if 1
static void set_mirror_flip(kal_uint8 image_mirror)
{
	printk( "image_mirror=%d\n", image_mirror );

	switch( image_mirror ) {
		case IMAGE_NORMAL:
			write_cmos_sensor( 0x00, 0x20 );
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor( 0x00, 0x28 );
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor( 0x00, 0x24 );
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor( 0x00, 0x2c );
			break;
		default:
			printk( "Error image_mirror setting" );
	}
}
#endif
/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode( kal_bool enable)
{
	/*No Need to implement this function*/
}

static void sensor_init(void)
{
	printk( ">> sensor_init()\n" );
	
	//;==================INI==================
	//Product Ver:VAI01
	//;;Output Detail:
	//;;XCLK:24 MHz
	//;;PCLK:66 MHz
	//;;MipiCLK:660 MHz
	//;;FrameW:1780
	//;;FrameH:1236
	
	//write_cmos_sensor( 0xf2, 0x01 );//0xf2[0]:1,reset
	write_cmos_sensor(0xe1, 0x06);
	write_cmos_sensor(0xe2, 0x06);
	write_cmos_sensor(0xe3, 0x0e);
	write_cmos_sensor(0xe4, 0x40);
	write_cmos_sensor(0xe5, 0x67);
	write_cmos_sensor(0xe6, 0x02);
	write_cmos_sensor(0xe8, 0x84);
	write_cmos_sensor(0x01, 0x14);
	write_cmos_sensor(0x03, 0x98);
	write_cmos_sensor(0x27, 0x21);
	write_cmos_sensor(0x29, 0x20);
	write_cmos_sensor(0x59, 0x10);
	write_cmos_sensor(0x5a, 0x10);
	write_cmos_sensor(0x5c, 0x11);
	write_cmos_sensor(0x5d, 0x73);
	write_cmos_sensor(0x6a, 0x2f);//global gain
	write_cmos_sensor(0x6b, 0x0e);
	write_cmos_sensor(0x6c, 0x7e);//{0x6b,0x6c}int time
	write_cmos_sensor(0x6f, 0x10);
	write_cmos_sensor(0x70, 0x08);
	write_cmos_sensor(0x71, 0x05);
	write_cmos_sensor(0x72, 0x10);
	write_cmos_sensor(0x73, 0x08);
	write_cmos_sensor(0x74, 0x05);
	write_cmos_sensor(0x75, 0x06);
	write_cmos_sensor(0x76, 0x20);
	write_cmos_sensor(0x77, 0x03);
	write_cmos_sensor(0x78, 0x0e);
	write_cmos_sensor(0x79, 0x08); 
	
	
	printk( "<< sensor_init()\n" );
}

static void preview_setting(void)
{
	printk( ">> preview_setting()\n" );
	write_cmos_sensor( 0xe0, 0x00 );//power up
}/*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	printk( ">> capture_setting()\n" );
	write_cmos_sensor( 0xe0, 0x00 );//power up
}

static void normal_video_setting(kal_uint16 currefps)
{
	printk( ">> normal_video_setting(): currefps=%d\n", currefps );
	write_cmos_sensor( 0xe0, 0x00 );//power up
	mDELAY( 30 );

	printk( "<< normal_video_setting()\n" );
}

static void hs_video_setting(kal_uint16 currefps)
{
	printk( ">> hs_video_setting()\n" );
	write_cmos_sensor( 0xe0, 0x00 );//power up
	mDELAY( 30 );

	printk( "<< hs_video_setting()\n" );
}

static void slim_video_setting(void)
{
	printk( ">> slim_video_setting()\n" );
	write_cmos_sensor( 0xe0, 0x00 );//power up
	mDELAY( 30 );

	printk( "<< slim_video_setting()\n" );
}


static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	printk( ">> set_test_pattern_mode(): enable=%d\n", enable );
	printk( "enable: %d\n", enable );

	if( enable ) {	
		write_cmos_sensor( 0x7e, 0x50 );
		write_cmos_sensor( 0x82, 0xff );
		write_cmos_sensor( 0x83, 0xff );
		write_cmos_sensor( 0x84, 0xff );
	}
	else {		
		write_cmos_sensor( 0x7e, 0x10 );
	}

	spin_lock( &imgsensor_drv_lock );
	imgsensor.test_pattern = enable;
	spin_unlock( &imgsensor_drv_lock );

	printk( "<< set_test_pattern_mode()\n" );
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8	i = 0;
	kal_uint8	retry = 2;

	printk( ">> get_imgsensor_id()\n" );

	while( imgsensor_info.i2c_addr_table[i] != 0xff ) {
		spin_lock( &imgsensor_drv_lock );
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock( &imgsensor_drv_lock );

		do {
			*sensor_id = return_sensor_id();
			if( *sensor_id == imgsensor_info.sensor_id ) {
				printk( "[bf2253L] i2c_write_id=0x%x, sensor_id=0x%x\n", imgsensor.i2c_write_id, *sensor_id );
				printk( "<< get_imgsensor_id(): OK\n" );
				return ERROR_NONE;
			}
			printk( "[bf2253L] Read sensor id fail, write_id=0x%x, sensor_id=0x%x\n", imgsensor.i2c_write_id, *sensor_id );
			retry--;
		} while( retry > 0 );

		i++;
		retry = 2;
	}
	if( *sensor_id != imgsensor_info.sensor_id ) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
		*sensor_id = 0xFFFFFFFF;				/// Invalid sensor ID
		printk( "<< get_imgsensor_id(): FAIL\n" );
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	printk( "<< get_imgsensor_id()\n" );
	return ERROR_NONE;
}
 
/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8	i = 0;
	kal_uint8	retry = 2;
	kal_uint32	sensor_id = 0;

	printk( ">> open()\n" );

	LOG_1;
	LOG_2;

	while( imgsensor_info.i2c_addr_table[i] != 0xff ) {
		spin_lock( &imgsensor_drv_lock );
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock( &imgsensor_drv_lock );

		do {
			sensor_id = return_sensor_id();
			if( sensor_id == imgsensor_info.sensor_id ) {
				printk( "[bf2253L] i2c_write_id=0x%x, sensor_id=0x%x\n", imgsensor.i2c_write_id, sensor_id );
				break;
			}
			printk( "[bf2253L] Read sensor id fail, write_id=0x%x, sensor_id=0x%x\n", imgsensor.i2c_write_id, sensor_id );
			retry--;
		} while( retry > 0 );

		i++;
		if( sensor_id == imgsensor_info.sensor_id )
			break;

		retry = 2;
	}
	if( imgsensor_info.sensor_id != sensor_id ) {
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	/* initail sequence write in  */
	sensor_init();

	spin_lock( &imgsensor_drv_lock );
	imgsensor.autoflicker_en	= KAL_FALSE;
	imgsensor.sensor_mode		= IMGSENSOR_MODE_INIT;
	imgsensor.pclk				= imgsensor_info.pre.pclk;
	imgsensor.frame_length		= imgsensor_info.pre.framelength;
	imgsensor.line_length		= imgsensor_info.pre.linelength;
	imgsensor.min_frame_length	= imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel		= 0;
	imgsensor.dummy_line		= 0;
	imgsensor.ihdr_en			= 0;
	imgsensor.test_pattern		= KAL_FALSE;
	imgsensor.current_fps		= imgsensor_info.pre.max_framerate;
	spin_unlock( &imgsensor_drv_lock );

	printk( "<< open()\n" );
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	printk( ">> close()\n" );

	/*No Need to implement this function*/
	
	printk( "<< close()\n" );
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> preview()\n" );

	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode		= IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk				= imgsensor_info.pre.pclk;
	//imgsensor.video_mode		= KAL_FALSE;
	imgsensor.line_length		= imgsensor_info.pre.linelength;
	imgsensor.frame_length		= imgsensor_info.pre.framelength;
	imgsensor.min_frame_length	= imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en	= KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );

	preview_setting();
	set_mirror_flip( imgsensor.mirror );

	printk( "<< preview()\n" );
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> capture(()\n" );

	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if( imgsensor.current_fps == imgsensor_info.cap1.max_framerate ) { //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk				= imgsensor_info.cap1.pclk;
		imgsensor.line_length		= imgsensor_info.cap1.linelength;
		imgsensor.frame_length		= imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length	= imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en	= KAL_FALSE;
	} else {
		if( imgsensor.current_fps != imgsensor_info.cap.max_framerate )
			printk( "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n", imgsensor.current_fps, imgsensor_info.cap.max_framerate/10 );

		imgsensor.pclk				= imgsensor_info.cap.pclk;
		imgsensor.line_length		= imgsensor_info.cap.linelength;
		imgsensor.frame_length		= imgsensor_info.cap.framelength;
		imgsensor.min_frame_length	= imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en	= KAL_FALSE;
	}
	spin_unlock( &imgsensor_drv_lock );

	capture_setting( imgsensor.current_fps );
	set_mirror_flip( imgsensor.mirror );

	printk( "<< capture(()\n" );
	return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> normal_video()\n" );

	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode		= IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk				= imgsensor_info.normal_video.pclk;
	imgsensor.line_length		= imgsensor_info.normal_video.linelength;
	imgsensor.frame_length		= imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length	= imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps		= 300;
	imgsensor.autoflicker_en	= KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );

	normal_video_setting( imgsensor.current_fps );
	set_mirror_flip( imgsensor.mirror );

	printk( "<< normal_video()\n" );
	return ERROR_NONE;
}

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> hs_video()\n" );

	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode		= IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk				= imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode		= KAL_TRUE;
	imgsensor.line_length		= imgsensor_info.hs_video.linelength;
	imgsensor.frame_length		= imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length	= imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line		= 0;
	imgsensor.dummy_pixel		= 0;
	imgsensor.autoflicker_en	= KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );

	hs_video_setting(imgsensor.current_fps);
	set_mirror_flip( imgsensor.mirror );

	printk( "<< hs_video()\n" );
	return ERROR_NONE;
}

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> slim_video()\n" );

	spin_lock( &imgsensor_drv_lock );
	imgsensor.sensor_mode		= IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk				= imgsensor_info.slim_video.pclk;
	imgsensor.line_length		= imgsensor_info.slim_video.linelength;
	imgsensor.frame_length		= imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length	= imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line		= 0;
	imgsensor.dummy_pixel		= 0;
	imgsensor.autoflicker_en	= KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );

	slim_video_setting();
	set_mirror_flip( imgsensor.mirror );

	printk( "<< slim_video()\n" );
	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	printk( ">> get_resolution()\n" );

	sensor_resolution->SensorFullWidth				= imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight				= imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth			= imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight			= imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth				= imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight			= imgsensor_info.normal_video.grabwindow_height;

	sensor_resolution->SensorHighSpeedVideoWidth	= imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	= imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth			= imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight		= imgsensor_info.slim_video.grabwindow_height;

	printk( "<< get_resolution()\n" );
	return ERROR_NONE;
}

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_INFO_STRUCT *sensor_info, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> get_info()\n" );
	printk( "scenario_id=%d\n", scenario_id );

	//sensor_info->SensorVideoFrameRate				= imgsensor_info.normal_video.max_framerate/10;	/* not use */
	//sensor_info->SensorStillCaptureFrameRate		= imgsensor_info.cap.max_framerate/10;			/* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate	= imgsensor_info.v.max_framerate;				/* not use */

	sensor_info->SensorClockPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity	= SENSOR_CLOCK_POLARITY_LOW;	/* not use */
	sensor_info->SensorHsyncPolarity		= SENSOR_CLOCK_POLARITY_LOW;	// inverse with datasheet
	sensor_info->SensorVsyncPolarity		= SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines	= 4;		/* not use */
	sensor_info->SensorResetActiveHigh		= FALSE;	/* not use */
	sensor_info->SensorResetDelayCount		= 5;		/* not use */

	sensor_info->SensroInterfaceType		= imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType				= imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode			= imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat		= imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame			= imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame			= imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame			= imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame	= imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame		= imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch	= 0;		/* not use */
	sensor_info->SensorDrivingCurrent		= imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame			= imgsensor_info.ae_shut_delay_frame;			/* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame		= imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame		= imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support				= imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine			= imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum				= imgsensor_info.sensor_mode_num;

	sensor_info->SensorMIPILaneNumber		= imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq			= imgsensor_info.mclk;
	sensor_info->SensorClockDividCount		= 3;		/* not use */
	sensor_info->SensorClockRisingCount		= 0;
	sensor_info->SensorClockFallingCount	= 2;		/* not use */
	sensor_info->SensorPixelClockCount		= 3;		/* not use */
	sensor_info->SensorDataLatchCount		= 2;		/* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount	= 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount	= 0;
	sensor_info->SensorWidthSampling		= 0;		// 0 is default 1x
	sensor_info->SensorHightSampling		= 0;		// 0 is default 1x
	sensor_info->SensorPacketECCOrder		= 1;

	switch( scenario_id )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	printk( "<< get_info()\n" );
	return ERROR_NONE;
}

static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk( ">> control()\n" );
	printk( "scenario_id=%d\n", scenario_id );

	spin_lock( &imgsensor_drv_lock );
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock( &imgsensor_drv_lock );

	switch( scenario_id )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview( image_window, sensor_config_data );
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture( image_window, sensor_config_data );
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video( image_window, sensor_config_data );
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video( image_window, sensor_config_data );
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video( image_window, sensor_config_data );
			break;
		default:
			printk( "Error ScenarioId setting" );
			preview( image_window, sensor_config_data );
			return ERROR_INVALID_SCENARIO_ID;
	}

	printk( "<< control()\n" );
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{	//This Function not used after ROME
	printk( ">> set_video_mode(): framerate=%d\n", framerate );
	printk( "framerate=%d\n", framerate );

	// SetVideoMode Function should fix framerate
	if( framerate == 0 )
		// Dynamic frame rate
		return ERROR_NONE;

	spin_lock( &imgsensor_drv_lock );
	if( (framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE) )
		imgsensor.current_fps = 296;
	else if( (framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE) )
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock( &imgsensor_drv_lock );

	set_max_framerate( imgsensor.current_fps, 1 );

	printk( "<< set_video_mode()\n" );
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	printk( ">> set_auto_flicker_mode(): enable=%d, framerate=%d\n", enable, framerate );
	printk( "enable=%d, framerate=%d \n", enable, framerate );

	spin_lock( &imgsensor_drv_lock );
	if( enable ) //enable auto flicker
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock( &imgsensor_drv_lock );

	printk( "<< set_auto_flicker_mode()\n" );
	return ERROR_NONE;
}

static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32	frame_length;

	printk( ">> set_max_framerate_by_scenario(): scenario_id=%d, framerate=%d\n", scenario_id, framerate );
	printk( "scenario_id=%d, framerate=%d\n", scenario_id, framerate );

	switch( scenario_id )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if( framerate == 0 )
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if( imgsensor.current_fps == imgsensor_info.cap1.max_framerate ) {
				frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
				spin_lock( &imgsensor_drv_lock );
				imgsensor.dummy_line		= (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
				imgsensor.frame_length		= imgsensor_info.cap1.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length	= imgsensor.frame_length;
				spin_unlock( &imgsensor_drv_lock );
			} else {
				if( imgsensor.current_fps != imgsensor_info.cap.max_framerate )
					printk( "Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n", framerate, imgsensor_info.cap.max_framerate/10 );
				frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
				spin_lock( &imgsensor_drv_lock );
				imgsensor.dummy_line		= (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
				imgsensor.frame_length		= imgsensor_info.cap.framelength + imgsensor.dummy_line;
				imgsensor.min_frame_length	= imgsensor.frame_length;
				spin_unlock( &imgsensor_drv_lock );
			}
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			break;
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock( &imgsensor_drv_lock );
			imgsensor.dummy_line		= (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length		= imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length	= imgsensor.frame_length;
			spin_unlock( &imgsensor_drv_lock );
			//set_dummy();
			printk( "error scenario_id = %d, we use preview scenario \n", scenario_id );
			break;
	}

	printk( "<< set_max_framerate_by_scenario()\n" );
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	printk( ">> get_default_framerate_by_scenario()\n" );
	printk( "scenario_id = %d\n", scenario_id );

	switch( scenario_id )
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	printk( "<< get_default_framerate_by_scenario()\n" );
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id, UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16	= (UINT16 *) feature_para;
	UINT16 *feature_data_16			= (UINT16 *) feature_para;
	UINT32 *feature_return_para_32	= (UINT32 *) feature_para;
	UINT32 *feature_data_32			= (UINT32 *) feature_para;
	unsigned long long *feature_data		= (unsigned long long *) feature_para;
	//unsigned long long *feature_return_para	= (unsigned long long *) feature_para;

	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	printk( "feature_id=%d\n", feature_id );
	switch( feature_id )
	{
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			printk( "feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps );
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			set_shutter( *feature_data );
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			night_mode( (BOOL) *feature_data );
			break;
		case SENSOR_FEATURE_SET_GAIN:
			set_gain( (UINT16) *feature_data );
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			write_cmos_sensor( sensor_reg_data->RegAddr, sensor_reg_data->RegData );
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor( sensor_reg_data->RegAddr );
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			set_video_mode( *feature_data );
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id( feature_return_para_32 );
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode( (BOOL)*feature_data_16,*( feature_data_16+1) );
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			set_max_framerate_by_scenario( (enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1) );
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			get_default_framerate_by_scenario( (enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)( uintptr_t)( *(feature_data+1)) );
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			set_test_pattern_mode( (BOOL)*feature_data );
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len = 4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
			//printk( "current fps :%d\n", *feature_data );
			spin_lock( &imgsensor_drv_lock );
			imgsensor.current_fps = *feature_data;
			spin_unlock( &imgsensor_drv_lock );
			break;
		case SENSOR_FEATURE_SET_HDR:
			//printk( "ihdr enable :%d\n", (BOOL)*feature_data );
			spin_lock( &imgsensor_drv_lock );
			imgsensor.ihdr_en = (BOOL)*feature_data;
			spin_unlock( &imgsensor_drv_lock );
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
			wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)( *(feature_data+1) );

			switch( *feature_data_32 )
			{
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[1], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT) );
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[2], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT) );
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[3], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT) );
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[4], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT) );
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy( (void *)wininfo, (void *)&imgsensor_winsize_info[0], sizeof(struct SENSOR_WINSIZE_INFO_STRUCT) );
					break;
			}
  
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
			//printk( "SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n", (UINT16)*feature_data, (UINT16)*( feature_data+1), (UINT16)*( feature_data+2) );
			ihdr_write_shutter_gain( (UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2) );
			break;
		default:
			break;
	}

	return ERROR_NONE;
}


static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 BF2253L_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
	/* To Do : Check Sensor status here */
	if( pfFunc != NULL )
		*pfFunc = &sensor_func;

	return ERROR_NONE;
}    /*    BF2253LMIPI_RAW_SensorInit    */
