#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
	
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
	

extern struct i2c_client *g_tlsc6x_client;

#define GSTR_TL_DOUBLECLICK                     0x24
#define GSTR_TL_UP                              0x22
#define GSTR_TL_DOWN                            0x23
#define GSTR_TL_LEFT                            0x20
#define GSTR_TL_RIGHT                           0x21
#define GSTR_TL_C                               0x34
#define GSTR_TL_O                               0x30
#define GSTR_TL_W                               0x31
#define GSTR_TL_M                               0x32
#define GSTR_TL_E                               0x33
#define GSTR_TL_L                               0x44
#define GSTR_TL_S                               0x46
#define GSTR_TL_V                               0x54
#define GSTR_TL_Z                               0x65

//#define  KEY_GSTR_U								KEY_U
#define  KEY_TL_GSTR_UP								KEY_UP
#define  KEY_TL_GSTR_DOWN							KEY_DOWN
#define  KEY_TL_GSTR_LEFT							KEY_LEFT 
#define  KEY_TL_GSTR_RIGHT							KEY_RIGHT
#define  KEY_TL_GSTR_C								KEY_C
#define  KEY_TL_GSTR_O								KEY_O
#define  KEY_TL_GSTR_E								KEY_E
#define  KEY_TL_GSTR_M								KEY_M 
#define  KEY_TL_GSTR_W								KEY_W
#define  KEY_TL_GSTR_L								KEY_L
#define  KEY_TL_GSTR_S								KEY_S 
#define  KEY_TL_GSTR_V								KEY_V
#define  KEY_TL_GSTR_Z								KEY_Z

#if 1
u16 gstr_data_cnt = 0;
u16 rpt_res_y = 854;//480;
u16 rpt_res_x = 480;//854;

u8 letter_w_dirs[]={
	20,5,20,5
};
u8 letter_m_dirs[]={
	5,20,5,20
};
u8 letter_e_dirs[]={
	1,10,15,21,2
};
u8 letter_c_dirs[]={
	11,15,21,2
};
u8 letter_s_dirs[]={
	11,16,22,16,10
};
u8 letter_v_dirs[]={
	20,5
};
u8 letter_z_dirs[]={
	24,16,24
};
u8 letter_c1_dirs[]={
	15,23
};
u8 letter_z1_dirs[]={
	2,21,16,1
};
u8 letter_s1_dirs[]={
	16,22,16
};
u8 letter_m1_dirs[]={
	19,5,20,5,20
};
u8 letter_L_dirs[]={
	19,24
};
u8 letter_U_dirs[]={
	20,24,6
};
#define GSTR_EXTRA	0
#if GSTR_EXTRA
u8 letter_dy_dirs[]={ // >
	22,15
};
u8 letter_yh_dirs[]={ // ^
	4,20
};
u8 letter_sj_dirs[]={
	16,24,8
}; /*    /_\	*/
u8 letter_g_dirs[]={
	10,16,21,4,18,9,4
};
u8 letter_g1_dirs[]={
	10,16,4,18,9,4
};
u8 letter_a_dirs[]={
	10,16,21,3,21
};
u8 letter_d_dirs[]={
	10,17,20,6,18
};
u8 letter_n_dirs[]={
	19,5,20
};

u8 letter_b_dirs[]={
	18,4,22,16,11
};
u8 letter_q_dirs[]={
	10,16,21,4,18,3
};
u8 letter_q1_dirs[]={
	10,16,4,18,3
};
u8 letter_p_dirs[]={
	2,22,16,8,19
};
u8 letter_p1_dirs[]={
	23,16,8,19
};
#endif
typedef struct
{
    u8* p_lttr;
    u8  valid_elem_num;
    u8  gstr_tag;
	u8  code;
}lttr_cmp_data;

lttr_cmp_data letters[] = {
    {letter_w_dirs,   sizeof(letter_w_dirs),   0x31,  	'W'},
    {letter_m_dirs,   sizeof(letter_m_dirs),   0x32, 	'M'},
    {letter_e_dirs,   sizeof(letter_e_dirs),   0x33, 	'e'},
    {letter_c_dirs,   sizeof(letter_c_dirs),   0x34, 	'C'},
	{letter_s_dirs,	  sizeof(letter_s_dirs),   0x46, 	'S'},
	{letter_v_dirs,	  sizeof(letter_v_dirs),   0x54, 	'V'},
	{letter_z_dirs,	  sizeof(letter_z_dirs),   0x65, 	'Z'},
	{letter_L_dirs,   sizeof(letter_L_dirs),   0x44,	'L'},
	{letter_c1_dirs,  sizeof(letter_c1_dirs),  0x34, 	'C'},
	{letter_s1_dirs,  sizeof(letter_s1_dirs),  0x46, 	'S'},
	{letter_m1_dirs,  sizeof(letter_m1_dirs),  0x32, 	'M'},
	{letter_z1_dirs,  sizeof(letter_z1_dirs),  0x65,	'Z'},

#if GSTR_EXTRA
	{letter_dy_dirs,  sizeof(letter_dy_dirs),  0x52, 	'>'},
	{letter_yh_dirs,  sizeof(letter_yh_dirs),  0x53,	'^'},
	{letter_sj_dirs,  sizeof(letter_sj_dirs),  0x55,	'+'},
	{letter_g_dirs,   sizeof(letter_g_dirs),   0x35,	'g'},
	{letter_g1_dirs,  sizeof(letter_g1_dirs),  0x35,	'g'},
	{letter_q_dirs,   sizeof(letter_q_dirs),   0x43,	'q'},
	{letter_q1_dirs,  sizeof(letter_q1_dirs),  0x43,	'q'},
	{letter_a_dirs,   sizeof(letter_a_dirs),   0x36,	'a'},
	{letter_d_dirs,   sizeof(letter_d_dirs),   0x37,	'd'},
	{letter_n_dirs,   sizeof(letter_n_dirs),   0x40,	'n'},
	{letter_b_dirs,   sizeof(letter_b_dirs),   0x42,	'b'},
	{letter_p_dirs,   sizeof(letter_p_dirs),   0x45,	'p'},
	{letter_p1_dirs,  sizeof(letter_p1_dirs),  0x45,	'p'},
#endif		
};

typedef struct{
	u8  dir;
	u16 s_x;
	u16 s_y;
	u16 e_x;
	u16 e_y;
}gstr_feature_t;


u16 ratio0[sizeof(letters)/sizeof(lttr_cmp_data)] = {0};
u16 ratio1[sizeof(letters)/sizeof(lttr_cmp_data)] = {0};
#define MAX_DIR_CNT     (32)
gstr_feature_t gstr_ftrs[MAX_DIR_CNT];//features;
u8 dir_chg_cnt_real;
u8 dir_chg_cnt;

u8 tan_campare_table[4][6] = {
	{12,11,10,9, 8, 7},
	{13,14,15,16,17,18},
	{1, 2, 3, 4, 5, 6},
	{24,23,22,21,20,19},
};

#define ABS(a)   (((a)>0)?((a)):(-(a)))
int get_dis_square(int x1, int y1, int x2, int y2)
{
    int cc;

    cc = (x1-x2)*(x1-x2);
    cc += (y1-y2)*(y1-y2);
    
    return cc;
}

u8 get_dir(short x_old, short y_old, short x_new, short y_new) {
	u8 rough_dir = (x_new>x_old?2:0)+(y_new>y_old?1:0);//get_rough_dir(x_old, y_old, x_new, y_new);
	u8 idx = 5;
	if(x_old != x_new) {
		int tan_val=(ABS(y_old-y_new)<<10)/ABS(x_old-x_new);			
        idx = (tan_val<268)?0:((tan_val<577)?1:((tan_val<1000)?2:((tan_val<1732)?3:((tan_val<3732)?4:5))));
	}
	return tan_campare_table[rough_dir][idx];
}

bool is_adjacent(u8 dir1, u8 dir2, u8 range)//有bool 类型?
{
	return (range>=ABS(dir1-dir2) || (24-range)<=ABS((dir1-dir2)));
}

void dir_merge(u16 m_dx, u16 m_dy) {
	int i=1;
	u8 vld_pre = 0;
	u16 tmp = (m_dx+m_dy)/10; 
	dir_chg_cnt_real = dir_chg_cnt;
	for(; i<=dir_chg_cnt; i++) {
		if(i < dir_chg_cnt) {
			u8 dir1 = gstr_ftrs[vld_pre].dir;
			u16 dx1 = ABS(gstr_ftrs[vld_pre].s_x - gstr_ftrs[vld_pre].e_x);
			u16 dy1 = dx1+ABS(gstr_ftrs[vld_pre].s_y - gstr_ftrs[vld_pre].e_y);
			u8 dir2 = gstr_ftrs[i].dir;
			u16 dx2 = ABS(gstr_ftrs[i].s_x - gstr_ftrs[i].e_x);
			u16 dy2 = dx2+ABS(gstr_ftrs[i].s_y - gstr_ftrs[i].e_y);
			if(is_adjacent(dir1, dir2, 1) || (is_adjacent(dir1, dir2, 6) && (dy1<tmp || dy2<tmp))) {
				gstr_ftrs[vld_pre].dir = get_dir(gstr_ftrs[vld_pre].s_x,gstr_ftrs[vld_pre].s_y,gstr_ftrs[i].e_x,gstr_ftrs[i].e_y);
				gstr_ftrs[vld_pre].e_x = gstr_ftrs[i].e_x;
				gstr_ftrs[vld_pre].e_y = gstr_ftrs[i].e_y;
				gstr_ftrs[i].dir = 0;
				dir_chg_cnt_real--;
			}
			else {
				vld_pre++;
				gstr_ftrs[vld_pre].dir = gstr_ftrs[i].dir;
				gstr_ftrs[vld_pre].s_x = gstr_ftrs[i].s_x;
				gstr_ftrs[vld_pre].s_y = gstr_ftrs[i].s_y;
				gstr_ftrs[vld_pre].e_x = gstr_ftrs[i].e_x;
				gstr_ftrs[vld_pre].e_y = gstr_ftrs[i].e_y;
			}
		}
	}
}

u8 is_roll_moving(void)
{
	int i=1;
	u8 original_dir;
	u8 dir;
	for(; i<dir_chg_cnt_real; i++) {
		u8 dir1 = gstr_ftrs[i-1].dir;
		u8 dir2 = gstr_ftrs[i].dir;
		if(is_adjacent(dir1, dir2, 9)) {
			if((dir1<dir2 && dir2-dir1<9) || (dir1>dir2 && dir1-dir2>14)) {//轨迹夹角不能太小
				dir = 1;
			}
			else {
				dir = 0;
			}
			if(1==i) {
				original_dir = dir;
			}
			else {
				if(dir != original_dir) {
					return 0;
				}
			}
		}
		else {
			return 0;
		}
	}
	return 1;
}

void dir_update(short x_old, short y_old, short x_new, short y_new) {
    if(dir_chg_cnt >= MAX_DIR_CNT) {
        return;
    }
	gstr_ftrs[dir_chg_cnt].dir = get_dir(x_old, y_old, x_new, y_new);//计算细分的方向，分辨率为24
	gstr_ftrs[dir_chg_cnt].s_x = x_old;
	gstr_ftrs[dir_chg_cnt].s_y = y_old;
	gstr_ftrs[dir_chg_cnt].e_x = x_new;
	gstr_ftrs[dir_chg_cnt].e_y = y_new;
	dir_chg_cnt++;
}

static u32 last_x, last_y;

u8 ChrRecognize(u8 touch_flg, short x, short y) {
     static short max_x, max_y, min_x, min_y;
     static short start_x, start_y;
     static char rough_dir_last;
	 u8 rough_dir;
	 u16 max_dx ;//计算画线区域矩形大小
     u16 max_dy ;
	 u8 dir3 ;
	 u8 tmp ;
	 int tmp_k;
	 u16 res_x ;//屏幕分辨率，这个需要改
     u16 res_y ;
      int i,j,k;
     if(touch_flg) {
         if(!gstr_data_cnt) {//第一帧，
             max_x = min_x = x;
             max_y = min_y = y;
             start_x = x;
             start_y = y;
             dir_chg_cnt = 0;
             memset(ratio0,0,sizeof(ratio0));
             memset(ratio1,0,sizeof(ratio1));
             memset(gstr_ftrs,0,sizeof(gstr_ftrs));
         }
         else {
             if(x > max_x) {//计算画线区域矩形大小
                 max_x = x;
             }
             else if(x < min_x) {
                 min_x = x;
             }
             if(y > max_y) {
                 max_y = y;
             }
             else if(y < min_y) {
                 min_y = y;
             }
 
             rough_dir = (x>last_x?2:0)+(y>last_y?1:0);//计算相对于上一点所在象限           
             if(2 >= gstr_data_cnt) {//忽略第2点的方向，取点2-3的方向为初始方向
                 rough_dir_last = rough_dir;
             }
             else { 
                 if(rough_dir_last != rough_dir) { //不在同一象限，大方向有变化
                     rough_dir_last = rough_dir;//下一段轨迹的初始方向
                     dir_update(start_x,start_y,last_x,last_y);//保存这一段轨迹
                     start_x = last_x;//下一段轨迹的起点
                     start_y = last_y;
                 }
             }
         }
     }
     else if(gstr_data_cnt >= 1){//松开，并且本次触摸多于一点
         dir_update(start_x,start_y,last_x,last_y);//保存最后一段轨迹
         max_dx = max_x-min_x;//计算画线区域矩形大小
         max_dy = max_y-min_y;
         dir_merge(max_dx,max_dy);//合并一些在大方向临界区，方向变化不大的轨迹，如6和7
         res_x = rpt_res_x;//屏幕分辨率，这个需要改
         res_y = rpt_res_y;
         if(1 == dir_chg_cnt_real) {//画直线
             u8  dir = gstr_ftrs[0].dir;
             if(max_dx > (res_x>>1)) {//画线长度大于屏幕一半
                 if(1==dir || 24==dir) { //右
                     return 0x21;
                 }
                 else if(12==dir || 13==dir) { //左 
                     return 0x20;
                 }
             }
             else if(max_dy > (res_y>>1)) {
                 if(6==dir || 7==dir) {  //上
                     return 0x22;
                 }
                 else if(18==dir || 19==dir) {  //下
                     return 0x23;
                 }
             }
         }
         else if(dir_chg_cnt_real > 3) {
             u8 rolling = is_roll_moving();
             if(rolling && dir_chg_cnt_real<7) {
                 u16 x1 = gstr_ftrs[0].s_x;
                 u16 y1 = gstr_ftrs[0].s_y;
                 u16 x2 = gstr_ftrs[dir_chg_cnt_real-1].e_x;
                 u16 y2 = gstr_ftrs[dir_chg_cnt_real-1].e_y;
                 if((ABS(x1-x2) < ((3*max_dx)>>3)) && (ABS(y1-y2) < ((3*max_dy)>>3))) {//起点、终点距离不能太远
                     return 0x30;
                     //return 'O';
                 }
             }
         }
         if(max_dx>=res_x/10 && max_dy>res_y>>4){//画的图案不能太小
            
             u8 match_cnt[sizeof(letters)/sizeof(lttr_cmp_data)] = {0};
             for(i=0; i<sizeof(letters)/sizeof(lttr_cmp_data);i++) {//遍历符号表
                 u8 rel_cnt = dir_chg_cnt_real;
                 for(j=0,k=0; j<letters[i].valid_elem_num&&k<dir_chg_cnt_real;j++,k++) {//遍历该符号的特征方向
                     u8 dir1 = gstr_ftrs[k].dir;//实际画线方向
                     u8 dir2 = letters[i].p_lttr[j];//符号表中的方向
                     if(is_adjacent(dir1, dir2, 2)) {//相差小于2
                         match_cnt[i]++;//实际画线与符号表匹配度加1
                         dir3 = gstr_ftrs[k+1].dir;
                         tmp_k = k;
                         while(k+1<dir_chg_cnt_real && is_adjacent(dir3, dir2, 2)) {//跳过实际画线中没有匹配的方向
                             k++;
                             dir3 = gstr_ftrs[k+1].dir;
                             rel_cnt--;
                         }
                         if(6==i || 11==i) { //Z
                             int dis = max_dx;
                             if(0==j && 6==i) {
                                 dis = ABS(gstr_ftrs[0].s_x - gstr_ftrs[k].e_x);
                             }
                             else if(letters[i].valid_elem_num-1 == j) {
                                 dis = ABS(gstr_ftrs[tmp_k].s_x - gstr_ftrs[dir_chg_cnt_real-1].e_x);
                             }
                             if(3*dis < max_dx) {//轨迹比例失调，认为不匹配
                                 match_cnt[i] = 0;
                                 break;
                             }
                         }
                     }
                     else if(3==i || 4==i){// 'C' 'S' ，可以从符号表第2个方向开始与实际画线对应匹配
                         u8 dir4 = letters[i].p_lttr[j+1];
                         if(!match_cnt[i] && is_adjacent(dir1, dir4, 2)) {//第2个方向匹配了
                             match_cnt[i]++;
                             j++;
                         }
                     }
                 }
                 tmp = match_cnt[i];
                 ratio0[i] = 1000*tmp/letters[i].valid_elem_num;//匹配的数量与符号表特征方向数量的比值
                 ratio1[i] = 1000*tmp/rel_cnt;//匹配的数量与画线方向数量的比值
                 if(ratio0[i]>=750 && ratio1[i]>900) {
                     
				  #if GSTR_EXTRA
                     if(i==16 || i==17) { // q
                         u8 dir = gstr_ftrs[dir_chg_cnt_real-1].dir;
                         if(!is_adjacent(dir, 3,  2)) {
                             continue;
                         }
                     }
				  #endif	
                     if(ratio0[i]>800 || (i>1 && i<10)) {
                         if(7 == i) {//L
                             k = ABS(gstr_ftrs[0].s_x - gstr_ftrs[dir_chg_cnt_real-1].e_x);
                             if(4*k < 3*max_dx) {
                                 return 0;
                             }
                         }
                         if(5 == i) {//V
                             k = ABS(gstr_ftrs[0].s_y - gstr_ftrs[dir_chg_cnt_real-1].e_y);
                             if(2*k > max_dy) {
                                 return 0;
                             }
                         }
                         return letters[i].gstr_tag;//满足两个if 条件，返回匹配到的字符，否则继续在字符表中找
                     }
                 }
             }
         }
         return 0; 
     }
     return 0;
}

typedef unsigned long clock_tm_t;
#if 1
//int gettimeofday(struct timeval *tv, struct timezone *tz);

clock_tm_t get_cur_msec(void) {
    struct timeval t_tmvl;
	clock_tm_t cur_ms;
    do_gettimeofday(&t_tmvl); 
    cur_ms = ((long)t_tmvl.tv_sec)*1000+(long)t_tmvl.tv_usec/1000; 
    return cur_ms;
}
#else
clock_tm_t get_cur_msec(){
    return jiffies*1000/HZ;//转为ms 为单位
}
#endif

u8 DoubleCheck(int touch_flg, int x, int y){
    static u32 db_clck_status; //double click status;
    static u32 be_rls=1;
    static u32 dis;
    static clock_tm_t touch_tick;
    static clock_tm_t rls_tick;
    clock_tm_t cur_tick = get_cur_msec();
    u32 rls_interval = (u32)(cur_tick - rls_tick);//两次点击间隔;
    u32 touch_interval = (u32)(cur_tick - touch_tick);//按下松开间隔
    
    if(be_rls && rls_interval>500){ //>500ms //新的触摸先清除上一次触摸的状态
        db_clck_status = 0;
    }

    if(touch_flg) {//有触摸
        if(be_rls) {//按下第一帧
            dis = get_dis_square(last_x,last_y,x,y);//两次单击的位置间距
            if(1==db_clck_status) {//第二次单击
                if(dis<2500 && rls_interval<500) {//>70ms; <300ms 
                    db_clck_status++;// 2
                }else{
                    db_clck_status = 0;
                }
            }
            dis=0;
            touch_tick = get_cur_msec();
        }
        else {
            dis += get_dis_square(last_x,last_y,x,y);//计算按下时总的移动距离
        }
        be_rls = 0;
    }
    else {//触摸松开
        if(dis<128 /*&& touch_interval>10*/ && touch_interval<200) {//>10ms; <200ms//单次按下时间
            db_clck_status++;// 1, 3
        }
        else{
            if(db_clck_status == 2){//第二次按下移动了或按下时间太长
                db_clck_status =0;
            }
        }

        rls_tick = get_cur_msec();
        be_rls = 1;
    }


    if(3==db_clck_status) {//双击松开
        db_clck_status = 0;
        return 0x24;
    }
    return 0;
}

void tlsc6x_gstr_init_para(int x_pixel,int y_pixel,int time_slot,int cut_x_pixel,int cut_y_pixel)
{
    rpt_res_x = x_pixel;
    rpt_res_y = y_pixel;
}
#endif


/*******************************************************************************
* Name: tlsc6x_gesture_init
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
int tlsc6x_gesture_init(struct input_dev *input_dev)
{
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	//input_set_capability(input_dev, EV_KEY, KEY_GSTR_U); 
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_UP); 
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_LEFT); 
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_RIGHT); 
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_C);
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_O);
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_E); 
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_M); 
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_W);
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_L);
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_S); 
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_V);
	input_set_capability(input_dev, EV_KEY, KEY_TL_GSTR_Z);
		
    __set_bit(KEY_TL_GSTR_UP, input_dev->keybit);
    __set_bit(KEY_TL_GSTR_DOWN, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_LEFT, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_RIGHT, input_dev->keybit);
	//__set_bit(KEY_GSTR_U, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_C, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_O, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_E, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_M, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_W, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_L, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_S, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_V, input_dev->keybit);
	__set_bit(KEY_TL_GSTR_Z, input_dev->keybit);

	return 0;
}

/*******************************************************************************
* Name: tlsc6x_check_gesture
* Brief:
* Input:
* Output: None
* Return: None
*******************************************************************************/
static int tlsc6x_check_gesture(struct input_dev *input_dev,int gesture_id)
{
    int keycode = 0;

    switch(gesture_id){
        case GSTR_TL_DOUBLECLICK:
            keycode = KEY_POWER;
            break;
#if 0
        case GSTR_TL_LEFT:
            keycode =KEY_LEFT;
            break;
        case GSTR_TL_RIGHT:
            keycode = KEY_RIGHT;
            break;
        case GSTR_TL_UP:
            keycode = KEY_UP;
            break;
        case GSTR_TL_DOWN:
            keycode = KEY_DOWN;
            break;
        case GSTR_TL_O:
            keycode = KEY_O;			
            break;
        case GSTR_TL_W:
            keycode = KEY_W;			
            break;
        case GSTR_TL_M:
            keycode = KEY_M;			
            break;
        case GSTR_TL_E:
            keycode = KEY_E;			
            break;
        case GSTR_TL_C:
            keycode = KEY_C;			
            break;
        case GSTR_TL_V:
            keycode = KEY_V;
            break;
        case GSTR_TL_L:
            keycode = KEY_L;
            break;
        case GSTR_TL_S:
            keycode = KEY_S;
            break;
        case GSTR_TL_Z:
            keycode = KEY_Z;
            break;
        case KEY_POWER:
            keycode = KEY_POWER;			
            break;
#endif
        default:
            break;
    }
    if(keycode > 0){
        input_report_key(input_dev, keycode, 1);
        input_sync(input_dev);
        input_report_key(input_dev, keycode, 0);
        input_sync(input_dev);	
    }
    //msleep(200);

    return keycode;
}
extern int tlsc6x_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
/************************************************************************
* Name: tlsc6x_read_gstr_data
* Brief: read data from TP register
* Input: no
* Output: no
* Return: fail <0
***********************************************************************/
int tlsc6x_read_gstr_data(struct input_dev * input_dev)
{
    u8 buf[20] = {0};
    int ret = -1;  
    int x,y,dis;
    u8 touch_state;	
    if(NULL == g_tlsc6x_client){
        return -1;
    }
    if(tlsc6x_i2c_read(g_tlsc6x_client, buf,  1, buf, 7) < 0){
        pr_err("%s read_data i2c_rxdata failed.\n", __func__);
        return -1;
    }

    touch_state = buf[2] & 0x07;
    if(1== touch_state) {//在画线过程中
        x = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];  
        y = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
        if(y > rpt_res_y){
            return 0;
        }
        dis = get_dis_square(x, y, last_x, last_y);
        if(!gstr_data_cnt || dis >= 20) {//skip dis
            ChrRecognize(1, x, y);
            DoubleCheck(1, x, y);
            last_x = x;
            last_y = y;
            gstr_data_cnt++;
        }
    }
    else if( gstr_data_cnt && 0 == touch_state){//画线结束，上报符号
        ret = DoubleCheck(0, 0, 0);
        if(0x24 != ret) {
            ret = ChrRecognize(0, 0, 0);
        }
        gstr_data_cnt = 0;//松开后清零，用来区分第一帧
        if(ret > 0){
            return tlsc6x_check_gesture(input_dev,ret);
        }
	    
    }
    return 0;
}