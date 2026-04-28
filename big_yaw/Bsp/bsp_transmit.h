
#ifndef __BSP_TRANSMIT_H_
#define __BSP_TRANSMIT_H_
#include "struct_typedef.h"
#include "usart.h"
#include "stdio.h"

#define RECIVE_TASK_INIT_TIME 10
#define RECIVE_TASK_TIME_MS   1

#define DATA_COUNT_RX	160
#define DATA_COUNT_TX	57
#define DATA_COUNT	36//接收字节数

#define USART_RX_HEAD   0XA5
#define USART_RX_END    0XAA

#define USART_TX_HAED   0XA5
#define USART_TX_END    0XAA

#define USART_DATA_COUNT  48//发送字节数

typedef enum 
{
	COMMUNICATION_NORMAL=0,
	COMMUNICATION_ERR=1,
}communication_state_e;



// 接收端数据结构体
typedef struct
{
    uint8_t head;
    
    // 模式位域 - 总共2个字节
    union ModeUnion {
        uint16_t mode_pack;  // 用于整体操作的16位
        struct ModeBits {
            uint8_t controls_mode      : 2;  // 位0-1
            uint8_t gimbal_mode        : 3;  // 位2-4
            uint8_t vision_mode        : 2;  // 位5-6
            uint8_t shoot_mode         : 2;  // 位7-8
            uint8_t trigger_mode       : 2;  // 位9-10
            uint8_t chassis_mode       : 2;  // 位11-12
            uint8_t chassis_speed_mode : 2;  // 位13-14
			uint8_t infantry_sentry_mode     	 : 1;  // 位15
        } bits;
    } mode;
    
    int16_t rc_ctrl_r_vx;
    int16_t rc_ctrl_r_vy;
    int16_t rc_ctrl_l_vx;
    int16_t rc_ctrl_l_vy;
    float small_yaw_pos;
    float yaw;
    float mouse_vx;
    float mouse_vy;
    
    // 键盘按键位域 - 总共1个字节
    union KeyUnion {
        uint8_t key_pack;  // 用于整体操作的8位
        struct KeyBits {
            uint8_t Key_W : 1;  // 位0
            uint8_t Key_S : 1;  // 位1
            uint8_t Key_A : 1;  // 位2
            uint8_t Key_D : 1;  // 位3
            //uint8_t reserved_keys : 4;  // 位4-7，保留位
						uint8_t Key_Shift : 1;  //位4
						uint8_t Key_Flag_E : 1;//位5
						uint8_t Key_E : 1;//位6
						uint8_t Key_G : 1;//位7
        }bits;
    } key;
		
		union RC_CTRL_S_Union {
        uint8_t rc_s_pack;  // 用于整体操作的8位
        struct Rc_S_Bits {
            uint8_t s_l : 2;  // 位0-1
						uint8_t s_r : 2;	// 位2-3
					
					// 位6-7，保留位
        }bits;
    } rc_ctrl_s;
		
		uint32_t Communication_count;

        union FLAG_Union 
        {
            uint16_t flag_pack;  // 用于整体操作的8位
            struct Flag_Bits 
            {
                uint8_t IF_DISCERN :1;
                uint8_t IF_PT_OVER :1;
                uint8_t stuck_state :1;// 1正常，0卡弹
                uint8_t shoot_l :1;// 位3-7，保留位
                uint8_t shoot_r :1;
				uint8_t	down_over_flag :1;
				uint8_t	rotate_direction:1;
                uint8_t top_mode : 1; // 小陀螺变速模式
                uint8_t detect_flag :1;
				uint8_t DT_OVER_FLAG : 1;
				uint8_t re_flag : 6; // 位8-15，保留位										
            }bits;
         } flag;
    uint8_t tail;
} USART_Rx_data_t;

// 发送端数据结构体
typedef struct
{
	uint8_t head;
	
	float chassis_diff_angle;
	float initial_speed;
	float ins_big_yaw;//大yaw陀螺仪值
	float big_yaw_target;
	uint16_t shooter_barrel_heat_limit;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_17mm_1_barrel_heat; 
	uint16_t chassis_power_limit;
	float real_power;
	uint16_t buffer_energy;
	float cap_v;
	uint32_t Communication_count;
	int16_t speed_out;
	int16_t chassis_given_current;
	int16_t chassis_speed_rpm;
     union FLAG_Tx_Union 
        {
            uint16_t flag_pack;  // 用于整体操作的8位
            struct Flag_Tx_Bits 
            {
                uint8_t chassis_if_blackout :1 ;  
                uint8_t vision_color :1; // 0蓝色 1红色
                uint8_t robot_id :1; // 0步兵 1哨兵
                uint8_t navi_if_conctrl :1; // 导航控制标志位
                uint8_t navi_if_arrived :1; // 是否到达目标点 阈值 0.3m
                uint8_t navi_close_flag :1; // //是否靠近目标点 阈值 1.0m
                uint8_t navi_need_tunnel :1;//是否需要过洞 在靠近洞且规划路径需要过洞时为1
                uint8_t if_lost_navi :1; // 是否丢失导航 0正常 1丢失
                uint8_t reserved_flags_2 :7; // 位8-15，保留
            }bits;
         } flag_tx;
	uint8_t tail;
}USART_TX_data_t;

typedef union
{
    uint32_t data;
    uint8_t d[4];
} Algorithm_int32_u;

typedef union
{
    uint16_t data;
    uint8_t d[2];
} Algorithm_2_u;

typedef union
{
    int16_t data;
    uint8_t d[4];
} Algorithm_int16_u;

typedef union
{
    float data;
    uint8_t d[4];
} Algorithm_fp32_u;
extern USART_Rx_data_t USART_Rx_data;
extern uint8_t USART_Rx_data_handle[DATA_COUNT];
extern uint8_t USART_Tx_buff[USART_DATA_COUNT];

extern USART_TX_data_t  USART_TX_data;

void Transmit_Data_Task(void const *pvParameters);
void Head1_data_Handle (uint8_t *buff,USART_Rx_data_t *data);
void USART_Data_Send( USART_TX_data_t *data , uint8_t *buff);
void USART_Data_Handle(USART_TX_data_t *data);
void USART_Data_init(USART_TX_data_t *data);
extern communication_state_e communication_state;

#endif
