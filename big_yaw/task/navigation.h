#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#define CONST_HEAD0             0XA5     // 帧头
#define CONST_END0              0XAA     // 帧尾

#define CONST_HEAD1             0XB0     // 帧头
#define CONST_END1              0XBB    // 帧尾

#define CONST_HEAD2             0XB1    // 帧头
#define CONST_END2              0XBB    // 帧尾

#define navigation_tx_len   25
#define decision_tx_len     28
#include "stdbool.h"
#include "struct_typedef.h"
#include "bsp_transmit.h"
#include "protocol.h"
#include "main.h"

typedef enum
{
  navi_state_off=0,
  navi_state_on,
  
}Navi_state_t;




//uint8_t m_FrameHead = 0xA5;
//    float linear_vel_x;
//    float linear_vel_y;
//    float angular_z;
//    float cur_x;
//    float cur_y;
//    bool has_path_;
//    bool get_goal;
//    

//    uint8_t if_control;
//    uint8_t seq;
//    uint8_t arrive_flag;  
//    uint8_t close_flag;
//    // uint8_t arrive_flag;
//    uint8_t m_FrameTail = 0xAA;
//typedef struct
//{

////消息包原生协议
//  float navi_vx;//目标x速度
//  float navi_vy;//目标y速度
//  float navi_yaw_diff;//正常情况下的yaw增量，加上当前yaw就是目标yaw 
//  float current_x;//基于建图坐标系的当前x位置
//  float current_y;//基于建图坐标系的当前y位置
//  uint8_t If_get_path;//导航是否获取到路径
//  uint8_t get_goal;//导航是否获取到目标
//  uint8_t if_arrived;//是否到达目标点 阈值 0.3m
//  uint8_t close_flag;//是否靠近目标点 阈值 1.0m
//  //过洞相关
//  uint8_t need_tunnel;//是否需要过洞 在靠近洞且规划路径需要过洞时为1
//  float tunnel_yaw_error;//过洞情况下的yaw增量，加上当前yaw就是目标yaw
//     //yaw_set = ins.yaw + tunnel_yaw_error
//  uint8_t seq;//包序号
//  
//  
////解算以及处理后的一些数据  
//  float navigate_yaw_target;
//	float chassis_vx;//实际给轮子电机的目标转速
//	float chassis_vy;
//  uint8_t if_control;
//	
//  uint8_t if_lost_navi;
// 
//  float yaw_target;//导航目标方向角
////  uint8_t seq;//包序号
//}navigation_rx_t;

typedef struct
{

//消息包原生协议
  float navi_vx;//目标x速度
  float navi_vy;//目标y速度
  float navi_yaw_diff;//正常情况下的yaw增量，加上当前yaw就是目标yaw
  float current_x;//基于建图坐标系的当前x位置
  float current_y;//基于建图坐标系的当前y位置
  uint8_t If_get_path;//导航是否获取到路径
  uint8_t get_goal;//导航是否获取到目标
  uint8_t if_arrived;//是否到达目标点 阈值 0.3m
  uint8_t close_flag;//是否靠近目标点 阈值 1.0m
	
	
  //过洞相关
  uint8_t need_tunnel;//是否需要过洞 在靠近洞且规划路径需要过洞时为1
  float tunnel_yaw_error;//过洞情况下的yaw增量，加上当前yaw就是目标yaw
	uint8_t if_on_attack;//是否追击
//  uint8_t if_on_attack;//正在追击中 1 未追击0
  uint8_t sentry_attitude_switch;//1=进攻/2=防御/3=移动
	float current_navi_yaw;//当前导航坐标系下的yaw角度
  
  uint8_t seq;//包序号
  
  
//解算以及处理后的一些数据  
  float navigate_yaw_target;
 	float chassis_vx;//实际给轮子电机的目标转速
 	float chassis_vy;
  uint8_t if_control;
 
  uint8_t if_lost_navi;
 
  float yaw_target;//导航目标方向角
//  uint8_t seq;//包序号
}navigation_rx_t;



typedef struct
{
  float yaw_diff;
  float yaw_init_ecd;
  float relative_ecd;
  
}odom_navi_t;

#include <stdint.h>

typedef struct __attribute__((packed)) {
  uint8_t if_get_msg: 1;                   // 保留位
  uint8_t IF_Arrived: 1;                 //判断哨兵是否到达指定位置 
  uint8_t IF_3s_NotHurted: 1;           //判断哨兵是否未受击超过3s
  uint8_t IF_10s_NotHurted: 1;           //判断哨兵是否未受击超过10s
  uint8_t IF_5s_NotHurted: 1;           //判断哨兵是否未受击超过5s
  uint8_t IF_3s_NotFound: 1;             //判断哨兵是否未发现敌人超过3s  
  uint8_t IF_5s_NotFound: 1;             //判断哨兵是否未发现敌人超过5s
  uint8_t IF_10s_NotFound: 1;            //判断哨兵是否未发现敌人超过10s
} sentry_decision_data_1_t;

typedef struct __attribute__((packed)) {
    
  uint8_t IF_HP_Less_50: 1;              //判断哨兵是否血量低于50 
  uint8_t IF_HP_Less_100: 1;             //判断哨兵是否血量低于100
  uint8_t IF_base_armor_spred: 1;        //判断己方基地护甲是否展开
  uint8_t IF_outpost_destroyed: 1;       //判断前哨站是否被击毁
  uint8_t  IF_fire_lock: 1;                //判断发射机构是否锁住
  uint8_t IF_allowance_less_50: 1;        // 判断允许发弹量是否小于50
  uint8_t IF_allowance_less_100: 1;        // 判断允许发弹量是否小于100
  uint8_t IF_HP_recover: 1;               // 判断是否回血完成
} sentry_decision_data_2_t;

typedef struct __attribute__((packed)) {
  uint8_t If_on_toss: 1;                  // 判断是否在中央荒地上
  uint8_t If_need_to_enemy_fortress: 1;   // 判断是否需要上敌方堡垒//todo
  uint8_t If_stop_navi: 1;                // 判断是否停下来击打敌人
  uint8_t If_chassis_weak: 1;             // 判断是否进入虚弱模式
  uint8_t If_get_allow_17: 1;             // 判断是否需要补给区补给弹丸
  uint8_t IF_fortress_allow_less_50: 1;   //判断堡垒增益点
  uint8_t IF_energy_Mechanism: 1;         // 判断是否需要给打符的车让位   打符时间到且打符点附近有步兵的时候
  uint8_t IF_need_to_protect: 1;          // 判断是否需要去保护基地
} sentry_decision_data_3_t;

typedef struct __attribute__((packed)) {
uint8_t If_fortress_free: 1;            // 堡垒增益区是否空闲
  uint8_t If_enemy_outpost_lock: 1;       // 判断敌方前哨站是否停转
  uint8_t IF_enemy_outpost_destroyed: 1;  // 判断对方前哨站是否被摧毁
  uint8_t If_moving_v: 1;                 // 判断是否正在过u型弯
  uint8_t If_chip_base: 1;                // 判断基地是否在吊射基地
  uint8_t If_hp_less_200: 1;              // 判断血量是否小于300
  uint8_t If_enemy_small_energy: 1;       // 判断敌方是否开了小能量机关
  uint8_t If_close_to_enemy_out: 1; // 判断是否距离敌方前哨站较近
} sentry_decision_data_4_t;


typedef struct __attribute__((packed))
{
  sentry_decision_data_1_t sentry_decision_data_1;
  sentry_decision_data_2_t sentry_decision_data_2;
  sentry_decision_data_3_t sentry_decision_data_3;
  sentry_decision_data_4_t sentry_decision_data_4;
}sentry_decision_data_t;

typedef struct
{
  uint8_t if_on_vision;
  int16_t enemy_pos_x;
  int16_t enemy_pos_y;
  uint8_t enemy_id;
}enemy_pose_t;



typedef enum
{
  navigation_nav_id=1,
  navigation_decision_id=2,
}navigation_cmd_id_t;

//1+1+4*4+1+2*2+1=24
typedef struct 
{ 
  uint8_t m_FrameHead;//帧头
  uint8_t nav_cmd_id;//命令字 0x01
  float navi_set_x_pos; //云台手发布导航目标点x坐标
  float navi_set_y_pos; //导航目标点y坐标
  float current_yaw; //当前yaw角
  float current_pitch; //当前pitch角
  enemy_pose_t enemy_pose;//视觉识别的地方坐标信息 ，基于当前yaw角的相对坐标系，坐标乘100倍 单位cm
  uint8_t m_FrameTail;//帧尾
  
}navigation_tx_t;


//1+1+4+2+1+2*3+2*2+1=20
typedef struct
{
  uint8_t m_FrameHead;//帧头
  uint8_t decision_cmd_id;//命令字 0x02
  sentry_decision_data_t sentry_decision_data;//哨兵决策打包数据，当前全部用1bit表示，打包成4个uint8_t
  uint16_t game_remain_time;//比赛剩余时间 单位s
  uint8_t game_state;//比赛状态 直接用裁判系统的，0x04比赛开始
  int16_t projectile_allowance_17mm;//剩余发弹量
  uint16_t current_hp;//机器人当前血量
  uint16_t my_base_hp;//我方基地当前血量
  uint16_t we_outpost_hp;//我方前哨站血量
  uint16_t enemy_outpost_hp;//敌方前哨站血量
  int16_t enemy_hero_x;//敌方英雄相对坐标x，单位待商榷
  int16_t enemy_hero_y;//敌方英雄相对坐标y，单位待商榷
  uint8_t real_sentry_attitude_switch;//裁判系统反馈的真实姿态
  uint8_t remaining_energy_flags;//剩余能量信息
  uint8_t if_get_manual_msg;//是否获取到云台手消息
  uint8_t if_get_radar_msg;//是否获得雷达站数据
  uint8_t m_FrameTail;//帧尾
  
}Decision_tx_t;
// 4+4+4+4+4+2+1+1+2+2=28
typedef struct
{

 uint8_t navigation_cmd_id;
 
 float Sx;
 float Sy;
 float Sx_set;
 float Sy_set;
 float steer_real_angle[4];
 float steer_init_ecd[4];
 float real_Vx,real_Vy;
 float real_Vx_c,real_Vy_c;
 float vx,vy;
 float Vx_c,Vy_c;
 float vx_all[4];
 float init_yaw;  // 检测上电那一刻的陀螺仪值，只是为了里程计计算用,后续考虑
 float diff_yaw;
 float diff_angle;
 bool yaw_update;
 float yaw_set;
}location_t;

extern int navi_tx_count;
extern navigation_tx_t navigation_tx;
extern navigation_rx_t navigation_rx;
extern Decision_tx_t Decision_tx;
extern  uint8_t ninin;
extern uint8_t navi_state_get;
extern int navigation_seq;
void navigation_rx_handle(uint8_t *buff,uint32_t Len,navigation_rx_t *data);
void Navigation_Tx_Send(navigation_tx_t *data);
void Decision_Tx_Send(Decision_tx_t *data);
#endif

