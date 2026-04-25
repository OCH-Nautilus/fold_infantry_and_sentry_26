#ifndef MODE_TASK_H
#define MODE_TASK_H

#include "struct_typedef.h"

typedef enum
{
	INFANTRY_CTRL=0,
	SENTRY_CTRL=1,
}infantry_sentry_t;//步兵哨兵模式

typedef enum
{
	RC_ctrl=0,
	KEY_ctrl=1,
}controls_t;//操作模式


typedef enum
{
    GIMBAL_IDLE = 0,
    GIMBAL_NORMAL = 1,	
	GIMBAL_VISION = 2,
	GIMBAL_FOLD = 3,
	GIMBAL_CRUISE = 4,//云台巡航模式
}gimbal_state_t;//云台模式


typedef enum
{
	VISION_CLOSE = 0,
    VISION_ARMOR =1,
    VISION_SMALL_BUFF =2,
    VISION_BIG_BUFF =3,	
}vision_switch_state_t;//视觉开关


typedef enum
{
	SHOOT_IDLE = 0,
	SHOOT_OPEN = 1,
}shoot_state_t;//摩擦轮模式


typedef enum
{
	TRIGGER_IDLE = 0,
	TRIGGER_SINGLE,
	TRIGGER_LONG,
	TRIGGER_STATIC,
	TRIGGER_BACK,
	TRIGGER_CAL,
}trigger_state_t;//拨弹盘模式


typedef enum
{
	CHASSIS_IDLE = 0,
	CHASSIS_FOLLOW = 1,
	CHASSIS_TOP =2,
	CHASSIS_NARIGATION = 3,//导航模式
}chassis_state_t;//底盘模式


typedef enum
{
	SPEED_NORMAL = 0,
	SPEED_SHIFT =1,	
	SPEED_FLY =2,
}chassis_speed_state_t;//底盘加速度限制


typedef struct 
{
	infantry_sentry_t        infantry_sentry_state;
    controls_t               controls_state;
    gimbal_state_t           gimbal_state;
    vision_switch_state_t    vision_switch_state;
    shoot_state_t            shoot_state;
	trigger_state_t          trigger_state;
	chassis_state_t          chassis_state;
	chassis_speed_state_t    chassis_speed_state;
}mode_t;//模式总控

void mode_init(void);
void chassis_mode_change(void);
void infantry_chassis_rc_ctrl(void);
void infantry_shoot_rc_ctrl(void);
void infantry_system_conctrl(void);
void infantry_chassis_pc_ctrl(void);
void infantry_gimbal_rc_ctrl(void);
void infantry_gimbal_pc_ctrl(void);
void infantry_shoot_pc_ctrl(void);
void infantry_vision_rc_ctrl(void);
void infantry_vision_pc_ctrl(void);
void infantry_trigger_rc_ctrl(void);
void infantry_trigger_pc_ctrl(void);
void Vision_to_Normal_init(void);
void Normal_to_Vision_init(void);
bool_t CHASSIS_LIMIT(void);
void chassis_speed(void);

void infantry_sentry_ctrl(void);
void sentry_gimbal_state_ctrl(void);
void sentry_chassis_state_ctrl(void);	
void sentry_shoot_state_ctrl(void);
void trigger_state_ctrl(void) ;              

extern mode_t mode;

#endif
