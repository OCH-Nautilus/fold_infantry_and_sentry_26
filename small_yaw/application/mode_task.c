#include "mode_task.h"
#include "config.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "detect.h"
#include "gimbal_task.h"
#include "vision.h"
#include "ins_task.h"
#include "bsp_transmit.h"
#include "trigger_task.h"
mode_t mode;
uint16_t chassis_last_mode=0;
uint16_t chassis_now_mode=0;
uint8_t Rotate_direction=1;//小陀螺旋转方向
uint8_t top_mode=0;
/**
 * @brief 主控模式切换
 * @note  滚轮上切换为键鼠控制，滚轮下切换为遥控器控制
 * @param
 */
void mode_task(void const * argument)
{
  /* USER CODE BEGIN mode_task */
     mode_init();
	vTaskDelay(10);
  /* Infinite loop */
  for(;;)
  {
		infantry_sentry_ctrl();//步哨模式切换		
		trigger_state_ctrl();//拨弹盘状态机
		if(mode.infantry_sentry_state==INFANTRY_CTRL)//步兵模式
		{
				infantry_system_conctrl();//键鼠切换
				if(mode.controls_state==RC_ctrl)
				{
					infantry_chassis_rc_ctrl();
					infantry_gimbal_rc_ctrl();
					infantry_vision_rc_ctrl();
					infantry_shoot_rc_ctrl();
					chassis_speed();
				}
				else if(mode.controls_state==KEY_ctrl)
				{
					infantry_chassis_pc_ctrl();
					infantry_gimbal_pc_ctrl();
					infantry_vision_pc_ctrl();
					infantry_shoot_pc_ctrl();
					chassis_speed();
				}
				else
				{
					infantry_chassis_rc_ctrl();
					infantry_gimbal_rc_ctrl();
					infantry_vision_rc_ctrl();
					infantry_shoot_rc_ctrl();
					chassis_speed();
				}

		}
		else//哨兵模式
		{
			sentry_gimbal_state_ctrl();
			sentry_chassis_state_ctrl();
			sentry_shoot_state_ctrl();
		}
		
		
    vTaskDelay(1);
  }
  /* USER CODE END mode_task */
}

/**
 * @brief 主控模式初始化
 * @note  
 * @param
 */

void mode_init()
{
	mode.infantry_sentry_state=INFANTRY_CTRL;
    mode.controls_state=RC_ctrl;
    mode.chassis_speed_state=SPEED_NORMAL;
    mode.chassis_state=CHASSIS_IDLE;
    mode.gimbal_state=GIMBAL_IDLE;
    mode.shoot_state=SHOOT_IDLE;
    mode.trigger_state=TRIGGER_IDLE;
    mode.vision_switch_state=VISION_CLOSE;
	toe_offline[0].communication_state = COMMUNICATION_NONE;
}

void infantry_sentry_ctrl()
{
	if(USART_Rx_data.robot_id==0)
		mode.infantry_sentry_state=INFANTRY_CTRL;
	else
		mode.infantry_sentry_state=SENTRY_CTRL;
}
/******************************步兵状态机*****************************************/
/**
 * @brief 控制模式切换
 * @note
 * @param
 */
void infantry_system_conctrl()
{
	
	if(toe_offline[0].communication_state == COMMUNICATION_NONE)
		mode.controls_state=RC_ctrl;
	
    if (rc_ctrl.rc.wheel >= 600)
        mode.controls_state=RC_ctrl;
    else if (rc_ctrl.rc.wheel <= -600)
        mode.controls_state=KEY_ctrl;
    if(mode.gimbal_state==GIMBAL_IDLE)
        mode.chassis_state=CHASSIS_IDLE;
    switch (mode.controls_state)
    {
    case RC_ctrl:
        if (rc_ctrl.rc.wheel <= -600)
		{
			mode.controls_state = KEY_ctrl;
			mode.chassis_state = CHASSIS_IDLE;
			mode.shoot_state = SHOOT_IDLE;
			mode.gimbal_state = GIMBAL_IDLE;
			mode.trigger_state =TRIGGER_IDLE;
			mode.vision_switch_state = VISION_CLOSE;
			top_mode=0;
			
		}
        break;
    case KEY_ctrl:
        if (rc_ctrl.rc.wheel >= 600)
		{
			mode.controls_state = RC_ctrl;
			mode.controls_state = KEY_ctrl;
			mode.chassis_state = CHASSIS_IDLE;
			mode.shoot_state = SHOOT_IDLE;
			mode.gimbal_state = GIMBAL_IDLE;
			mode.trigger_state =TRIGGER_IDLE;
			mode.vision_switch_state = VISION_CLOSE;
			top_mode=0;
		}
        break;
    default:
        break;
    }
}

/**
 * @brief 底盘模式切换
 * @note  优化陀螺模式下掉头切换跟随
 * @param
 */
void chassis_mode_change()
{

	chassis_now_mode = mode.chassis_state;

	
	if (chassis_last_mode == CHASSIS_TOP && chassis_now_mode == CHASSIS_FOLLOW)
	{
		if (Rotate_direction == 1)
		{
			if (USART_Rx_data.chassis_diff_angle < -0.6f || USART_Rx_data.chassis_diff_angle > 0.0f)
				mode.chassis_state = CHASSIS_TOP;
			else
			{
				mode.chassis_state = CHASSIS_FOLLOW;
				Rotate_direction ^=1;
			}
		}
		else if (Rotate_direction == 0)
		{
			if (USART_Rx_data.chassis_diff_angle > 0.6f || USART_Rx_data.chassis_diff_angle < 0.0f)
				 mode.chassis_state = CHASSIS_TOP;
			else
			{
				 mode.chassis_state = CHASSIS_FOLLOW;
				Rotate_direction ^=1;
			}
		}
	}
	
	
	
	chassis_last_mode =  mode.chassis_state;
}

/**
 * @brief 遥控器控制底盘
 * @note
 * @param
 */uint8_t last_s_l=0;
void infantry_chassis_rc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE || rc_ctrl.rc.s[0] == 2||USART_Rx_data.chassis_if_blackout)
		mode.chassis_state = CHASSIS_IDLE; // 无力
	else if (mode.gimbal_state != GIMBAL_IDLE)
	{
		switch (rc_ctrl.rc.s[0])
		{
		case 1: // 陀螺
			if(CHASSIS_LIMIT())
				mode.chassis_state = CHASSIS_TOP;//mode.chassis_state = CHASSIS_TOP;
			else
				mode.chassis_state = CHASSIS_IDLE; // 无力
			break;
		case 3: // 跟随
			if(CHASSIS_LIMIT())
				mode.chassis_state = CHASSIS_FOLLOW;
			else
				mode.chassis_state = CHASSIS_IDLE; // 无力
			break;
		case 2:
			mode.chassis_state = CHASSIS_IDLE; // 无力
			break;
		default:
			mode.chassis_state = CHASSIS_IDLE; // 无力
			break;
		}
	}
	else
		mode.chassis_state = CHASSIS_IDLE; // 无力

	//变速小陀螺模式
	if(last_s_l != 1&&rc_ctrl.rc.s[0]==1)
	{	
		if(++top_mode>=2)
			top_mode=0;
	}
	
	chassis_mode_change();
	last_s_l=rc_ctrl.rc.s[0];
}

/**
 * @brief 遥控器控制云台
 * @note
 * @param
 */

void infantry_gimbal_rc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE||small_yaw_offline_protect())
	{
		mode.gimbal_state = GIMBAL_IDLE;
	}
	if (toe_offline[0].communication_state == COMMUNICATION_NORMAL&&!small_yaw_offline_protect())
	{//未进入视觉模式下，左拨杆最上是折叠模式，中下是正常模式
		if (rc_ctrl.rc.s[1] != 1)
		{
			if(rc_ctrl.rc.s[0]!=1)
				mode.gimbal_state = GIMBAL_NORMAL;
			else 
				mode.gimbal_state = GIMBAL_NORMAL;//GIMBAL_FOLD
		}
		else
			mode.gimbal_state = GIMBAL_VISION;
		//直接从折叠进入视觉
		if(rc_ctrl.rc.s[1]==1&&GIMBAL.last_mode == GIMBAL_FOLD)
		{
			if(GIMBAL.rise_over==0)
				mode.gimbal_state = GIMBAL_NORMAL;
			else
				mode.gimbal_state = GIMBAL_VISION;
		}

		//平头
		if (GIMBAL.last_mode == GIMBAL_IDLE && mode.gimbal_state != GIMBAL_IDLE)
		{
			GIMBAL.PT_flag = 1;
		}
	}

	GIMBAL.last_mode = mode.gimbal_state;
}

/**
 * @brief 遥控器控制视觉
 * @note
 * @param
 */
void infantry_vision_rc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE)
	{
		mode.vision_switch_state = VISION_CLOSE;
	}
	else
	{
		if (rc_ctrl.rc.s[0] == 3)
			mode.vision_switch_state = VISION_ARMOR;//VISION_SMALL_BUFF
		else if (rc_ctrl.rc.s[0] == 1)
			mode.vision_switch_state = VISION_ARMOR;//VISION_BIG_BUFF
		else if (rc_ctrl.rc.s[0] == 2)
			mode.vision_switch_state = VISION_ARMOR;
	}
}

/**
 * @brief 遥控器控制摩擦轮
 * @note
 * @param
 */
void infantry_shoot_rc_ctrl()
{
	if (rc_ctrl.rc.s[1] == 2 || toe_offline[0].communication_state == COMMUNICATION_NONE||mode.gimbal_state!=GIMBAL_FOLD)
		mode.shoot_state=SHOOT_IDLE;
	if (rc_ctrl.rc.s[1] != 2 && toe_offline[0].communication_state == COMMUNICATION_NORMAL)
		mode.shoot_state=SHOOT_OPEN;
}




 /**
 * @brief 键盘控制底盘
 * @note
 * @param
 */int a1,b1,c1,d1;

void infantry_chassis_pc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE||USART_Rx_data.chassis_if_blackout)
	{
		mode.chassis_state = CHASSIS_IDLE; // 无力
	}
	else
	{
		if (mode.gimbal_state!=GIMBAL_IDLE)
		{
			if(GIMBAL.IF_DT_OVER==0)//处在掉头
			{
				if(rc_ctrl.keyboard.key_CTRL&&CHASSIS_LIMIT())
					mode.chassis_state=CHASSIS_TOP;				
				else 
					mode.chassis_state=CHASSIS_IDLE;
			}
			else
			{
				if(rc_ctrl.keyboard.key_CTRL&&CHASSIS_LIMIT())
					mode.chassis_state=CHASSIS_TOP;
				else if(!rc_ctrl.keyboard.key_CTRL&&CHASSIS_LIMIT())
					mode.chassis_state=CHASSIS_FOLLOW;
				else
					mode.chassis_state=CHASSIS_IDLE;
			}
		}
		else
		{
			mode.chassis_state=CHASSIS_IDLE;
		}
			
	}

	//变速小陀螺模式
	
			top_mode=rc_ctrl.keyboard.flag_F;
	
	chassis_mode_change();
}

/**
 * @brief 键盘控制云台
 * @note
 * @param
 */
void infantry_gimbal_pc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE||small_yaw_offline_protect())
		mode.gimbal_state = GIMBAL_IDLE;
	else 
	{
		if (rc_ctrl.mouse.press_r == 0)//不处于视觉模式
		{
			if (rc_ctrl.keyboard.flag_X == 0)
				mode.gimbal_state = GIMBAL_NORMAL;
			else
				mode.gimbal_state = GIMBAL_FOLD;
		}
		else if (GIMBAL.IF_DT_OVER == 1&&mode.gimbal_state != GIMBAL_FOLD&&GIMBAL.rise_over==1)
			mode.gimbal_state = GIMBAL_VISION;
	}

	

	if (rc_ctrl.keyboard.key_B == 1)
		GIMBAL.PT_flag = 1;
	else
		GIMBAL.PT_flag = 0;


	
	if (rc_ctrl.keyboard.key_R == 1 && GIMBAL.IF_DT_OVER == 1 && (mode.gimbal_state == GIMBAL_NORMAL||mode.gimbal_state == GIMBAL_FOLD)) // 调头——键鼠
	{
		GIMBAL.IF_DT = 1;
		GIMBAL.IF_DT_OVER = 0;
	}


	GIMBAL.last_mode = mode.gimbal_state;
}

/**
 * @brief 键鼠控制视觉
 * @note
 * @param
 */
void infantry_vision_pc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE)
	{
		mode.vision_switch_state = VISION_CLOSE;
	}
	else
	{
		if (mode.gimbal_state == GIMBAL_VISION)
		{
//			if (rc_ctrl.mouse.press_l == 1)
//				mode.vision_switch_state = VISION_ARMOR;
			if (rc_ctrl.keyboard.key_V)
				mode.vision_switch_state = VISION_SMALL_BUFF;
			else if (rc_ctrl.keyboard.key_Z)
				mode.vision_switch_state = VISION_BIG_BUFF;
			else
				mode.vision_switch_state = VISION_ARMOR;
		}
	}
}

/**
 * @brief 键鼠控制摩擦轮
 * @note
 * @param
 */
void infantry_shoot_pc_ctrl()
{
	if(rc_ctrl.keyboard.flag_C==1&&toe_offline[0].communication_state == COMMUNICATION_NORMAL&&mode.gimbal_state!=GIMBAL_FOLD)
		 mode.shoot_state=SHOOT_OPEN;
	else
		 mode.shoot_state=SHOOT_IDLE;
}



/**
 * @brief 云台模式切换数据重置
 * @note
 * @param
 */
void Vision_to_Normal_init()
{
	if (GIMBAL.last_mode == GIMBAL_VISION &&mode.gimbal_state != GIMBAL_VISION)
	{
		GIMBAL.yaw_target = INS.YawTotalAngle;
		GIMBAL.pitch_target = INS.Pitch;
		mode.trigger_state =TRIGGER_IDLE;
		
		GIMBAL.pitch_target = pitch_protect(GIMBAL.pitch_target);
	}

	GIMBAL.last_mode = mode.gimbal_state;
}

void Normal_to_Vision_init()
{
	if (GIMBAL.last_mode == GIMBAL_NORMAL && mode.gimbal_state == GIMBAL_VISION)
	{
		GIMBAL.pitch_target = INS.Pitch;
		GIMBAL.yaw_target = INS.YawTotalAngle;
	}
	GIMBAL.last_mode = mode.gimbal_state;
}
// 底盘加速度
void chassis_speed(void)
{
	if (rc_ctrl.keyboard.key_Shift)
	{
		mode.chassis_speed_state = SPEED_SHIFT;
	}
	else if (rc_ctrl.keyboard.flag_E)
	{
		mode.chassis_speed_state = SPEED_FLY;
	}
	else
	{
		mode.chassis_speed_state = SPEED_NORMAL;
	}
}

/**
 * @brief 底盘模式切换限制
 * @note	判断当前pitch角度，俯角过低底盘无力
 * @param 可控底盘返回1，否则返回0
 */
bool_t CHASSIS_LIMIT()
{
	if(INS.Pitch<PITCH_LIMIT)
		return 1;
	else
		return 0;
}

/******************************哨兵状态机*****************************************/
/**
 * @brief 云台
 * @note
 * @param
 */
void sentry_gimbal_state_ctrl()
{
	switch(mode.gimbal_state)
	{
		case GIMBAL_IDLE:
			if(toe_offline[0].communication_state == COMMUNICATION_NONE||small_yaw_offline_protect())
				mode.gimbal_state = GIMBAL_IDLE;
			else if()//折叠		
				mode.gimbal_state = GIMBAL_FOLD;
			else if(IF_DISCERN()==1&&)//不折叠
				mode.gimbal_state = GIMBAL_VISION;
			else 
				mode.gimbal_state = GIMBAL_CRUISE;				
		break;
		case GIMBAL_FOLD:
			if(toe_offline[0].communication_state == COMMUNICATION_NONE||small_yaw_offline_protect())
				mode.gimbal_state = GIMBAL_IDLE;
			else if()//要求折叠
				mode.gimbal_state = GIMBAL_FOLD;
			else 
				mode.gimbal_state = GIMBAL_CRUISE;	
		break;
		case GIMBAL_CRUISE:
			if(toe_offline[0].communication_state == COMMUNICATION_NONE||small_yaw_offline_protect())
				mode.gimbal_state = GIMBAL_IDLE;
			else if()//要求折叠
				mode.gimbal_state = GIMBAL_FOLD;
			else if(IF_DISCERN()==1&&)//不折叠
				mode.gimbal_state = GIMBAL_VISION;
			else 
				mode.gimbal_state = GIMBAL_CRUISE;
		break;
		case GIMBAL_VISION:
			if(toe_offline[0].communication_state == COMMUNICATION_NONE||small_yaw_offline_protect())
				mode.gimbal_state = GIMBAL_IDLE;
			else if(IF_DISCERN()==1&&)//不折叠
				mode.gimbal_state = GIMBAL_VISION;
			else 
				mode.gimbal_state = GIMBAL_CRUISE;
	}
	
}
/**
 * @brief 底盘
 * @note
 * @param
 */
void sentry_chassis_state_ctrl()
{
	switch(mode.chassis_state)
	{
		case CHASSIS_IDLE:
			if(toe_offline[0].communication_state == COMMUNICATION_NONE||USART_Rx_data.chassis_if_blackout)
				mode.chassis_state = CHASSIS_IDLE;
			else
				mode.chassis_state = CHASSIS_NARIGATION;
		break;
		case CHASSIS_NARIGATION:
			if(toe_offline[0].communication_state == COMMUNICATION_NONE)
				mode.chassis_state = CHASSIS_IDLE;
			else 
				mode.chassis_state = CHASSIS_NARIGATION;
		break;
		default:
			
		break;
	}
}

void sentry_vision_rc_ctrl()
{
	if (toe_offline[0].communication_state == COMMUNICATION_NONE)
		mode.vision_switch_state = VISION_CLOSE;
	else
		mode.vision_switch_state = VISION_ARMOR;

}


/**
 * @brief 摩擦轮
 * @note
 * @param
 */
void sentry_shoot_state_ctrl()
{
	switch(mode.shoot_state)
	{
		case SHOOT_IDLE:
			if(toe_offline[0].communication_state == COMMUNICATION_NONE)
				mode.shoot_state = SHOOT_IDLE;
			else 
				mode.shoot_state = SHOOT_OPEN;
		break;
		case SHOOT_OPEN:
			if(toe_offline[0].communication_state == COMMUNICATION_NONE)
				mode.shoot_state = SHOOT_IDLE;
			else 
				mode.shoot_state = SHOOT_OPEN;
		break;
		
	}
}



/**
 * @brief 拨盘
 * @note
 * @param
 */
void trigger_state_ctrl(void)               
{
	TRIGGER.cal_protect_now_time = HAL_GetTick();
	switch(mode.trigger_state)
	{
		case TRIGGER_IDLE:
			   if(( rc_ctrl.rc.s[1] == 2 && mode.controls_state == RC_ctrl)  || toe_offline[DBUS_TOE].communication_state == COMMUNICATION_NONE)//|| ( mode.controls_state == KEY_ctrl)
				 {
					 mode.trigger_state = TRIGGER_IDLE;
				 }
//				 else
//				 {
//				 	 TRIGGER.cal_step[0] = 1;
//					 TRIGGER.cal_step[1] = 0;
//					 TRIGGER.if_cal = 1;
//					 TRIGGER.cal_protect_start_time = HAL_GetTick();
//					 mode.trigger_state = TRIGGER_CAL;
//				 }
				 else
					 mode.trigger_state = TRIGGER_STATIC;
				 break;
				 
		case TRIGGER_STATIC:
			   if(( rc_ctrl.rc.s[1] == 2 && mode.controls_state == RC_ctrl)  || toe_offline[DBUS_TOE].communication_state == COMMUNICATION_NONE)//|| ( mode.controls_state == KEY_ctrl)
					 mode.trigger_state = TRIGGER_IDLE;
//				 else if(if_trigger_cal())
//				 {
//				 	 TRIGGER.cal_step[0] = 1;
//					 TRIGGER.cal_step[1] = 0;
//					 TRIGGER.if_cal = 1;
//					 TRIGGER.cal_protect_start_time = HAL_GetTick();
//					 mode.trigger_state = TRIGGER_CAL;
//				 }
				 else if(((mode.controls_state == RC_ctrl && rc_ctrl.rc.WHEEL_State == DOWN_LONG) || (mode.controls_state == KEY_ctrl && rc_ctrl.mouse.KEY_L_State == PUSH_LONG)) || (mode.gimbal_state==GIMBAL_VISION&&mode.vision_switch_state == VISION_ARMOR &&IF_FIRE()))
				 { 
				   mode.trigger_state = TRIGGER_LONG;
				 }
				 else if(((((mode.controls_state == RC_ctrl && rc_ctrl.rc.WHEEL_State == DOWN_SHORT) || (mode.controls_state == KEY_ctrl && rc_ctrl.mouse.KEY_L_State == PUSH_SHORT)) && mode.vision_switch_state !=VISION_SMALL_BUFF && mode.vision_switch_state != VISION_BIG_BUFF) ||
					       (((mode.controls_state == RC_ctrl && rc_ctrl.rc.WHEEL_State == DOWN_SHORT) || (mode.controls_state == KEY_ctrl && rc_ctrl.mouse.KEY_L_State == PUSH_SHORT)) && (mode.vision_switch_state ==VISION_SMALL_BUFF || mode.vision_switch_state == VISION_BIG_BUFF) && IF_FIRE()))  && TRIGGER.flag_if_single_over == 1 )
				 {
				   mode.trigger_state = TRIGGER_SINGLE;
				   TRIGGER.flag_if_single = 1;
					 TRIGGER.flag_if_single_over = 0;
				 }
				 else 
				 {
					 mode.trigger_state = TRIGGER_STATIC;
				 }
				 break;           
		case TRIGGER_SINGLE:
         if(( rc_ctrl.rc.s[1] == 2 && mode.controls_state == RC_ctrl)  || toe_offline[DBUS_TOE].communication_state == COMMUNICATION_NONE)//|| ( mode.controls_state == KEY_ctrl)
					 mode.trigger_state = TRIGGER_IDLE;
			   else if(TRIGGER.flag_if_flug[0] == 1)
				 {
				   mode.trigger_state = TRIGGER_BACK;
					 TRIGGER.flag_if_back = 1;
					 TRIGGER.flag_if_back_over = 0;
					 TRIGGER.flag_if_single_over = 1;
				 	 TRIGGER.tire_retreat_current = 0;
				 }
			   else if( TRIGGER.flag_if_single_over == 0 )
					 mode.trigger_state = TRIGGER_SINGLE;
				 else
					 mode.trigger_state = TRIGGER_STATIC;
				 break;
				 
		case TRIGGER_LONG:
			   if(( rc_ctrl.rc.s[1] == 2 && mode.controls_state == RC_ctrl)  || toe_offline[DBUS_TOE].communication_state == COMMUNICATION_NONE)//|| ( mode.controls_state == KEY_ctrl)
					 mode.trigger_state = TRIGGER_IDLE;
			   else if(TRIGGER.flag_if_flug[0] == 1)
				 {
				   mode.trigger_state = TRIGGER_BACK;
					 TRIGGER.flag_if_back = 1;
					 TRIGGER.flag_if_back_over = 0;
				 }
				 else if(((mode.controls_state == RC_ctrl && rc_ctrl.rc.WHEEL_State == DOWN_LONG) || (mode.controls_state == KEY_ctrl && rc_ctrl.mouse.KEY_L_State == PUSH_LONG)) || (mode.vision_switch_state == VISION_ARMOR && TRIGGER.vision_fire == 1))
					 mode.trigger_state = TRIGGER_LONG;
				 else
					 mode.trigger_state = TRIGGER_STATIC;
				 break;

		case TRIGGER_BACK:
				 if(( rc_ctrl.rc.s[1] == 2 && mode.controls_state == RC_ctrl)  || toe_offline[DBUS_TOE].communication_state == COMMUNICATION_NONE)//|| ( mode.controls_state == KEY_ctrl)
					 mode.trigger_state = TRIGGER_IDLE;
			   else if(TRIGGER.flag_if_back_over == 0 && TRIGGER.flag_if_flug[1] == 0)
				   mode.trigger_state = TRIGGER_BACK;
				 else
				 {
					 mode.trigger_state = TRIGGER_STATIC;
					 TRIGGER.flag_if_back_over = 1;
				 	 TRIGGER.tire_retreat_current = 0;
					 TRIGGER.tire_retreat_current_back = 0;
				 }
				 break;
		
		case TRIGGER_CAL:
			   if(( rc_ctrl.rc.s[1] == 2 && mode.controls_state == RC_ctrl) || ( mode.controls_state == KEY_ctrl) || toe_offline[DBUS_TOE].communication_state == COMMUNICATION_NONE)					
				 {
					 TRIGGER.cal_protect_start_time = 0;
					 mode.trigger_state = TRIGGER_IDLE;
				 }
				 else if(TRIGGER.if_cal == 1 && (TRIGGER.cal_protect_now_time - TRIGGER.cal_protect_start_time) < TRIGGER_CAL_PROTECT_TIME)
				 {
				   mode.trigger_state = TRIGGER_CAL;
				 }
				 else
				 {
					 TRIGGER.cal_protect_start_time = 0;
					 mode.trigger_state = TRIGGER_STATIC;
				 }
			break;
				 
		default:
			   break;
	}
}

//是否校准
bool if_trigger_cal()
{
  static int cnt_cal=0, flag_cal=0;
	
	if(mode.controls_state == RC_ctrl && rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 3)  
	{
	  cnt_cal++;
	}
	else cnt_cal=0;
		
	if(cnt_cal==10)
		flag_cal=1;
	else
		flag_cal=0;
	
	return flag_cal;
}

