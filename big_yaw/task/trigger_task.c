#include "trigger_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "config.h"
#include "CAN_receive.h"
#include "bsp_transmit.h"
#include "math.h"
#include <stdlib.h>
#include "referee.h"
trigger_t TRIGGER;
uint32_t trigger_time;
uint8_t trigger_mode=0;
uint8_t last_trigger_mode=0;
pid_type_def pid_speed_trigger_single,pid_angle_trigger_single;   //单发pid
pid_type_def pid_speed_trigger_long;

void trigger_task(void const * argument)
{
  trigger_init();
  vTaskDelay(10);
  for(;;)
  {
	trigger_heat();
		trigger_retreat();
		trigger_mode_choose();
		trigger_pid_calc();	
     
    vTaskDelay(1);
  }
  
}


/**
 * @brief 拨盘
 * @note  步兵拨盘状态控制
 * @param
 */
void infantry_trigger_state_ctrl(void)               
{
	TRIGGER.cal_protect_now_time = HAL_GetTick();
	switch(trigger_mode)
	{
		case TRIGGER_IDLE:
			   if(( USART_Rx_data.rc_ctrl_s.bits.s_r == 2 && USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL)  || USART_Rx_data.flag.bits.detect_flag == DETECT_NONE)//|| ( mode.controls_state == KEY_ctrl)
				 {
					 trigger_mode = TRIGGER_IDLE;
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
					 trigger_mode = TRIGGER_STATIC;
				 break;
				 
		case TRIGGER_STATIC:
			   if(( USART_Rx_data.rc_ctrl_s.bits.s_r == 2 && USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL)  || USART_Rx_data.flag.bits.detect_flag == DETECT_NONE)//|| ( mode.controls_state == KEY_ctrl)
					 trigger_mode = TRIGGER_IDLE;
//				 else if(if_trigger_cal())
//				 {
//				 	 TRIGGER.cal_step[0] = 1;
//					 TRIGGER.cal_step[1] = 0;
//					 TRIGGER.if_cal = 1;
//					 TRIGGER.cal_protect_start_time = HAL_GetTick();
//					 mode.trigger_state = TRIGGER_CAL;
//				 }
				 else if((((USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL && USART_Rx_data.rc_ctrl_s.bits.WHEEL_State == DOWN_LONG) || (USART_Rx_data.mode.bits.controls_mode == CONTROL_KEYBOARD_CTRL && USART_Rx_data.rc_ctrl_s.bits.KEY_L_State == PUSH_LONG)) || (USART_Rx_data.mode.bits.gimbal_mode==GIMBAL_VISION&&USART_Rx_data.mode.bits.vision_mode == VISION_ARMOR &&USART_Rx_data.flag.bits.fire_flag))&&TRIGGER.weak_flag==0)
				 { 
				   trigger_mode = TRIGGER_LONG;
				 }
				 else if(((((USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL && USART_Rx_data.rc_ctrl_s.bits.WHEEL_State == DOWN_SHORT) || (USART_Rx_data.mode.bits.controls_mode == CONTROL_KEYBOARD_CTRL && USART_Rx_data.rc_ctrl_s.bits.KEY_L_State == PUSH_SHORT)) && USART_Rx_data.mode.bits.vision_mode !=VISION_SMALL_BUFF && USART_Rx_data.mode.bits.vision_mode != VISION_BIG_BUFF) ||
					       (((USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL && USART_Rx_data.rc_ctrl_s.bits.WHEEL_State == DOWN_SHORT) || (USART_Rx_data.mode.bits.controls_mode == CONTROL_KEYBOARD_CTRL && USART_Rx_data.rc_ctrl_s.bits.KEY_L_State == PUSH_SHORT)) && (USART_Rx_data.mode.bits.vision_mode ==VISION_SMALL_BUFF || USART_Rx_data.mode.bits.vision_mode == VISION_BIG_BUFF) && USART_Rx_data.flag.bits.fire_flag))  && TRIGGER.flag_if_single_over == 1 && TRIGGER.weak_flag==0)
				 {
				   trigger_mode = TRIGGER_SINGLE;
				   TRIGGER.flag_if_single = 1;
					 TRIGGER.flag_if_single_over = 0;
				 }
				 else 
				 {
					 trigger_mode = TRIGGER_STATIC;
				 }
				 break;           
		case TRIGGER_SINGLE:
         if(( USART_Rx_data.rc_ctrl_s.bits.s_r == 2 && USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL)  || USART_Rx_data.flag.bits.detect_flag == DETECT_NONE)//|| ( mode.controls_state == KEY_ctrl)
					 trigger_mode = TRIGGER_IDLE;
			   else if(TRIGGER.flag_if_flug[0] == 1)
				 {
				   trigger_mode = TRIGGER_BACK;
					 TRIGGER.flag_if_back = 1;
					 TRIGGER.flag_if_back_over = 0;
					 TRIGGER.flag_if_single_over = 1;
				 	 TRIGGER.tire_retreat_current = 0;
				 }
			   else if( TRIGGER.flag_if_single_over == 0&&TRIGGER.weak_flag==0 )
					 trigger_mode = TRIGGER_SINGLE;
				 else
					 trigger_mode = TRIGGER_STATIC;
				 break;
				 
		case TRIGGER_LONG:
			   if(( USART_Rx_data.rc_ctrl_s.bits.s_r == 2 && USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL)  || USART_Rx_data.flag.bits.detect_flag == DETECT_NONE)//|| ( mode.controls_state == KEY_ctrl)
					 trigger_mode = TRIGGER_IDLE;
			   else if(TRIGGER.flag_if_flug[0] == 1)
				 {
				   trigger_mode = TRIGGER_BACK;
					 TRIGGER.flag_if_back = 1;
					 TRIGGER.flag_if_back_over = 0;
				 }
				 else if((((USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL && USART_Rx_data.rc_ctrl_s.bits.WHEEL_State == DOWN_LONG) || (USART_Rx_data.mode.bits.controls_mode == CONTROL_KEYBOARD_CTRL && USART_Rx_data.rc_ctrl_s.bits.KEY_L_State == PUSH_LONG)) || (USART_Rx_data.mode.bits.vision_mode == VISION_ARMOR && USART_Rx_data.flag.bits.fire_flag))&&TRIGGER.weak_flag==0)
					 trigger_mode = TRIGGER_LONG;
				 else
					 trigger_mode = TRIGGER_STATIC;
				 break;

		case TRIGGER_BACK:
				 if(( USART_Rx_data.rc_ctrl_s.bits.s_r == 2 && USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL)  || USART_Rx_data.flag.bits.detect_flag == DETECT_NONE)//|| ( mode.controls_state == KEY_ctrl)
					 trigger_mode = TRIGGER_IDLE;
			   else if(TRIGGER.flag_if_back_over == 0 && TRIGGER.flag_if_flug[1] == 0)
				   trigger_mode = TRIGGER_BACK;
				 else
				 {
					 trigger_mode = TRIGGER_STATIC;
					 TRIGGER.flag_if_back_over = 1;
				 	 TRIGGER.tire_retreat_current = 0;
					 TRIGGER.tire_retreat_current_back = 0;
				 }
				 break;
		
		case TRIGGER_CAL:
			   if(( USART_Rx_data.rc_ctrl_s.bits.s_r == 2 && USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL) || ( USART_Rx_data.mode.bits.controls_mode == CONTROL_KEYBOARD_CTRL) || USART_Rx_data.flag.bits.detect_flag == DETECT_NONE)					
				 {
					 TRIGGER.cal_protect_start_time = 0;
					 trigger_mode = TRIGGER_IDLE;
				 }
				 else if(TRIGGER.if_cal == 1 && (TRIGGER.cal_protect_now_time - TRIGGER.cal_protect_start_time) < TRIGGER_CAL_PROTECT_TIME)
				 {
				   trigger_mode = TRIGGER_CAL;
				 }
				 else
				 {
					 TRIGGER.cal_protect_start_time = 0;
					 trigger_mode = TRIGGER_STATIC;
				 }
			break;
				 
		default:
			   break;
	}
}

/**
 * @brief 拨弹盘状态机
 * @note  哨兵模式下拨弹盘由视觉触发，视觉识别装甲板且满足开火条件时进入开火状态，否则进入静止状态；卡弹优先级最高，进入卡弹状态后退弹完成进入静止状态；无信号或遥控器切换到无力状态
 * @param
 */
void sentry_trigger_state_ctrl(void)
{
    TRIGGER.cal_protect_now_time = HAL_GetTick();

    uint8_t is_stop =((USART_Rx_data.rc_ctrl_s.bits.s_r == 2 && USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL)  || USART_Rx_data.flag.bits.detect_flag == DETECT_NONE);

    uint8_t fire_cmd = 0;

    // ===== RC模式 =====
    if(USART_Rx_data.mode.bits.controls_mode == CONTROL_RC_CTRL)
    {
        if(USART_Rx_data.rc_ctrl_s.bits.WHEEL_State == DOWN_LONG && TRIGGER.weak_flag == 0)
            fire_cmd = 1;
    }

    // ===== AUTO模式 =====
    else if(USART_Rx_data.mode.bits.controls_mode == CONTROL_AUTO_CTRL)
    {
        if(USART_Rx_data.mode.bits.gimbal_mode == GIMBAL_VISION && USART_Rx_data.mode.bits.vision_mode == VISION_ARMOR && USART_Rx_data.flag.bits.fire_flag && TRIGGER.weak_flag == 0)
        {
            fire_cmd = 1;
        }
    }

    switch(trigger_mode)
    {
        case TRIGGER_IDLE:
            if(is_stop)
                trigger_mode = TRIGGER_IDLE;
            else
                trigger_mode = TRIGGER_STATIC;
        break;

        case TRIGGER_STATIC:
            if(is_stop)
                trigger_mode = TRIGGER_IDLE;
            else if(TRIGGER.flag_if_flug[0])   // 卡弹优先（不受weak_flag影响）
            {
                trigger_mode = TRIGGER_BACK;
                TRIGGER.flag_if_back = 1;
                TRIGGER.flag_if_back_over = 0;
            }
            else if(fire_cmd)
                trigger_mode = TRIGGER_LONG;
            else
                trigger_mode = TRIGGER_STATIC;
        break;

        case TRIGGER_LONG:
            if(is_stop)
                trigger_mode = TRIGGER_IDLE;
            else if(TRIGGER.flag_if_flug[0])
            {
                trigger_mode = TRIGGER_BACK;
                TRIGGER.flag_if_back = 1;
                TRIGGER.flag_if_back_over = 0;
            }
            else if(fire_cmd)
                trigger_mode = TRIGGER_LONG;
            else
                trigger_mode = TRIGGER_STATIC;
        break;

        case TRIGGER_BACK:
            if(is_stop)
                trigger_mode = TRIGGER_IDLE;
            else if(TRIGGER.flag_if_back_over == 0 &&
                    TRIGGER.flag_if_flug[1] == 0)
            {
                trigger_mode = TRIGGER_BACK;
            }
            else
            {
                trigger_mode = TRIGGER_STATIC;
                TRIGGER.flag_if_back_over = 1;
                TRIGGER.tire_retreat_current = 0;
                TRIGGER.tire_retreat_current_back = 0;
            }
        break;

        default:
            trigger_mode = TRIGGER_IDLE;
        break;
    }
}




//拨弹盘初始化
void trigger_init(void)
{
	TRIGGER.flag_shoot_heat_warning = 0;
	TRIGGER.flag_if_back = 0;
	TRIGGER.flag_if_back_over = 1;
	TRIGGER.flag_if_single = 0;
	TRIGGER.flag_if_single_over = 1;

	PID_init(&pid_angle_trigger_single, 0,PID_TRIGGER_ANGLE_SINGLE_KP, PID_TRIGGER_ANGLE_SINGLE_KI, PID_TRIGGER_ANGLE_SINGLE_KD,  PID_TRIGGER_ANGLE_SINGLE_IMAX, PID_TRIGGER_ANGLE_SINGLE_MAX);
  PID_init(&pid_speed_trigger_single,  0,PID_TRIGGER_SPEED_SINGLE_KP, PID_TRIGGER_SPEED_SINGLE_KI, PID_TRIGGER_SPEED_SINGLE_KD, PID_TRIGGER_SPEED_SINGLE_IMAX, PID_TRIGGER_SPEED_SINGLE_MAX);

	PID_init(&pid_speed_trigger_long, 0, PID_TRIGGER_SPEED_LONG_KP, PID_TRIGGER_SPEED_LONG_KI, PID_TRIGGER_SPEED_LONG_KD, PID_TRIGGER_SPEED_LONG_IMAX, PID_TRIGGER_SPEED_LONG_MAX);
}




//拨弹盘pid计算
void trigger_pid_calc(void)
{
	                                  PID_calc(&pid_angle_trigger_single , trigger_motor.total_ecd, TRIGGER.ecd_total);
	 TRIGGER.pid_trigger_single_out = PID_calc(&pid_speed_trigger_single , trigger_motor.speed_rpm , pid_angle_trigger_single.out);
	 TRIGGER.pid_trigger_long_out =   PID_calc(&pid_speed_trigger_long  ,  trigger_motor.speed_rpm , TRIGGER.v_trigger );
	
	#ifdef TRIGGER_SENT  
	if(trigger_mode == TRIGGER_BACK || trigger_mode == TRIGGER_SINGLE || trigger_mode == TRIGGER_STATIC || (trigger_mode == TRIGGER_CAL && TRIGGER.cal_step[0] == 0 && TRIGGER.cal_step[1] == 1))
	{                            
			TRIGGER.pid_trigger_out = TRIGGER.pid_trigger_single_out;
	}
	else if(trigger_mode == TRIGGER_LONG || (trigger_mode == TRIGGER_CAL && TRIGGER.cal_step[0] == 1 && TRIGGER.cal_step[1] == 0))
	{
	  TRIGGER.pid_trigger_out = TRIGGER.pid_trigger_long_out;
	}
	else if(trigger_mode == TRIGGER_IDLE)
	{
	  TRIGGER.pid_trigger_out = 0;
	}	
	#else 
	TRIGGER.pid_trigger_out = 0;
	#endif

}


//模式选择
void trigger_mode_choose(void)    
{
	
	switch(trigger_mode)
	{
		case TRIGGER_SINGLE:    // 单发
			   if(TRIGGER.flag_if_single == 1)
				 {
//					 if(TRIGGER.flag_shoot_heat_warning == 0)
//					 {
					   TRIGGER.ecd_total += TRIGGER_ECD;
//					 }
					 TRIGGER.flag_if_single = 0;
				 }
					
				 if(fabs(trigger_motor.total_ecd - TRIGGER.ecd_total) < TRIGGER_SINGLE_OVER_THRESHOLD && TRIGGER.flag_if_single == 0)
				 {
					 TRIGGER.flag_if_single_over = 1;
				 }
			   break;
		
		case TRIGGER_LONG:      // 连发
			   TRIGGER.v_trigger =  TRIGGER.limit_v;	
         TRIGGER.ecd_total = trigger_motor.total_ecd+TRIGGER_ECD-((int)(trigger_motor.total_ecd - TRIGGER.initial_ecd)%(int)TRIGGER_ECD);
		     break;		
		
		case TRIGGER_BACK:    // 回退
		     if(TRIGGER.flag_if_back == 1)
				 {
					 TRIGGER.ecd_total -= TRIGGER_ECD;
					 TRIGGER.flag_if_back = 0;
				 }
					
				 if(fabs(trigger_motor.total_ecd - TRIGGER.ecd_total) < TRIGGER_BACK_OVER_THRESHOLD && TRIGGER.flag_if_back == 0)
				 {
					 TRIGGER.flag_if_back_over = 1;
					 TRIGGER.tire_retreat_current = 0;
					 TRIGGER.tire_retreat_current_back = 0;
				 }
		     break;
		
		case TRIGGER_CAL:   //校准
			   if(TRIGGER.if_cal == 1)
				 {
				   if(TRIGGER.cal_step[0] == 1)
					 {
						 TRIGGER.ecd_total = trigger_motor.total_ecd;
					   TRIGGER.v_trigger = TRIGGER_CAL_SPEED;
					 }
					 else if(TRIGGER.cal_step[1] == 1)
					 {
					   TRIGGER.v_trigger = 0;
						 TRIGGER.ecd_total = trigger_motor.total_ecd + TRIGGER_CAL_OFFSET_ECD;
						 TRIGGER.initial_ecd = trigger_motor.total_ecd + TRIGGER_CAL_OFFSET_ECD;
						 TRIGGER.cal_step[1] = 0;
						 TRIGGER.if_cal = 0;
					 }			 
				 }
				 
				 if(TRIGGER.cal_step[0] == 1 && TRIGGER.cal_step[1] == 0 && TRIGGER.flag_if_flug[2] == 1)  //第一阶段结束判断
				 {
				   TRIGGER.cal_step[0] = 0;
					 TRIGGER.cal_step[1] = 1;
				 } 
			   break;
			
		case TRIGGER_STATIC:   //不动
			   TRIGGER.v_trigger = 0;
//         TRIGGER.ecd_total=motor[TRIGGER_motor].total_ecd;
		     if(last_trigger_mode == TRIGGER_LONG && TRIGGER.initial_ecd != 0 && trigger_motor.total_ecd != 0)
		      TRIGGER.ecd_total = trigger_motor.total_ecd + TRIGGER_ECD - (int)(trigger_motor.total_ecd - TRIGGER.initial_ecd)%(int)TRIGGER_ECD;
				 TRIGGER.tire_retreat_current = 0;
		     TRIGGER.tire_retreat_current_back = 0;
		     break;
		
		case TRIGGER_IDLE:      // 无力
			   TRIGGER.v_trigger = 0;
         TRIGGER.ecd_total=trigger_motor.total_ecd;
	       TRIGGER.flag_if_back = 0;
	       TRIGGER.flag_if_back_over = 1;
	       TRIGGER.flag_if_single = 0;
	       TRIGGER.flag_if_single_over = 1;
		     break;

		default:
			break;
	}
	last_trigger_mode = trigger_mode;
}

/*******************************************************************摩擦轮其他控制*******************************************************************/
//卡弹计时
void trigger_retreat(void)
{
		if( abs(trigger_motor.speed_rpm) <= 50 && TRIGGER.pid_trigger_out >TRIGGER_FORWARD_TORQUE_THRESHOLD )//如果转速低而输出电流大的情况则判断为卡弹(正拨)
		{
				TRIGGER.tire_retreat_current++;
		}
		else if(TRIGGER.tire_retreat_current>0)
		{
			  TRIGGER.tire_retreat_current--;
		}
		
		 if( abs(trigger_motor.speed_rpm) <= 50 && TRIGGER.pid_trigger_out < (-TRIGGER_BACK_TORQUE_THRESHOLD ))//如果转速低而输出电流大的情况则判断为卡弹(反拨)
		{
				TRIGGER.tire_retreat_current_back++;
		}
		else if(TRIGGER.tire_retreat_current_back>0)
		{
			  TRIGGER.tire_retreat_current_back--;
		}
		
		if( abs(trigger_motor.speed_rpm) <= 50 && TRIGGER.pid_trigger_out < (-TRIGGER_CAL_TORQUE_THRESHOLD ))//如果转速低而输出电流大的情况则判断为卡弹(校准)
		{
				TRIGGER.tire_retreat_current_cal++;
		}
		else if(TRIGGER.tire_retreat_current_cal>0)
		{
			  TRIGGER.tire_retreat_current_cal--;
		}
		

   if(trigger_mode != TRIGGER_CAL)
	 {		 
		if(TRIGGER.tire_retreat_current > TRIGGER_FORWARD_TIME_THRESHOLD)   //正转标志位
			 TRIGGER.flag_if_flug[0] = 1;
		else
			 TRIGGER.flag_if_flug[0] = 0;
		
		if(TRIGGER.tire_retreat_current_back > TRIGGER_BACK_TIME_THRESHOLD) //反转标志位
			 TRIGGER.flag_if_flug[1] = 1;
    else
			 TRIGGER.flag_if_flug[1] = 0;
	}
	else
	{
		if(TRIGGER.tire_retreat_current_cal > TRIGGER_CAL_TIME_THRESHOLD)   //校准标志位
			 TRIGGER.flag_if_flug[2] = 1;
    else
			 TRIGGER.flag_if_flug[2] = 0;		
	}
		
}



//热量反馈检测器（反馈裁判系统下次反馈剩余热量变化-仅连发时使用)
int next_heat_change(void)
{
    int next_Shoot_quantity=0;//下次消耗弹数
    int heat_recover=0;
    int heat_consume=0;
    int heat_change=0;//剩余热量变化
    next_Shoot_quantity=(int)(TRIGGER.limit_v/(V_TRIGGER_15)*0.25f+1);//TRIGGER.limit_v为连发转速，0.25f为反馈时差
//    heat_recover=robot_status.shooter_barrel_cooling_value/10;
    heat_consume=next_Shoot_quantity*10;
    heat_change=heat_recover-heat_consume;
    return heat_change;
}


//枪管热量限制
void trigger_heat(void)
{
int heat_limit=robot_status.shooter_barrel_heat_limit;
int heat=power_heat_data.shooter_17mm_barrel_heat; 
;
	
	if(heat_limit - heat + next_heat_change() < 20)
	{
		trigger_mode = TRIGGER_STATIC;
		TRIGGER.weak_flag = 1;
	}
	else 
	{
		TRIGGER.weak_flag = 0;
	}
//			 
//	 if(robot_status.shooter_barrel_cooling_value == 40 && heat_limit==50)
//	 {
//		TIRGGER.limit_v = V_TIRGGER_10;
//	 }
//	 else
//	 {
//		 if(mode.vision_switch_state == VISION_ARMOR)
//		 {
//			 TIRGGER.limit_v = V_TIRGGER_18;
//		 }
//		   
//		 if(USART_Rx_data.rc_ctrl_key_f == 0&&mode.vision_switch_state == VISION_CLOSE)
//		 {	
	     TRIGGER.limit_v = V_TRIGGER_15;
//		 }
//		 else if(USART_Rx_data.rc_ctrl_key_f == 1&&mode.vision_switch_state == VISION_CLOSE)
//		 {
//		   TRIGGER.limit_v = -V_TRIGGER_20;
//		 }
//	 }
}


