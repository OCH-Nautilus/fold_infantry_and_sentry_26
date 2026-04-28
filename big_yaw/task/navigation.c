/*导航信息处理*/

// #include "Odometer.h"
#include "ins_task.h"
#include "navigation.h"
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "usb_device.h"
// #include "bsp_transmit.h"
#include "bsp_transmit.h"
#include "referee.h"
#include "CAN_receive.h"
location_t location =
    {
        .Sx = 0,
        .Sy = 0,
        .Sx_set = 0,
        .Sy_set = 0,
        .yaw_update = 0,
};

// 如果红色开始建图
/*
1如果车体是红色的，那么发过去的坐标都不需要转换
2如果车体是蓝色的，那么发过去的坐标需要经过一个坐标系转换后才可以发过去
3接收到云台手的坐标也是对的，不需要经过任何转换
*/
/*如果蓝色开始建图
1如果车体是红色的，那么发过去的坐标需要转换、
2如果车体是蓝色的，那么发过去的需要转换
3云台手的标点需要经过完整的坐标系转换才能正常
*/

navigation_rx_t navigation_rx;
navigation_tx_t navigation_tx;
Decision_tx_t Decision_tx;

uint8_t Navigate_Tx_buff[navigation_tx_len];
uint8_t Decisin_Tx_buff[decision_tx_len];
int navi_tx_count;
int navigation_seq=0;
uint8_t last_point_get;
uint16_t opopok;
uint8_t navi_state_get;
// uint8_t buff_const;
// uint8_t buff_head;
uint8_t buff_count[100];
int i_count = 0;
int olklk;
// uint8_t rx_data_navi[24]={0};
void navigation_rx_handle(uint8_t *buff, uint32_t Len, navigation_rx_t *data)
{

  navigation_seq++;

  if (buff == NULL)
  {
    return;
  }
  if (buff[0] == CONST_HEAD0 && buff[Len - 1] == CONST_END0)
  {

    navi_tx_count = 0;

    Algorithm_fp32_u Vx, Vy, yaw, Sx, Sy,tunnel_yaw;

    for (int i = 0; i < 4; i++)
    {

      Vx.d[i] = buff[i + 1];
      Vy.d[i] = buff[i + 5];
      yaw.d[i] = buff[i + 9];
      Sx.d[i] = buff[i + 13];
      Sy.d[i] = buff[i + 17];
      tunnel_yaw.d[i] = buff[i+26];
      
    }
    data->If_get_path = buff[21];
    data->get_goal = buff[22];
    data->if_arrived=buff[23];
    data->close_flag=buff[24];
    data->close_flag=buff[25];
    
    data->need_tunnel = buff[30];
    

    

    data->navi_vx = Vx.data;
    data->navi_vy = Vy.data;

    //      data->navi_vx=0;
    //      data->navi_vy=0;
    data->navi_yaw_diff = yaw.data * 57.3f;
    data->current_x = Sx.data;
    data->current_y = Sy.data;

    if (data->navi_yaw_diff > 180.0f)
    {
      data->navi_yaw_diff -= 360;
    }
    else if (data->navi_yaw_diff < -180.0f)
    {
      data->navi_yaw_diff += 360;
    }

    if (fabs(data->navi_yaw_diff) < 30)
    {
      data->navi_yaw_diff = 0;
    }
    
    navi_tx_count = 0;
    data->navigate_yaw_target = INS.YawTotalAngle + (data->navi_yaw_diff);
  
    data->if_control=(data->If_get_path!=0&&data->get_goal!=0);
  }
}

uint8_t last_point;
uint8_t clear_count;
USBD_StatusTypeDef IOIOL;
uint8_t data_test[4];


float transform_angle=0.0f;
float dist_x=0.0f;
float dist_y=0.0f;
float diff_yaw=0.0f;

float transform_x=0.0f;
float transform_y=0.0f;
void Navigation_Tx_Send(navigation_tx_t *data)
{
  transform_angle = receive_gimbal_data.current_transform_angle * 2 * 3.14 / 360;
  dist_x = -receive_gimbal_data.armor_dist * cos(transform_angle);
  dist_y = -receive_gimbal_data.armor_dist * sin(transform_angle);
  
  data->nav_cmd_id = navigation_nav_id;
  data->navi_set_x_pos=0;
  data->navi_set_y_pos=0;
  data->current_yaw = INS.Yaw;
  data->current_pitch = INS.Roll;
  
  if( receive_gimbal_data.vision_state == vision_frount)
  {
    data->enemy_pose.if_on_vision=0;  
  }
  else 
  {
    data->enemy_pose.if_on_vision=0;  
  }
  data->enemy_pose.enemy_pos_x = dist_x;
  data->enemy_pose.enemy_pos_y = dist_y;

  
  Navigate_Tx_buff[0] = CONST_HEAD0;
  Navigate_Tx_buff[1] = 0x01;
//  memcmp(Navigate_Tx_buff + 1, &data->nav_cmd_id,1);
  memcpy(Navigate_Tx_buff + 2, &data->navi_set_x_pos, 4);
  memcpy(Navigate_Tx_buff + 6, &data->navi_set_y_pos, 4);
  memcpy(Navigate_Tx_buff + 10, &data->current_yaw, 4);
  memcpy(Navigate_Tx_buff + 14, &data->current_pitch, 4);
  memcpy(Navigate_Tx_buff + 18, &data->enemy_pose.if_on_vision, 1);
  memcpy(Navigate_Tx_buff + 19, &data->enemy_pose.enemy_pos_x, 2);
  memcpy(Navigate_Tx_buff + 21, &data->enemy_pose.enemy_pos_y, 2);
  Navigate_Tx_buff[navigation_tx_len - 1] = CONST_END0;
  

  IOIOL = CDC_Transmit_FS(Navigate_Tx_buff, navigation_tx_len);
}

int ppppppp;
void Decision_Tx_Send(Decision_tx_t *data)
{
  
  data->decision_cmd_id = navigation_decision_id;
  memcpy(&data->sentry_decision_data, &decision.Judge_condition, sizeof(sentry_decision_data_t));
  data->game_remain_time=game_state.stage_remain_time;
  data->game_state = game_state.game_progress;
  
  data->projectile_allowance_17mm=0;
  data->current_hp = robot_status.current_HP;
  data->my_base_hp = game_robot_HP.we_base_HP;
  data->enemy_hero_x=0;
  data->enemy_hero_y=0;
  
  
  ppppppp++;
  
  Decisin_Tx_buff[0] = CONST_HEAD0;
  Decisin_Tx_buff[1] = 0x02;
//  memcmp(Decisin_Tx_buff + 1, &data->decision_cmd_id,1);
  memcmp(Decisin_Tx_buff + 2, &data->sentry_decision_data.sentry_decision_data_1,1);
  memcmp(Decisin_Tx_buff + 3, &data->sentry_decision_data.sentry_decision_data_2,1);
  memcmp(Decisin_Tx_buff + 4, &data->sentry_decision_data.sentry_decision_data_3,1);
  memcmp(Decisin_Tx_buff + 5, &data->sentry_decision_data.sentry_decision_data_4,1);
  memcmp(Decisin_Tx_buff + 6, &data->game_remain_time,2);
  memcmp(Decisin_Tx_buff + 8, &data->game_state,1);
  memcmp(Decisin_Tx_buff + 9, &data->projectile_allowance_17mm,2);
  memcmp(Decisin_Tx_buff + 11, &data->current_hp,2);
  memcmp(Decisin_Tx_buff + 13, &data->my_base_hp,2);
  memcmp(Decisin_Tx_buff + 15, &data->enemy_hero_x,2);
  memcmp(Decisin_Tx_buff + 17, &data->enemy_hero_y,2);

  Decisin_Tx_buff[decision_tx_len - 1] = CONST_END0;
//  memcpy(TX_Buff + 17, &data->sentry_decision_data.sentry_decision_data_1, 1);
//  memcpy(TX_Buff + 18, &data->sentry_decision_data.sentry_decision_data_2, 1);
//  memcpy(TX_Buff + 19, &data->sentry_decision_data.sentry_decision_data_3, 1);
//  memcpy(TX_Buff + 20, &data->sentry_decision_data.sentry_decision_data_4, 1);
//  memcpy(TX_Buff + 21, &data->game_remain_time, 2);
//  memcpy(TX_Buff + 23, &data->game_state, 1);
IOIOL = CDC_Transmit_FS(Decisin_Tx_buff, decision_tx_len);
//  
}

void Serial_Data_Handle()
{
}

