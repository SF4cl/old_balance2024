#include "chassisLtask.h"
#include "fdcan.h"
#include "cmsis_os.h"

extern DMA_HandleTypeDef hdma_usart1_tx;

extern chassis_t chassis_move;
extern INS_t INS;
// extern vmc_t vmc_R;

float LQR_K_L[12] = {
    -15.7725, -1.4432, -2.8507, -2.8933, 9.5062, 2.1972,
    18.0773, 2.5909, 9.1851, 8.3496, 20.4557, 2.3536};

extern float Poly_Coefficient[12][4];

vmc_t vmc_L;

PidTypeDef LegL_Pid;

first_order_filter_type_t d_phi1_l_filter;
first_order_filter_type_t d_phi4_l_filter;
first_order_filter_type_t pitch_gyro_l_filter;

float d_phi1_l_filter_para[1] = {0.05};
float d_phi4_l_filter_para[1] = {0.05};
float pitch_gyro_l_filter_para[1] = {0.01};

float crtL = 35.0f;
float Ltest = 0.0f;

void chassisLtask(void)
{
  while (INS.ins_flag == 0)
  {
    osDelay(1);
  }

  chassisL_init(&chassis_move, &vmc_L, &LegL_Pid);

  while (1)
  {
    chassisL_update(&chassis_move, &vmc_L, &INS);

    chassisL_cal(&chassis_move, &vmc_L, &INS, &LegL_Pid, LQR_K_L);

    if (chassis_move.start_flag == 1)
    {
			LFMotor_T(0.0f);
			osDelay(1);
			LBMotor_T(0.0f);
			
			//HT04电机 力矩大于零角度反馈值增大
//                   canSend_comd(LFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

//                   canSend_comd(LBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    //  canSend_comd(LFmotor, 0.0f, 0.0f, 0.0f, 0.0f, -vmc_L.T_set[1] / 6.0f);

    //  canSend_comd(LBmotor, 0.0f, 0.0f, 0.0f, 0.0f, -vmc_L.T_set[0] / 6.0f);

//      wheel_Lctrl(chassis_move.wheel_motor[0].T_wheel / 6.33f);
      						// wheel_Lctrl(0.0f);
      //					wheel_Lctrl(0.05f);
//      					wheel_Lctrl(Ltest);
      osDelay(2);
    }
    else if (chassis_move.start_flag == 0)
    {
			
			LFMotor_T(0.0f);
			osDelay(1);
			LBMotor_T(0.0f);
      // canSend_comd(LFmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      // canSend_comd(LBmotor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      // wheel_Lctrl(0.0f);
      osDelay(2);
    }
  }
}

void chassisL_init(chassis_t *chassis, vmc_t *vmc, PidTypeDef *legl)
{
  const static float legl_pid[3] = {LEG_LEFT_PID_KP, LEG_LEFT_PID_KI, LEG_LEFT_PID_KD};

  // joint_motor_set_mode(10);

//！todo：需将关节电机编码器值置零

  VMC_init(vmc);

  PID_init(legl, PID_POSITION, legl_pid, LEG_LEFT_PID_MAX_OUT, LEG_LEFT_PID_MAX_IOUT);

  // 滤波器初始化
  first_order_filter_init(&d_phi1_l_filter, 0.002, d_phi1_l_filter_para);
  first_order_filter_init(&d_phi4_l_filter, 0.002, d_phi4_l_filter_para);
  first_order_filter_init(&pitch_gyro_l_filter, 0.002, pitch_gyro_l_filter_para);
}

void chassisL_update(chassis_t *chassis, vmc_t *vmc, INS_t *ins)
{
  chassis->joint_motor[0].para.pos = LFMotor_Recv.Pos/9.1f;
  chassis->joint_motor[0].para.vel = LFMotor_Recv.W/9.1f;
  chassis->joint_motor[1].para.pos = LBMotor_Recv.Pos/9.1f;
  chassis->joint_motor[1].para.vel = LBMotor_Recv.W/9.1f;

//！需根据实际修改得到模型中的phi1和phi4
  vmc->phi4 = -chassis->joint_motor[0].para.pos + pi / 2.0f - 1.9823f;
  vmc->phi1 = -chassis->joint_motor[1].para.pos + pi / 2.0f + 1.9823f;

  vmc->d_phi1 = -chassis->joint_motor[1].para.vel;
  vmc->d_phi4 = -chassis->joint_motor[0].para.vel;

  chassis->pitchL = -ins->Pitch + 0.021f; //修正pitch值
  chassis->pitchGyroL = -ins->Gyro[1];

  first_order_filter_cali(&d_phi1_l_filter, vmc->d_phi1);
  first_order_filter_cali(&d_phi4_l_filter, vmc->d_phi4);
  first_order_filter_cali(&pitch_gyro_l_filter, chassis->pitchGyroL);

  // 把滤波后的值赋给vmc
  vmc->d_phi1 = d_phi1_l_filter.out;
  vmc->d_phi4 = d_phi4_l_filter.out;
  chassis->pitchGyroL = pitch_gyro_l_filter.out;
  // todo: 倒地检测
}

void chassisL_cal(chassis_t *chassis, vmc_t *vmc, INS_t *ins, PidTypeDef *legl, float *K)
{
  VMC_L_cal1(vmc, chassis->pitchL, chassis->pitchGyroL);

  for (int i = 0; i < 12; i++)
  {
    K[i] = LQR_K_cal(&Poly_Coefficient[i][0], vmc->L0);
  }

//  chassis->x_set = 0.0f;//x_set 在不断累加，暂时先强制置零
  chassis->wheel_motor[0].T_wheel = -(K[0] * vmc->theta + K[1] * vmc->d_theta + K[2] * (chassis->x-chassis->x_set) + K[3] * chassis->v + K[4] * chassis->pitchR  + K[5] * chassis->pitchGyroR);
  vmc->Tp = - (K[6] * vmc->theta + K[7] * vmc->d_theta+ K[8] * (chassis->x - chassis->x_set) + K[9] * chassis->v + K[10] * chassis->pitchR + K[11] * chassis->pitchGyroR);
  // chassis->wheel_motor[0].T_wheel = chassis->wheel_motor[1].T_wheel - chassis->T_turn;
  //! 防劈叉
  // vmc->Tp = vmc->Tp - chassis->Tp_2legs;

  vmc->F0 = crtL + PID_Calc(legl, vmc->L0, chassis->leg_set);

  limit(&chassis->wheel_motor[0].T_wheel, -5.0f, 5.0f);
  limit(&vmc->F0, -150.0f, 150.0f);

  VMC_cal2(vmc);

  limit(&vmc->T_set[0], -15.0f, 15.0f);
  limit(&vmc->T_set[1], -15.0f, 15.0f);
}
