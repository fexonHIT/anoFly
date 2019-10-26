#include "Ano_LocCtrl.h"
#include "Drv_Gps.h"
#include "Ano_Imu.h"
#include "Ano_FlightCtrl.h"
#include "Ano_OF.h"
#include "Ano_Parameter.h"
//λ���ٶȻ����Ʋ���
_PID_arg_st loc_arg_1[2] ; 
//λ���ٶȻ���������
_PID_val_st loc_val_1[2] ; 
//λ���ٶȻ��������Ʋ���
_PID_arg_st loc_arg_1_fix[2] ; 
//λ���ٶȻ�������������
_PID_val_st loc_val_1_fix[2] ; 
/*�ǶȻ�PID������ʼ��*/
void Loc_1level_PID_Init()
{
	if(switchs.gps_on)
	{
		loc_arg_1[X].kp = Ano_Parame.set.pid_gps_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = Ano_Parame.set.pid_gps_loc_1level[KI]  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = Ano_Parame.set.pid_gps_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;	
		loc_arg_1[Y] = loc_arg_1[X];

	}
	else if(switchs.of_flow_on)
	{
		loc_arg_1[X].kp = Ano_Parame.set.pid_loc_1level[KP];//0.22f  ;
		loc_arg_1[X].ki = 0.0f  ;
		loc_arg_1[X].kd_ex = 0.00f ;
		loc_arg_1[X].kd_fb = Ano_Parame.set.pid_loc_1level[KD];
		loc_arg_1[X].k_ff = 0.02f;
		
		loc_arg_1[Y] = loc_arg_1[X];
	//fix	
		loc_arg_1_fix[X].kp = 0.0f  ;
		loc_arg_1_fix[X].ki = Ano_Parame.set.pid_loc_1level[KI] ;
		loc_arg_1_fix[X].kd_ex = 0.00f;
		loc_arg_1_fix[X].kd_fb = 0.00f;
		loc_arg_1_fix[X].k_ff = 0.0f;
		
		loc_arg_1_fix[Y] = loc_arg_1_fix[X];	
	}
	else
	{
		//null
	}
	

}

_loc_ctrl_st loc_ctrl_1;//Ϊ����PID�����ṩ����ֵ�ͷ���ֵ
static float fb_speed_fix[2];
static u8 mode_f[2];
/*λ���ٶȻ�*/
void Loc_1level_Ctrl(u16 dT_ms,s16 *CH_N)
{ static float loc_hand_exp_vel[2]={0};
	static unsigned short waite_gps_loc_cnt = 0;
	float ne_pos_control[2];
	unsigned char vel_diff = 5;
	float loc_gps_h_out[2];
	float loc_gps_w_out[2];
	if(switchs.of_flow_on && (!switchs.gps_on))//����򿪹���������
	{
		mode_f[1] = 1;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		loc_ctrl_1.exp[X] = fs.speed_set_h[X];
		loc_ctrl_1.exp[Y] = fs.speed_set_h[Y];


		loc_ctrl_1.fb[X] = OF_DX2;
		loc_ctrl_1.fb[Y] = OF_DY2;
		
		fb_speed_fix[0] = OF_DX2FIX;
		fb_speed_fix[1] = OF_DY2FIX;
		
		for(u8 i =0;i<2;i++)//��PID�������ͷ���ֵ���ù���ֵ
		{
			
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										loc_ctrl_1.fb[i] ,			//����ֵ����
										&loc_arg_1[i], //PID�����ṹ��
										&loc_val_1[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;	
			
			//fix
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										fb_speed_fix[i] ,			//����ֵ����
										&loc_arg_1_fix[i], //PID�����ṹ��
										&loc_val_1_fix[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;	
			
			loc_ctrl_1.out[i] = loc_val_1[i].out + loc_val_1_fix[i].out;		
		}		


	}
	else if (switchs.gps_on)//���GPS��
	{
		mode_f[1] = 2;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		for(u8 j = 0; j < 2; j++)
		{
			if (fs.speed_set_h[j] != 0)				//�ж��Ƿ񶯿���ҡ��
			{
				if (ABS(loc_hand_exp_vel[j]) < ABS(fs.speed_set_h[j]))	//�ж��ٶ��Ƿ�ﵽ�����ٶ�
				{
					if (loc_hand_exp_vel[j]*fs.speed_set_h[j] < 0)	//�ж�ҡ�˷�������������Ƿ���ͬ
					{
						if (fs.speed_set_h[j] > 0)					//�ж�ҡ�˷���
						{
							fs.speed_set_h[j] = MAX_SPEED;			//��������ٶ�
						}
						else 
						{
							fs.speed_set_h[j] = -MAX_SPEED;
						}
					}
					loc_hand_exp_vel[j] += 0.5f*dT_ms*fs.speed_set_h[j]/MAX_SPEED;		//���������ٶ�
				}
				else												
				{
					if (loc_hand_exp_vel[j] > 0)					//�ظ���Ӧ�������ٶ���Ӧ
					{
						loc_hand_exp_vel[j] -= vel_diff;
					}
					else
					{
						loc_hand_exp_vel[j] += vel_diff;
					}
				}
			}
			else
			{
				if (loc_hand_exp_vel[j] > vel_diff)					//�ظ���Ӧ�������ٶ���Ӧ
				{
					loc_hand_exp_vel[j] -= vel_diff;
				}
				else if (loc_hand_exp_vel[j] <= -vel_diff)
				{
					loc_hand_exp_vel[j] += vel_diff;
				}
				else
				{
					loc_hand_exp_vel[j] = 0;
				}
			}
		}
		if (loc_hand_exp_vel[X] || loc_hand_exp_vel[Y])				//�ж��Ƿ����ֶ������ٶ�
		{
			ne_pos_control[0] = 0;									//λ�ÿ���������
			ne_pos_control[1] = 0;
			waite_gps_loc_cnt = 50;									//�����ٶȹ���֮��ȴ�500ms
		}
		else
		{
			if (waite_gps_loc_cnt > 0)
			{
				ne_pos_control[0] = 0;					//λ�ÿ���������
				ne_pos_control[1] = 0;
				waite_gps_loc_cnt--;
				if (waite_gps_loc_cnt == 0)				//����λ���Ѿ��ȶ� ��¼����λ���Լ���λ�ý��п���
				{
					Gps_information.hope_latitude = Gps_information.latitude_offset;		//����λ�õ��ڵ�ǰλ��
					Gps_information.hope_longitude = Gps_information.longitude_offset;
				}
			}
			else
			{
				Gps_information.hope_latitude_err = Gps_information.hope_latitude - Gps_information.latitude_offset;		//γ�����
				Gps_information.hope_longitude_err = Gps_information.hope_longitude - Gps_information.longitude_offset;		//�������
				length_limit(&(Gps_information.hope_latitude_err), &(Gps_information.hope_longitude_err), MAX_SPEED*1.2f, ne_pos_control);	//����ģ������
			}
		}
		
		
		loc_ctrl_1.exp[X] =  ne_pos_control[0]*Ano_Parame.set.pid_gps_loc_2level[KP] + loc_hand_exp_vel[X]*imu_data.hx_vec[0] - loc_hand_exp_vel[Y]*imu_data.hx_vec[1];		//�����ٶȣ���������ת������������NED��
		loc_ctrl_1.exp[Y] = -ne_pos_control[1]*Ano_Parame.set.pid_gps_loc_2level[KP] + loc_hand_exp_vel[X]*imu_data.hx_vec[1] + loc_hand_exp_vel[Y]*imu_data.hx_vec[0];		

		loc_ctrl_1.fb[X] =  (Gps_information.last_N_vel) + (wcx_acc_use*980/4096);			//�ٶȷ���+���ٶ���ǰ
		loc_ctrl_1.fb[Y] = -(Gps_information.last_E_vel) + (wcy_acc_use*980/4096);
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //���ڣ���λ���룩
										loc_ctrl_1.exp[i] ,				//ǰ��ֵ
										loc_ctrl_1.exp[i] ,				//����ֵ���趨ֵ��
										loc_ctrl_1.fb[i] ,			//����ֵ����
										&loc_arg_1[i], //PID�����ṹ��
										&loc_val_1[i],	//PID���ݽṹ��
										50,//��������޷�
										10 *flag.taking_off			//integration limit�������޷�
										 )	;		

			if (!flag.taking_off)
			{
				loc_val_1[i].err_i = 0;
			}				
		}
		loc_gps_w_out[0] = loc_val_1[0].out;
		loc_gps_w_out[1] = loc_val_1[1].out;
		w2h_2d_trans(loc_gps_w_out, imu_data.hx_vec, loc_gps_h_out);	//�������꣨NED�����ƽ��ת��������������
		loc_ctrl_1.out[X] = loc_gps_h_out[0];
		loc_ctrl_1.out[Y] = loc_gps_h_out[1];
	}
	else
	{
		mode_f[1] = 3;
		if(mode_f[1] != mode_f[0])
		{
			Loc_1level_PID_Init();
			mode_f[0] = mode_f[1];
		}
		////
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[X] ;
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[Y] ;
	}
}_loc_ctrl_st loc_ctrl_2;

