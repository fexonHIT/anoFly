#include "Ano_Imu.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_spl06.h"
#include "Ano_AltCtrl.h" //�߶ȿ���
#include "Ano_MotionCal.h"  //�˶�����
#include "Ano_FlightCtrl.h"  //���п���
#include "Ano_MotorCtrl.h"     //�������
#include "Ano_AttCtrl.h"     //��̬����
#include "Ano_LocCtrl.h"       //λ�ÿ���
//�Զ�����ٶ�
static s16 auto_taking_off_speed;
#define AUTO_TAKE_OFF_KP 2.0f
////extern _filter_1_st wz_spe_f1;
//�Զ���ɽ�������
void Auto_Take_Off_Land_Task(u8 dT_ms)
{
	static u16 take_off_ok_cnt;
	//ִ��һ���������
	one_key_take_off_task(dT_ms);
	if(flag.fly_ready)   //����ѽ�׼������
	{
		if(flag.taking_off)  //������������
		{	
			if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)   //�����δ���
			{
				flag.auto_take_off_land = AUTO_TAKE_OFF;		
				//���
			}
		}
	}
	else
	{ auto_taking_off_speed = 0;	
		flag.auto_take_off_land = AUTO_TAKE_OFF_NULL;	
	}
	if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
	{ //������
		take_off_ok_cnt += dT_ms;//�ӳٵȴ�2500
		auto_taking_off_speed = AUTO_TAKE_OFF_KP *(Ano_Parame.set.auto_take_off_height - wcz_hei_fus.out);
		auto_taking_off_speed = LIMIT(auto_taking_off_speed,0,150);
		//�õ��ٶȲ��ҽ����޷�
		if(take_off_ok_cnt>=5000 || (Ano_Parame.set.auto_take_off_height - wcz_hei_fus.out <5))//(auto_ref_height>AUTO_TAKE_OFF_HEIGHT)
		{//���cnt>5000����˵�����߶Ⱥ�ʵ�ʸ߶����������Χ��
			flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
			//�Զ�������
			}
		

		if(take_off_ok_cnt >1000 && ABS(fs.speed_set_h_norm[Z])>0.1f)// һ���Ѿ�taking_off
		{
			flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
		}
	
	}
	else 
	{
		take_off_ok_cnt = 0;
		
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF_FINISH)
		{
			auto_taking_off_speed = 0;
			
		}
		
	}


////////////
	
	if(flag.auto_take_off_land == AUTO_LAND)
	{
		auto_taking_off_speed = -60;

	}
}


_PID_arg_st alt_arg_2;
_PID_val_st alt_val_2;

/*�߶Ȼ�PID������ʼ��*/
void Alt_2level_PID_Init()
{
	alt_arg_2.kp = Ano_Parame.set.pid_alt_2level[KP];
	alt_arg_2.ki = Ano_Parame.set.pid_alt_2level[KI];
	alt_arg_2.kd_ex = 0.00f;
	alt_arg_2.kd_fb = Ano_Parame.set.pid_alt_2level[KD];
	alt_arg_2.k_ff = 0.0f;

}




void Alt_2level_Ctrl(float dT_s)
{
	Auto_Take_Off_Land_Task(1000*dT_s);
	
	fs.alt_ctrl_speed_set = fs.speed_set_h[Z] + auto_taking_off_speed;
	
	loc_ctrl_2.fb[Z] = wcz_hei_fus.out;/////////////
	
	if(fs.alt_ctrl_speed_set != 0)
	{
		flag.ct_alt_hold = 0;
	}
	else
	{
		if(ABS(loc_ctrl_1.exp[Z] - loc_ctrl_1.fb[Z])<20)
		{
			flag.ct_alt_hold = 1;
		}
	}

	if(flag.taking_off == 1)
	{
		if(flag.ct_alt_hold == 1)
		{
			PID_calculate( dT_s,            //���ڣ���λ���룩
						0,				//ǰ��ֵ
						loc_ctrl_2.exp[Z],				//����ֵ���趨ֵ��
						loc_ctrl_2.fb[Z],			//����ֵ����
						&alt_arg_2, //PID�����ṹ��
						&alt_val_2,	//PID���ݽṹ��
						100,//��������޷�
						0			//integration limit�������޷�									
						 );
		}
		else
		{
			loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z] + alt_val_2.err;
		}
	}
	else
	{
		loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z];
		alt_val_2.out = 0;
		
	}
	
	alt_val_2.out  = LIMIT(alt_val_2.out,-150,150);
}



_PID_arg_st alt_arg_1;
_PID_val_st alt_val_1;

/*�߶��ٶȻ�PID������ʼ��*/
void Alt_1level_PID_Init()
{
	alt_arg_1.kp = Ano_Parame.set.pid_alt_1level[KP];
	alt_arg_1.ki = Ano_Parame.set.pid_alt_1level[KI];
	alt_arg_1.kd_ex = 0.00f;
	alt_arg_1.kd_fb = Ano_Parame.set.pid_alt_1level[KD];
	alt_arg_1.k_ff = 0.0f;

}

//static u8 thr_start_ok;
static float err_i_comp;
void Alt_1level_Ctrl(float dT_s)
{
	u8 out_en;
	out_en = (flag.taking_off != 0) ? 1 : 0;
	
	flag.thr_mode = THR_AUTO;//THR_MANUAL;
	
	loc_ctrl_1.exp[Z] = fs.alt_ctrl_speed_set + alt_val_2.out;
	
	if(0) //(flag.thr_mode == THR_MANUAL)
	{
		loc_ctrl_1.fb[Z] = 0;
	}
	else
	{
		loc_ctrl_1.fb[Z] = wcz_spe_fus.out;
	}
	
	PID_calculate( dT_s,            //���ڣ���λ���룩
					0,				//ǰ��ֵ
					loc_ctrl_1.exp[Z],				//����ֵ���趨ֵ��
					loc_ctrl_1.fb[Z] ,			//����ֵ����
					&alt_arg_1, //PID�����ṹ��
					&alt_val_1,	//PID���ݽṹ��
					100,//��������޷�
					(THR_INTE_LIM *10 - err_i_comp)*out_en			//integration limit�������޷�									
					 );
	
	if(flag.taking_off == 1)
	{
		LPF_1_(1.0f,dT_s,THR_START *10,err_i_comp);//err_i_comp = THR_START *10;			
	}
	else
	{
		err_i_comp = 0;
	}
	
	alt_val_1.out = LIMIT(alt_val_1.out,-err_i_comp,MAX_THR *10);
	
	loc_ctrl_1.out[Z] = out_en *FINAL_P *(alt_val_1.out + err_i_comp - 0.2f *imu_data.w_acc[Z]);
	
	mc.ct_val_thr = loc_ctrl_1.out[Z];
}

