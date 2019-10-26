 /******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_IMU.c
 * 描述    ：姿态解算函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "Ano_Imu.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"
#include "Ano_DT.h"
//#include "ANO_RC.h"



/*参考坐标，定义为ANO坐标*

俯视，机头方向为x正方向
     +x
     |
 +y--|--
     |
		 
*/	
//磁力计的XY二维变换
void w2h_2d_trans(float w[VEC_XYZ],float ref_ax[VEC_XYZ],float h[VEC_XYZ])
{
	h[X] =  w[X] *  ref_ax[X]  + w[Y] *ref_ax[Y];
	h[Y] =  w[X] *(-ref_ax[Y]) + w[Y] *ref_ax[X];
	
}

void h2w_2d_trans(float h[VEC_XYZ],float ref_ax[VEC_XYZ],float w[VEC_XYZ])
{
	w[X] = h[X] *ref_ax[X] + h[Y] *(-ref_ax[Y]);
	w[Y] = h[X] *ref_ax[Y] + h[Y] *  ref_ax[X];
	
}

float mag_yaw_calculate(float dT,float mag_val[VEC_XYZ],float g_z_vec[VEC_XYZ],float h_mag_val[VEC_XYZ])//
{
//	float mag_h_norm;
//	float mag_2d_vec[VEC_XYZ];
		vec_3dh_transition(g_z_vec, mag_val, h_mag_val);
		
//		mag_h_norm = my_sqrt(my_pow(h_mag_val[X]) + my_pow(h_mag_val[Y]));
//		
//		mag_2d_vec[X] = safe_div(h_mag_val[X],mag_h_norm,0);
//		mag_2d_vec[Y] = safe_div(h_mag_val[Y],mag_h_norm,0);
	
//	return (fast_atan2(mag_2d_vec[Y], mag_2d_vec[X]) *57.3f) ;// 	
	return (fast_atan2(h_mag_val[Y], h_mag_val[X]) *57.3f) ;// 	
}
	


#define USE_MAG
#define USE_LENGTH_LIM
//imu最长的结构体定义变量赋初值
_imu_st imu_data =  {1,0,0,0,
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					{0,0,0},
					 0,0,0};

static float vec_err[VEC_XYZ];
static float vec_err_i[VEC_XYZ];
static float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;//q0q0,				
static float mag_yaw_err,mag_val_f[VEC_XYZ];					
static float imu_reset_val;		
static u16 reset_cnt;					 
_imu_state_st imu_state = {1,1,1,1,1,1,1,1};
float t[3][3],t_temp;
//float rpy_enr[VEC_XYZ],rpy_f1_a[VEC_XYZ],rpy_f1_b[VEC_XYZ],vec_err_f1[VEC_XYZ];
float imu_test[3];

//此函数在文件里面Ano_FlightDataCal.c被调用
//采样周期/imu状态结构体指针(实际传入的是&imu_state)/三向陀螺仪/三向加速度计/三向磁力计/imu结构体句柄指针(实际传入的是&imu_data)	
//IMU_update函数直接调用
//计算机体坐标下的运动加速度观测量。坐标系为北西天
//把测量值传给指针保存
// 加速度计的读数，单位化
// 载体坐标下的x方向向量，单位化
// 载体坐标下的y方向向量，单位化
// 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化
//水平面方向向量
// 计算载体坐标下的运动加速度。(与姿态解算无关)
	//计算世界坐标下的运动加速度。坐标系为北西天
   // 测量值与等效重力向量的叉积（计算向量误差）
//计算航向yaw误差
//误差积分
// 构造增量旋转（含融合纠正）
   // 计算姿态。
void IMU_update(float dT,_imu_state_st *state,float gyr[VEC_XYZ], s32 acc[VEC_XYZ],s16 mag_val[VEC_XYZ],_imu_st *imu)
{	
	static float kp_use = 0,ki_use = 0,mkp_use = 0;
	float acc_norm_l,acc_norm_l_recip,q_norm_l;		
	float acc_norm[VEC_XYZ];
	float d_angle[VEC_XYZ];
//直接调用
//		q0q0 = imu->w * imu->w;							
		q0q1 = imu->w * imu->x;
		q0q2 = imu->w * imu->y;
		q1q1 = imu->x * imu->x;
		q1q3 = imu->x * imu->z;
		q2q2 = imu->y * imu->y;
		q2q3 = imu->y * imu->z;
		q3q3 = imu->z * imu->z;
		q1q2 = imu->x * imu->y;
		q0q3 = imu->w * imu->z;
		if(state->obs_en)
		{	for(u8 i = 0;i<3;i++)//计算机体坐标下的运动加速度观测量。坐标系为北西天
			
			{
				s32 temp = 0;
				for(u8 j = 0;j<3;j++)
				{
					
					temp += imu->obs_acc_w[j] *t[j][i];//t[i][j] 转置为 t[j][i]
				}
				imu->obs_acc_a[i] = temp;
				
				imu->gra_acc[i] = acc[i] - imu->obs_acc_a[i];
			}
		}
		
		else//把测量值传给指针保存
		{
			for(u8 i = 0;i<3;i++)
			{			
				imu->gra_acc[i] = acc[i];      
			}
		}
		acc_norm_l_recip = my_sqrt_reciprocal(my_pow(imu->gra_acc[X]) + my_pow(imu->gra_acc[Y]) + my_pow(imu->gra_acc[Z]));
		acc_norm_l = safe_div(1,acc_norm_l_recip,0);	
		for(u8 i = 0;i<3;i++)// 加速度计的读数，单位化。
		{
			acc_norm[i] = imu->gra_acc[i] *acc_norm_l_recip;
		}
		
	// 载体坐标下的x方向向量，单位化。
    t[0][0] = imu->x_vec[X] = 1 - (2*q2q2 + 2*q3q3);
    t[0][1] = imu->x_vec[Y] = 2*q1q2 - 2*q0q3;
    t[0][2] = imu->x_vec[Z] = 2*q1q3 + 2*q0q2;		
	// 载体坐标下的y方向向量，单位化。
    t[1][0] = imu->y_vec[X] = 2*q1q2 + 2*q0q3;
    t[1][1] = imu->y_vec[Y] = 1 - (2*q1q1 + 2*q3q3);
    t[1][2] = imu->y_vec[Z] = 2*q2q3 - 2*q0q1;	
    // 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化。
    t[2][0] = imu->z_vec[X] = 2*q1q3 - 2*q0q2;
    t[2][1] = imu->z_vec[Y] = 2*q2q3 + 2*q0q1;
    t[2][2] = imu->z_vec[Z] = 1 - (2*q1q1 + 2*q2q2);		
	//水平面方向向量
	imu->hx_vec[X] = t[0][0];
	imu->hx_vec[Y] = t[1][0];
	// 计算载体坐标下的运动加速度。(与姿态解算无关)
		for(u8 i = 0;i<3;i++)
		{
			imu->a_acc[i] = (s32)(acc[i] - 981 *imu->z_vec[i]);
		}
		
    
		//计算世界坐标下的运动加速度。坐标系为北西天
		for(u8 i = 0;i<3;i++)
		{
			s32 temp = 0;
			for(u8 j = 0;j<3;j++)
			{
				
				temp += imu->a_acc[j] *t[i][j];
			}
			imu->w_acc[i] = temp;
		}
		
		w2h_2d_trans(imu->w_acc,imu_data.hx_vec,imu->h_acc);	
    // 测量值与等效重力向量的叉积（计算向量误差）。
    vec_err[X] =  (acc_norm[Y] * imu->z_vec[Z] - imu->z_vec[Y] * acc_norm[Z]);
    vec_err[Y] = -(acc_norm[X] * imu->z_vec[Z] - imu->z_vec[X] * acc_norm[Z]);
    vec_err[Z] = -(acc_norm[Y] * imu->z_vec[X] - imu->z_vec[Y] * acc_norm[X]);


#ifdef USE_MAG //计算航向yaw误差

		
		for(u8 i = 0;i<3;i++)
		{
			mag_val_f[i] = (float)mag_val[i];
		}	
			
		if(!(mag_val[X] ==0 && mag_val[Y] == 0 && mag_val[Z] == 0))
		{
			mag_yaw_err = mag_yaw_calculate(dT,mag_val_f,(imu->z_vec),(imu->h_mag)) - imu->yaw;
			mag_yaw_err = range_to_180deg(mag_yaw_err);	
		}
#endif
	
		for(u8 i = 0;i<3;i++)
		{

#ifdef USE_EST_DEADZONE	
			if(state->G_reset == 0 && state->obs_en == 0)
			{
				vec_err[i] = my_deadzone(vec_err[i],0,imu->gacc_deadzone[i]);
			}
#endif	

#ifdef USE_LENGTH_LIM			
			if(acc_norm_l>1060 || acc_norm_l<900)
			{
				vec_err[X] = vec_err[Y] = vec_err[Z] = 0;
			}
#endif
		//误差积分
		vec_err_i[i] +=  LIMIT(vec_err[i],-0.1f,0.1f) *dT *ki_use;

		
	// 构造增量旋转（含融合纠正）。	
	//    d_angle[X] = (gyr[X] + (vec_err[X]  + vec_err_i[X]) * kp_use - mag_yaw_err *imu->z_vec[X] *kmp_use *RAD_PER_DEG) * dT / 2 ;
	//    d_angle[Y] = (gyr[Y] + (vec_err[Y]  + vec_err_i[Y]) * kp_use - mag_yaw_err *imu->z_vec[Y] *kmp_use *RAD_PER_DEG) * dT / 2 ;
	//    d_angle[Z] = (gyr[Z] + (vec_err[Z]  + vec_err_i[Z]) * kp_use - mag_yaw_err *imu->z_vec[Z] *kmp_use *RAD_PER_DEG) * dT / 2 ;
			
			
#ifdef USE_MAG
			d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * kp_use - mag_yaw_err *imu->z_vec[i] *mkp_use *RAD_PER_DEG) * dT / 2 ;
#else
			d_angle[i] = (gyr[i] + (vec_err[i]  + vec_err_i[i]) * kp_use ) * dT / 2 ;
#endif
		}
    // 计算姿态。

    imu->w = imu->w            - imu->x*d_angle[X] - imu->y*d_angle[Y] - imu->z*d_angle[Z];
    imu->x = imu->w*d_angle[X] + imu->x            + imu->y*d_angle[Z] - imu->z*d_angle[Y];
    imu->y = imu->w*d_angle[Y] - imu->x*d_angle[Z] + imu->y            + imu->z*d_angle[X];
    imu->z = imu->w*d_angle[Z] + imu->x*d_angle[Y] - imu->y*d_angle[X] + imu->z;
		
		q_norm_l = my_sqrt_reciprocal(imu->w*imu->w + imu->x*imu->x + imu->y*imu->y + imu->z*imu->z);
    imu->w *= q_norm_l;
    imu->x *= q_norm_l;
    imu->y *= q_norm_l;
    imu->z *= q_norm_l;
		

  
  /////////////////////修正开关///////////////////////////
#ifdef USE_MAG
		if(state->M_fix_en==0)//磁力
		{
			mkp_use = 0;//不修正
			state->M_reset = 0;//罗盘修正不复位，清除复位标记
		}
		else
		{
			if(state->M_reset)//
			{
				//通过增量进行对准
				mkp_use = 5.0f;
				if(mag_yaw_err != 0 && ABS(mag_yaw_err)<2)
				{
					state->M_reset = 0;//误差小于2的时候，清除复位标记
				}
			}
			else
			{
				mkp_use = state->mkp; //正常修正
			}
		}
#endif
		
		if(state->G_fix_en==0)//重力方向修正
		{
			kp_use = 0;//不修正
		}
		else
		{
			if(state->G_reset == 0)//正常修正
			{			
				kp_use = state->gkp;
				ki_use = state->gki;
			}
			else//快速修正，通过增量进行对准
			{
				kp_use = 5.0f;
				ki_use = 0.0f;
//				imu->est_speed_w[X] = imu->est_speed_w[Y] = 0;
//				imu->est_acc_w[X] = imu->est_acc_w[Y] = 0;
//				imu->est_acc_h[X] = imu->est_acc_h[Y] =0;
				
				//计算静态误差是否缩小
//				imu_reset_val += (ABS(vec_err[X]) + ABS(vec_err[Y])) *1000 *dT;
//				imu_reset_val -= 0.01f;
				imu_reset_val = (ABS(vec_err[X]) + ABS(vec_err[Y]));
				
				imu_reset_val = LIMIT(imu_reset_val,0,1.0f);
				
				if((imu_reset_val < 0.05f) && (state->M_reset == 0))//计时
				{
					
					reset_cnt += 2;
					if(reset_cnt>400)
					{
						reset_cnt = 0;
						state->G_reset = 0;//已经对准，清除复位标记
					}
				}
				else
				{
					reset_cnt = 0;
				}
			}
		}
}

void calculate_RPY()
{	///////////////////////输出姿态角///////////////////////////////
	
		t_temp = LIMIT(1 - my_pow(t[2][0]),0,1);
		
		//imu_data.pit = asin(2*q1q3 - 2*q0q2)*57.30f;
	
		if(ABS(imu_data.z_vec[Z])>0.05f)//避免奇点的运算
		{
			imu_data.pit =  fast_atan2(t[2][0],my_sqrt(t_temp))*57.30f;
			imu_data.rol =  fast_atan2(t[2][1], t[2][2])*57.30f; 
			imu_data.yaw = -fast_atan2(t[1][0], t[0][0])*57.30f; 
		}
}


/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

