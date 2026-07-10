#include "dual_arm_function.h"

DualArmControl::DualArmControl()
{
}

DualArmControl::~DualArmControl()
{
}


double DualArmControl::LowPassFilter(double &input, double &output_before, double cutoff_frequency)
{
	double alpha = 0.0;
	double output = 0.0;
	alpha = 1 / (1 + 2*M_PI*cutoff_frequency*SAMPLING_TIME);
	output = alpha * output_before + (1 - alpha)*input;
	return output;
}


////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------Trajectory Planning----------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void DualArmControl::JointTrajectoryTrapezoidal(double* q_ini, double* q_cmd, MatrixXd& q_out, MatrixXd& q_dot_out)
{
	double q_dot_des = 0.6; //rad/s
	double q_double_dot_des = 1; //rad/s^2

	double max_q_error = 0;
	for (int i = 0; i < DoF; i++){
		if(max_q_error < fabs(q_cmd[i] - q_ini[i]))
			max_q_error = fabs(q_cmd[i] - q_ini[i]);
	}

	double Tf = max_q_error / q_dot_des; // Whole time
	// double Tf = 1.5;

	double Tb; // blending time
	if (q_double_dot_des >= 4*max_q_error/pow(Tf,2)) {
		Tb = Tf/2 - sqrt(pow(q_double_dot_des*Tf,2) - 4*q_double_dot_des*max_q_error) / (2*q_double_dot_des);
	}
	else {
		Tb = Tf/2;
	}
	int lin_step = round((Tf - 2*Tb) / SAMPLING_TIME_TRAJ);
	int acc_step = round(Tb / SAMPLING_TIME_TRAJ);
	int step = lin_step + 2*acc_step + 1;
	q_out.resize(step, q_out.cols());
	q_dot_out.resize(step, q_dot_out.cols());

	for (int i = 0; i < DoF; i++)
	{
		double q_dot = (q_cmd[i] - q_ini[i])/(Tf-Tb);
		double q_double_dot = q_dot/Tb;
		for (int j = 0; j < step; j++)
		{
			if (j <= acc_step) {
				q_out(j, i) = q_ini[i] + 0.5*q_double_dot*pow(j*SAMPLING_TIME_TRAJ,2);
			}
			else if (j <= acc_step + lin_step) {
				q_out(j, i) = q_out(acc_step, i) + q_dot*((j-acc_step)*SAMPLING_TIME_TRAJ);
			}
			else {
				q_out(j, i) = -0.5*q_double_dot*pow(j*SAMPLING_TIME_TRAJ - Tf, 2) + q_cmd[i];
			}
			if (j > 0) {
				q_dot_out(j,i) = (q_out(j,i) - q_out(j-1,i))/SAMPLING_TIME_TRAJ;
			}
		}
	}
}

void DualArmControl::JointTrajectoryQuintic(double* q_ini, double* q_cmd, MatrixXd& q_out, MatrixXd& q_dot_out, MatrixXd& q_acc_out)
{
	double q_dot_des = 0.5;  // 원하는 각속도 (rad/s)

	// 모든 관절 중 최대 오차를 기준으로 전체 궤적 시간(Tf) 계산
	double max_q_error = 0;
	for (int i = 0; i < DoF; i++) {
	double error = fabs(q_cmd[i] - q_ini[i]);
	if (max_q_error < error)
	max_q_error = error;
	}
	double Tf = max_q_error / q_dot_des;  // 궤적 전체 시간

	// 샘플링 시간에 따른 총 step 수 계산
	int step = round(Tf / SAMPLING_TIME_TRAJ);
	q_out.resize(step, DoF);
	q_dot_out.resize(step, DoF);
	q_acc_out.resize(step, DoF);

	// 각 관절별로 Quintic Polynomial 계수를 계산하고, 위치, 속도, 가속도 생성
	// 경계 조건: q(0)=q0, q(Tf)=qf,  dot{q}(0)=dot{q}(Tf)=0, ddot{q}(0)=ddot{q}(Tf)=0
	// 그러면: c0 = q0, c1 = 0, c2 = 0,
	// c3 = 10*(qf - q0) / Tf^3, c4 = -15*(qf - q0) / Tf^4, c5 = 6*(qf - q0) / Tf^5.
	for (int i = 0; i < DoF; i++)
	{
	double q0 = q_ini[i];
	double qf = q_cmd[i];

	double c0 = q0;
	double c1 = 0.0;
	double c2 = 0.0;
	double c3 = 10.0 * (qf - q0) / pow(Tf, 3);
	double c4 = -15.0 * (qf - q0) / pow(Tf, 4);
	double c5 = 6.0 * (qf - q0) / pow(Tf, 5);

	for (int j = 0; j < step; j++)
	{
	double t = j * SAMPLING_TIME_TRAJ;
	// 위치: q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
	q_out(j, i) = c0 + c1*t + c2*t*t + c3*pow(t, 3) + c4*pow(t, 4) + c5*pow(t, 5);
	// 속도: q_dot(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4
	q_dot_out(j, i) = c1 + 2.0*c2*t + 3.0*c3*t*t + 4.0*c4*pow(t, 3) + 5.0*c5*pow(t, 4);
	// 가속도: q_double_dot(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
	q_acc_out(j, i) = 2.0*c2 + 6.0*c3*t + 12.0*c4*t*t + 20.0*c5*pow(t, 3);
	}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------Controller---------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void DualArmControl::PDController(double* target_q, double* current_q, double* target_q_dot, double* current_q_dot, double* PDtorque)
{
    double q_error[DoF] = {0, };
    double P_term[DoF] = {0, },
           D_term[DoF] = {0, };
    int torque_limit = 100;
    for (int i = 0; i < DoF; i++)
    {
        q_error[i] = target_q[i] - current_q[i];

        P_term[i] = Kp[i] * q_error[i];
        D_term[i] = Kd[i] * (target_q_dot[i] - current_q_dot[i]);

        PDtorque[i] = P_term[i] + D_term[i];

        if (PDtorque[i] > torque_limit)
        {
            PDtorque[i] = torque_limit;
        }
        else if (PDtorque[i] < -torque_limit)
        {
            PDtorque[i] = -torque_limit;
        }
    }
}