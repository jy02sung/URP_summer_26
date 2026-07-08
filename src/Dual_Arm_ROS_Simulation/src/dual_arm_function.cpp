#include "dual_arm_function.h"

DualArmControl::DualArmControl() {}
DualArmControl::~DualArmControl() {}

double DualArmControl::LowPassFilter(double &input, double &output_before, double cutoff_frequency)
{
	double alpha = 0.0;
	double output = 0.0;
	alpha = 1 / (1 + 2*M_PI*cutoff_frequency*SAMPLING_TIME);
	output = alpha * output_before + (1 - alpha)*input;
	return output;
}

// -----------------------------------Trajectory Planning----------------------------------//
void DualArmControl::JointTrajectoryTrapezoidal(double* q_ini, double* q_cmd, MatrixXd& q_out, MatrixXd& q_dot_out)
{
	double q_dot_des = 1.2; //rad/s
	double q_double_dot_des = 1; //rad/s^2

	double max_q_error = 0;
	for (int i = 0; i < DoF; i++){
		if(max_q_error < fabs(q_cmd[i] - q_ini[i]))
			max_q_error = fabs(q_cmd[i] - q_ini[i]);
	}

	double Tf = max_q_error / q_dot_des; 

	double Tb; 
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
	double q_dot_des = 0.5;  

	double max_q_error = 0;
	for (int i = 0; i < DoF; i++) {
	double error = fabs(q_cmd[i] - q_ini[i]);
	if (max_q_error < error)
	max_q_error = error;
	}
	double Tf = max_q_error / q_dot_des;  
    if (Tf < 0.1) Tf = 0.1; // 너무 짧은 시간 방지

	int step = round(Tf / SAMPLING_TIME_TRAJ);
	q_out.resize(step, DoF);
	q_dot_out.resize(step, DoF);
	q_acc_out.resize(step, DoF);

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
	q_out(j, i) = c0 + c1*t + c2*t*t + c3*pow(t, 3) + c4*pow(t, 4) + c5*pow(t, 5);
	q_dot_out(j, i) = c1 + 2.0*c2*t + 3.0*c3*t*t + 4.0*c4*pow(t, 3) + 5.0*c5*pow(t, 4);
	q_acc_out(j, i) = 2.0*c2 + 6.0*c3*t + 12.0*c4*t*t + 20.0*c5*pow(t, 3);
	}
	}
}

// 3차원 공간상의 직선 경로를 생성하는 함수 (Quintic 활용)
void DualArmControl::CartesianTrajectoryQuintic(const Vector3d& p_ini, const Vector3d& p_cmd, MatrixXd& p_out, MatrixXd& p_dot_out, MatrixXd& p_acc_out)
{
    double v_des = 0.15; // 손끝 속도 (m/s)

    double distance = (p_cmd - p_ini).norm();
    double Tf = distance / v_des;
    if (Tf < 0.1) Tf = 0.1;

    int step = round(Tf / SAMPLING_TIME_TRAJ);
    p_out.resize(step, 3);
    p_dot_out.resize(step, 3);
    p_acc_out.resize(step, 3);

    for (int i = 0; i < 3; i++)
    {
        double p0 = p_ini(i);
        double pf = p_cmd(i);

        double c0 = p0;
        double c1 = 0.0;
        double c2 = 0.0;
        double c3 = 10.0 * (pf - p0) / pow(Tf, 3);
        double c4 = -15.0 * (pf - p0) / pow(Tf, 4);
        double c5 = 6.0 * (pf - p0) / pow(Tf, 5);

        for (int j = 0; j < step; j++)
        {
            double t = j * SAMPLING_TIME_TRAJ;
            p_out(j, i) = c0 + c1*t + c2*t*t + c3*pow(t, 3) + c4*pow(t, 4) + c5*pow(t, 5);
            p_dot_out(j, i) = c1 + 2.0*c2*t + 3.0*c3*t*t + 4.0*c4*pow(t, 3) + 5.0*c5*pow(t, 4);
            p_acc_out(j, i) = 2.0*c2 + 6.0*c3*t + 12.0*c4*t*t + 20.0*c5*pow(t, 3);
        }
    }
}


// ---------------------------------------Controller---------------------------------------//
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

// =========================================================================================
// DLS 역기구학 솔버 구현부
// =========================================================================================
bool DualArmControl::SolveIK_DLS(pinocchio::Model& model, pinocchio::Data& data, const pinocchio::FrameIndex frame_id, const pinocchio::SE3& target_pose, Eigen::VectorXd& q_inout)
{
    const int max_iter = 1000;
    const double eps = 1e-4; 
    const double lambda = 0.1; 
    const double dt = 0.5; 

    Eigen::VectorXd q = q_inout;
    Eigen::MatrixXd J(6, model.nv);
    bool success = false;

    for (int it = 0; it < max_iter; ++it) {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        
        const pinocchio::SE3 iMd = data.oMf[frame_id].actInv(target_pose);
        Eigen::VectorXd err = pinocchio::log6(iMd).toVector(); 
        
        if (err.norm() < eps) {
            success = true;
            break;
        }

        J.setZero();
        pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL, J);
        
        Eigen::MatrixXd JJt = J * J.transpose();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        Eigen::VectorXd dq = J.transpose() * (JJt + lambda * lambda * I).inverse() * err;
        
        q = pinocchio::integrate(model, q, dq * dt);
    }
    
    q_inout = q;
    return success;
}