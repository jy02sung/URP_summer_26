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



////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------- Inverse Kinematics ---------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
// DLS(Damped Least Squares) 위치 IK. 좌우를 6D 스택 태스크로 동시에 푼다.
//   e = [tL - xL ; tR - xR]   (6x1, 위치 오차)
//   J = [JL(상위3행) ; JR(상위3행)]  (6 x DoF)
//   dq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
void DualArmControl::SolveIK_Position(pinocchio::Model& model, pinocchio::Data& data,
                                      pinocchio::FrameIndex l_EE, pinocchio::FrameIndex r_EE,
                                      const Vector3d& target_L, const Vector3d& target_R,
                                      const VectorXd& q_seed, VectorXd& q_out)
{
    const double lambda = 0.1;    // DLS 댐핑
    const double tol    = 1e-4;   // 수렴 허용 오차 [m]
    const int    maxIter= 300;    // 최대 반복
    const double step   = 1.0;    // 스텝 스케일 (= K·Δt 개념, 발산하면 줄이기)

    VectorXd q = q_seed;

    for (int iter = 0; iter < maxIter; ++iter)
    {
        // 현재 q로 자코비안 + FK
        pinocchio::computeJointJacobians(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        Vector3d xL = data.oMf[l_EE].translation();
        Vector3d xR = data.oMf[r_EE].translation();

        VectorXd e(6);
        e.head<3>() = target_L - xL;
        e.tail<3>() = target_R - xR;

        if (e.norm() < tol) break;

        pinocchio::Data::Matrix6x JL(6, model.nv); JL.setZero();
        pinocchio::Data::Matrix6x JR(6, model.nv); JR.setZero();
        pinocchio::getFrameJacobian(model, data, l_EE, pinocchio::LOCAL_WORLD_ALIGNED, JL);
        pinocchio::getFrameJacobian(model, data, r_EE, pinocchio::LOCAL_WORLD_ALIGNED, JR);

        MatrixXd J(6, model.nv);
        J.topRows<3>()    = JL.topRows<3>();   // 위치 3행만
        J.bottomRows<3>() = JR.topRows<3>();

        MatrixXd JJt = J * J.transpose() + (lambda*lambda) * MatrixXd::Identity(6,6);
        VectorXd dq  = J.transpose() * JJt.ldlt().solve(e);

        q += step * dq;

        // 관절 한계 클램핑
        for (int i = 0; i < model.nq; ++i)
            q(i) = std::min(std::max(q(i), model.lowerPositionLimit(i)),
                                           model.upperPositionLimit(i));
    }

    q_out = q;
}

////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------ Cartesian Line Trajectory -------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
// 시작 위치 → 목표 위치를 5차 시간 스케일링 s(t)∈[0,1]로 직선 보간.
// 좌우 동일한 시간 Tf를 쓰되, 더 긴 이동거리를 기준으로 Tf를 잡는다.
void DualArmControl::CartesianLineTrajectory(const Vector3d& startL, const Vector3d& goalL,
                                             const Vector3d& startR, const Vector3d& goalR,
                                             MatrixXd& pos_out, MatrixXd& vel_out, MatrixXd& acc_out)
{
    double v_des = 0.1;

    Vector3d vecL = goalL - startL;
    Vector3d vecR = goalR - startR;
    double distL = vecL.norm();
    double distR = vecR.norm();
    double maxDist = std::max(distL, distR);

    double Tf = maxDist / v_des;
    if (Tf < 1e-6) Tf = 1.0;

    int step = std::round(Tf / SAMPLING_TIME_TRAJ);
    if (step < 1) step = 1;
    pos_out.resize(step, 6);
    vel_out.resize(step, 6);
    acc_out.resize(step, 6);

    for (int j = 0; j < step; ++j)
    {
        double t = j * SAMPLING_TIME_TRAJ;
        double tau = t / Tf;

        double s   = 10*pow(tau,3) - 15*pow(tau,4) + 6*pow(tau,5);
        double ds  = 30*pow(tau,2) - 60*pow(tau,3) + 30*pow(tau,4);   // ds/dtau
        double dds = 60*tau - 180*pow(tau,2) + 120*pow(tau,3);        // d^2s/dtau^2

        double sdot  = ds  / Tf;       
        double sddot = dds / (Tf*Tf);   

        Vector3d pL = startL + s * vecL;
        Vector3d pR = startR + s * vecR;
        Vector3d vL = sdot * vecL;
        Vector3d vR = sdot * vecR;
        Vector3d aL = sddot * vecL;
        Vector3d aR = sddot * vecR;

        pos_out(j,0)=pL.x(); pos_out(j,1)=pL.y(); pos_out(j,2)=pL.z();
        pos_out(j,3)=pR.x(); pos_out(j,4)=pR.y(); pos_out(j,5)=pR.z();

        vel_out(j,0)=vL.x(); vel_out(j,1)=vL.y(); vel_out(j,2)=vL.z();
        vel_out(j,3)=vR.x(); vel_out(j,4)=vR.y(); vel_out(j,5)=vR.z();

        acc_out(j,0)=aL.x(); acc_out(j,1)=aL.y(); acc_out(j,2)=aL.z();
        acc_out(j,3)=aR.x(); acc_out(j,4)=aR.y(); acc_out(j,5)=aR.z();
    }
}

MatrixXd DualArmControl::DampedPinv(const MatrixXd& J, double lambda)
{
    MatrixXd JJt = J * J.transpose();
    MatrixXd I = MatrixXd::Identity(JJt.rows(), JJt.cols());
    return J.transpose() * (JJt + lambda*lambda*I).inverse();
}