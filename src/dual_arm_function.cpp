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

	// 관절 인덱스 레이아웃: 0=waist,1=head_yaw,2=head_pitch,3~8=왼팔,9~14=오른팔
	// 왼팔/오른팔 Tf를 각자의 최대 오차로 독립 계산 -> 변위가 작은 팔이 큰 팔의 Tf에 끌려가서
	// 초반 속도가 지나치게 작아지는(=늦게 움직이는 것처럼 보이는) 문제를 제거한다.
	// waist/head는 팔이 아니므로 셋 중 가장 긴 Tf(Tf_max)에 맞춘다.
	double max_error_left = 0, max_error_right = 0, max_error_wh = 0;
	for (int i = 3; i <= 8; i++) {
		double error = fabs(q_cmd[i] - q_ini[i]);
		if (max_error_left < error) max_error_left = error;
	}
	for (int i = 9; i <= 14; i++) {
		double error = fabs(q_cmd[i] - q_ini[i]);
		if (max_error_right < error) max_error_right = error;
	}
	// waist/head(0~2) 자체의 변위도 Tf 계산에 반영한다. 이게 빠지면 팔이 하나도 안 움직이는
	// head-only 명령에서 Tf_max=0 -> no_motion 처리되어 head/waist 궤적이 통째로 무시되는
	// 버그가 있었다 (Head 자동 스캔 진단 중 실측으로 확인: head_pitch 단독 명령이 전혀 반영되지 않음).
	for (int i = 0; i <= 2; i++) {
		double error = fabs(q_cmd[i] - q_ini[i]);
		if (max_error_wh < error) max_error_wh = error;
	}

	double Tf_left  = max_error_left  / q_dot_des;
	double Tf_right = max_error_right / q_dot_des;
	double Tf_wh    = max_error_wh    / q_dot_des;
	double Tf_max   = std::max({Tf_left, Tf_right, Tf_wh});   // waist/head는 셋 중 가장 긴 Tf에 맞춤

	// 관절별 Tf 배열 (재생은 전부 t=0에서 동시에 시작, 각자 자신의 Tf에 도달하면 그 자리에서 정지 유지)
	double Tf[DoF];
	Tf[0] = Tf_max; Tf[1] = Tf_max; Tf[2] = Tf_max;
	for (int i = 3; i <= 8;  i++) Tf[i] = Tf_left;
	for (int i = 9; i <= 14; i++) Tf[i] = Tf_right;

	// 전체 재생 구간(step)은 가장 긴 Tf(=Tf_max) 기준. 이보다 Tf가 짧은 관절은 도달 후 목표값 유지.
	int step = round(Tf_max / SAMPLING_TIME_TRAJ);
	if (step < 1) step = 1;
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
		double Tf_i = Tf[i];

		bool no_motion = (Tf_i < 1e-9);  // 해당 관절(그룹)의 목표 변위가 사실상 0인 경우

		double c0 = q0;
		double c1 = 0.0;
		double c2 = 0.0;
		double c3 = no_motion ? 0.0 : 10.0 * (qf - q0) / pow(Tf_i, 3);
		double c4 = no_motion ? 0.0 : -15.0 * (qf - q0) / pow(Tf_i, 4);
		double c5 = no_motion ? 0.0 : 6.0 * (qf - q0) / pow(Tf_i, 5);

		for (int j = 0; j < step; j++)
		{
			double t = j * SAMPLING_TIME_TRAJ;

			if (t >= Tf_i) {
				// 자신의 Tf에 먼저 도달한 관절(waist/head보다 짧은 팔)은 목표 자세에서 정지 유지
				q_out(j, i) = qf;
				q_dot_out(j, i) = 0.0;
				q_acc_out(j, i) = 0.0;
				continue;
			}

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
//
// 2026-07-17: null-space q_pref(elbow-down + shoulder-roll 바깥벌림) 투영 추가 - jys 브랜치에서
// 검증한 로직을 이 브랜치의 coupled(양팔 스택) 구조에 맞게 이식했다. jys는 waist/head를 고정하고
// 양팔을 독립적으로 푸는 구조라, waist를 null-space에 추가로 포함시키려 하니 왼팔이 원하는 waist
// 회전 방향과 오른팔이 원하는 방향이 반대로 나와(오프라인 검증: 왼팔 -0.029 vs 오른팔 +0.029)
// 평균을 내면 서로 상쇄되는 문제가 있었다. 이 브랜치는 애초부터 waist를 포함해 양팔을 하나의 6D
// 스택 태스크로 동시에 푸는 구조라 그 충돌 자체가 없다 - waist가 공유 변수가 아니라 태스크에
// 자연스럽게 녹아있는 하나의 컬럼이기 때문. pinocchio로 재검증한 결과 실질 활성 관절 9개
// (Waist+양팔 4관절씩, Head는 별도 분기라 애초 관여 안 함)로 6D 위치 태스크를 풀므로 rank=6이
// 항상 유지되고(300회 랜덤자세 전부 동일, 이 세션 초반에 이미 확인) null-space 여유가 3으로
// jys 최선의 경우(1)보다 훨씬 넉넉하다.
void DualArmControl::SolveIK_Position(pinocchio::Model& model, pinocchio::Data& data,
                                      pinocchio::FrameIndex l_EE, pinocchio::FrameIndex r_EE,
                                      const Vector3d& target_L, const Vector3d& target_R,
                                      const VectorXd& q_seed, VectorXd& q_out)
{
    const double lambda = 0.1;    // DLS 댐핑
    const double tol    = 1e-4;   // 수렴 허용 오차 [m]
    const int    maxIter= 300;    // 최대 반복
    const double step   = 1.0;    // 스텝 스케일 (= K·Δt 개념, 발산하면 줄이기)
    // null-space 선호 자세 게인. jys 브랜치 원값(0.35)을 이 coupled 구조(9개 활성 관절,
    // null-space 여유 3)에 그대로 넣어보니 위치 수렴 잔차가 2.1cm까지 남았다(오프라인
    // pinocchio 검증, 감쇠 의사역행렬이 완벽한 직교투영이 아니라 태스크 방향으로 약간
    // 새는 게 원인). 0.02~0.35 스윕 결과 관절 마진(L_roll/R_roll)은 게인과 거의 무관하게
    // 항상 0.16~0.34rad로 넉넉히 유지됐다 - 이 구조는 애초 여유가 커서 null-space가
    // 세게 안 밀어도 붕괴하지 않는다. 잔차를 줄이면서도 선호가 유의미하게 반영되는
    // 절충값으로 0.1을 선택(잔차 6.5mm, 여전히 elbow는 -0.95~-1.19로 아래쪽 유지).
    const double posture_gain = 0.1;

    VectorXd q = q_seed;

    // elbow-down + shoulder-roll 바깥벌림 선호 자세(q_pref). 순서는 DoF 배열과 동일:
    // 0:Waist 1:Head_yaw 2:Head_pitch 3~8:L_arm 9~14:R_arm. Wrist preference는 0 rad다.
    // Waist/Head는
    // 0.0(중립) - Head는 애초 Jacobian에 관여 안 하므로 아래에서 null-space 기여분을 명시적으로
    // 0으로 마스킹한다(관여 안 하는데도 (I-J⁺J)의 대각항이 1이라 그대로 두면 Head가 원치 않게
    // 끌려간다).
    VectorXd q_pref = VectorXd::Zero(DoF);
    q_pref(3) = 0.80;  q_pref(4) =  0.55; q_pref(5) = -0.20; q_pref(6)  = -0.45;  // L arm
    q_pref(9) = 0.80; q_pref(10) = -0.55; q_pref(11) = 0.20; q_pref(12) = -0.45;  // R arm

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
        MatrixXd JJt_inv = JJt.ldlt().solve(MatrixXd::Identity(6,6));
        MatrixXd Jpinv = J.transpose() * JJt_inv;   // DoF x 6

        VectorXd dq = Jpinv * e;

        // Null-space 투영: (I - J⁺J)로 태스크에 영향 없는 방향만 골라 q_pref를 soft하게
        // 반영한다. Head_yaw/Head_pitch는 애초 J의 해당 컬럼이 0이라 (I-J⁺J)의 그 자리
        // 대각항이 1로 남는데, q_pref-q 값을 명시적으로 0으로 마스킹해서 Head가 이 항으로
        // 끌려가지 않게 막는다(다른 관절 행에는 영향 없음 - Head 컬럼이 J에서 이미 0이라
        // N의 비대각 결합도 전부 0).
        MatrixXd N = MatrixXd::Identity(model.nv, model.nv) - Jpinv * J;
        VectorXd postureErr = posture_gain * (q_pref - q);
        postureErr(1) = 0.0;  // Head_yaw
        postureErr(2) = 0.0;  // Head_pitch
        dq += N * postureErr;

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
                                             MatrixXd& pos_out, MatrixXd& vel_out, MatrixXd& acc_out,
                                             double v_des)
{
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
