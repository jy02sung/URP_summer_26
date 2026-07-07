#include "arm_function.h"

ArmControl::ArmControl()
{
}

ArmControl::~ArmControl()
{
}

////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------Utility-----------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
int ArmControl::sgn(double x) {
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else 
        return 0;
}

double ArmControl::LowPassFilter(double input, double output_before, double cutoff_frequency)
{
	double alpha_ = 0.0;
	double output_ = 0.0;
	alpha_ = 1 / (1 + 2*PI*cutoff_frequency*SAMPLING_TIME);
	output_ = alpha_ * output_before + (1 - alpha_)*input;
	return output_;
}


Matrix3d ArmControl::scew(Ref<Matrix<double, 3, 1>> vec_)
{
	double vx = vec_(0, 0);
	double vy = vec_(1, 0);
	double vz = vec_(2, 0);
	Matrix<double, 3, 3> scew_{ {0,	-vec_(2, 0), vec_(1, 0)},
								{vec_(2, 0), 0, -vec_(0, 0)},
								{-vec_(1, 0), vec_(0, 0), 0} };
	return scew_;
}

void ArmControl::Ry180(Matrix3d& R_)
{
	Matrix3d Ry_180_;
	Ry_180_ << -1,0,0,
				0,1,0,
				0,0,-1;
	R_ *= Ry_180_;
}

////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------Rotation----------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::Qua2Euler(Quaterniond& q, Vector3d& eul)
{
	// roll
	eul(0) = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1 - 2*(q.x()*q.x()+q.y()*q.y()));
	// pitch
	double sinp = 2*(q.w()*q.y()-q.z()*q.x());
	if(fabs(sinp) >= 1)
		eul(1) = copysign(PI/2, sinp); // use 90 degrees if out of range
	else
		eul(1) = asin(sinp);
	// yaw
	eul(2) = atan2(2*(q.w()*q.z()+q.x()*q.y()), 1 - 2*(q.y()*q.y()+q.z()*q.z()));
}

void ArmControl::Euler2Qua(Vector3d& eul, Quaterniond& q)
{
	double r_ = eul(0); // x
	double p_ = eul(1);
	double y_ = eul(2); // z
	
	q.w() = cos(y_/2) * cos(p_/2) * cos(r_/2) + sin(y_/2) * sin(p_/2) * sin(r_/2);
    q.x() = cos(y_/2) * cos(p_/2) * sin(r_/2) - sin(y_/2) * sin(p_/2) * cos(r_/2);
    q.y() = sin(y_/2) * cos(p_/2) * sin(r_/2) + cos(y_/2) * sin(p_/2) * cos(r_/2);
    q.z() = sin(y_/2) * cos(p_/2) * cos(r_/2) - cos(y_/2) * sin(p_/2) * sin(r_/2);
}

void ArmControl::Rot2Qua(Matrix3d& R, Quaterniond& q)
{
	double kw, kx, ky, kz;
	kw = 1+R(0,0)+R(1,1)+R(2,2);
	kx = R(0,0)-R(1,1)-R(2,2)+1;
	ky = -R(0,0)+R(1,1)-R(2,2)+1;
	kz = -R(0,0)-R(1,1)+R(2,2)+1;
	if(kw < 0) kw = 0.0;
	if(kx < 0) kx = 0.0;
	if(ky < 0) ky = 0.0;
	if(kz < 0) kz = 0.0;
	q.w() = 0.5*sqrt(kw);
	q.x() = 0.5*sgn(round((R(2,1)-R(1,2))*1000)/1000+0.0)*sqrt(kx);
	q.y() = 0.5*sgn(round((R(0,2)-R(2,0))*1000)/1000+0.0)*sqrt(ky);
	q.z() = 0.5*sgn(round((R(1,0)-R(0,1))*1000)/1000+0.0)*sqrt(kz);
}

void ArmControl::Eul2Rot(Vector3d& eul, Matrix3d& R)
{
	double sr = sin(eul(0)); double cr = cos(eul(0)); // x
	double sp = sin(eul(1)); double cp = cos(eul(1));
	double sy = sin(eul(2)); double cy = cos(eul(2)); // z
	R << 	cy*cp, 	cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
			sy*cp, 	sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
			-sp,	cp*sr,			cp*cr;
}

////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------Transformation-------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
Matrix4d ArmControl::T_craig(float th, float d, float al, float a)
{
	Matrix4d T_craig_;
	T_craig_ << cos(th), -sin(th), 0, a,
				sin(th)*cos(al), cos(th)*cos(al), -sin(al), -d*sin(al),
				sin(th)*sin(al), cos(th)*sin(al), cos(al), d*cos(al),
				0, 0, 0, 1;
	return T_craig_;
}

void ArmControl::Joint2Cartesian(double* th, VectorXd& pose) //, Quaterniond& q_act_, Matrix3d& R_act_)
{
	forwardKinematics(th, T06);
	R06 = T06.block(0,0,3,3);
	Rot2Qua(R06, q06);
	Qua2Euler(q06, eul06);
	pose << T06(0,3), T06(1,3), T06(2,3), eul06(0), eul06(1), eul06(2);
}

void ArmControl::Joint2CartesianQuat(double* th, VectorXd& pose_quat)
{
	forwardKinematics(th, T06);
	R06 = T06.block(0,0,3,3);
	Rot2Qua(R06, q06);
	pose_quat << T06(0,3), T06(1,3), T06(2,3), q06.w(), q06.x(), q06.y(), q06.z();
}

void ArmControl::PoseEul2Tf(const VectorXd& pose, Ref<Matrix<double, 4, 4>> T_)
{
	eul06 << pose(3), pose(4), pose(5);
	Eul2Rot(eul06, R06);
	T_.block(0,0,3,3) = R06;
	T_(0,3) = pose(0);
	T_(1,3) = pose(1);
	T_(2,3) = pose(2);
	T_(3,0) = 0;
	T_(3,1) = 0;
	T_(3,2) = 0;
	T_(3,3) = 1;
}

void ArmControl::Tf2PoseEul(const Matrix<double, 4, 4>& T_, VectorXd& pose) 
{
    // Extract the position
    pose(0) = T_(0, 3);
    pose(1) = T_(1, 3);
    pose(2) = T_(2, 3);

    // Extract the rotation matrix
    Matrix3d R;
	Quaterniond q;
	Vector3d eul;
    R = T_.block(0, 0, 3, 3);

	Rot2Qua(R, q);
	Qua2Euler(q, eul);
    
    // Set the Euler angles in the pose vector
    pose(3) = eul(0) * 180 / PI;
    pose(4) = eul(1) * 180 / PI;
    pose(5) = eul(2) * 180 / PI;
}


void ArmControl::PoseEuler2Quat(VectorXd& pose_eul, VectorXd& pose_quat)
{
	pose_quat = VectorXd::Zero(7);
	eul06(0) = pose_eul(3);
	eul06(1) = pose_eul(4);
	eul06(2) = pose_eul(5);
	Euler2Qua(eul06, q06);
	pose_quat(0) = pose_eul(0);
	pose_quat(1) = pose_eul(1);
	pose_quat(2) = pose_eul(2);
	pose_quat(3) = q06.w();
	pose_quat(4) = q06.x();
	pose_quat(5) = q06.y();
	pose_quat(6) = q06.z();
}

void ArmControl::PoseQuat2Euler(VectorXd& pose_quat, VectorXd& pose_eul)
{
	pose_eul = VectorXd::Zero(6);
	q06.w() = pose_quat(3);
	q06.x() = pose_quat(4);
	q06.y() = pose_quat(5);
	q06.z() = pose_quat(6);
	Qua2Euler(q06, eul06);
	pose_eul(0) = pose_quat(0);
	pose_eul(1) = pose_quat(1);
	pose_eul(2) = pose_quat(2);
	pose_eul(3) = eul06(0);
	pose_eul(4) = eul06(1);
	pose_eul(5) = eul06(2);
}

////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------Kinematics---------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
// input DH, output target value

void ArmControl::link5_T_cam(Matrix4d& T)
{
	Matrix4d T_ = Matrix4d::Identity();
	for (int i = 0; i < 2; i++) {
		T_ *= T_craig(th_cam[i], d_cam[i], alpha_cam[i], a_cam[i]);
	}

	T = T_;
}

void ArmControl::ab_T_link5(double* currentpos, Matrix4d& T)
{
	double th_[5] = {0,};
	for (int i = 0; i < 5; i++) {
		th_[i] = currentpos[i] + th_offset[i];
	}
	
	Matrix4d T_ = Matrix4d::Identity();
	for (int i = 0; i < 5; i++) {
		T_ *= T_craig(th_[i], d[i], alpha[i], a[i]);
	}
	T = T_;
}

void ArmControl::mb_T_ab(Matrix4d& T)
{
	Matrix4d T_ = Matrix4d::Identity();
	T_ *= T_craig(th_mb, d_mb, alpha_mb, a_mb);
	T = T_;
}

////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------Kinematics---------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
// input DH, output target value
void ArmControl::forwardKinematics(double* currentpos, Matrix4d& T)
{
	double _theta[DoF] = {0,};
    for (int i = 0; i < DoF; i++)
    {
        _theta[i] = currentpos[i] + th_offset[i];
    }

    Matrix4d T_ = Matrix4d::Identity();
    for (int i = 0; i < DoF; i++)
    {
        T_ *= T_craig(_theta[i], d[i], alpha[i], a[i]);
    }
    T_ *= Ty90; // 6 to EE TF

	T = T_;
}

void ArmControl::inverseKinematics(VectorXd& target_pose, double* currentpos, double* th_out)
{
	Matrix4d T0E_ = Matrix4d::Identity(4,4);
	Matrix4d T06_ = Matrix4d::Identity(4,4);
	Matrix4d T6E_ = Matrix4d::Identity(4,4);
	T6E_(2, 3) = d6;

	PoseEul2Tf(target_pose, T0E_);
	T0E_ *= Tyn90; // EE to link6
	T06_ = T0E_*T6E_.inverse();

	double r11 = T06_(0,0);		double r21 = T06_(1,0); 	double r31 = T06_(2,0);
	double r13 = T06_(0,2); 	double r23 = T06_(1,2);		double r33 = T06_(2,2);
	double Px = T06_(0,3); 		double Py = T06_(1,3);		double Pz = T06_(2,3) - d1;
	
	float RMSE = 0;
	int sol_num = 0;
	int ans = 0;
	int sholderSign = 1;
	int elbowSign = 1;
	int flipSign = 0;
	double 	K_, th1_, th2_, th23_, th3_, th4_, th5_, th6_,
			c1, s1, c23, s23, c3, s3, c4, s4, s5, c5, s6, c6;
	
	// PUMA560 -> delete: d3 / add: d1, d6, theta3-90, theta6+180 
	for (int i = 0; i < 8; i++)
	{
		sholderSign = (i == 0 || i == 1 || i == 2 || i == 3) ? 1 : -1;
		elbowSign = (i == 0 || i == 2 || i == 4 || i == 6) ? 1 : -1;
		flipSign = (i == 2 || i == 3 || i == 6 || i == 7) ? 1 : 0;

		K_ = (pow(Px,2)+pow(Py,2)+pow(Pz,2)-pow(a2,2)-pow(a3,2)-pow(d4,2))/(2*a2);

		th1_ = atan2(Py,sholderSign*Px);
		c1 = cos(th1_);
		s1 = sin(th1_);
	
		th3_ = atan2(a3,d4) - atan2(K_, elbowSign*sqrt(pow(a3,2)+pow(d4,2)-pow(K_,2)));
		c3 = cos(th3_);
		s3 = sin(th3_);

		th23_ = atan2((-a3-a2*c3)*Pz - (c1*Px+s1*Py)*(d4-a2*s3), (a2*s3-d4)*Pz + (a3+a2*c3)*(c1*Px+s1*Py));
		th2_ = th23_ - th3_;
		s23 = ((-a3-a2*c3)*Pz + (c1*Px+s1*Py)*(a2*s3-d4))/(pow(Pz,2)+pow(c1*Px+s1*Py,2));
		c23 = ((a2*s3-d4)*Pz + (a3+a2*c3)*(c1*Px+s1*Py))/(pow(Pz,2)+pow(c1*Px+s1*Py,2));
		
		th4_ = atan2(-r13*s1+r23*c1, -r13*c1*c23 - r23*s1*c23 + r33*s23);
		c4 = cos(th4_);
		s4 = sin(th4_);
		
		s5 = -(r13*(c1*c23*c4+s1*s4) + r23*(s1*c23*c4-c1*s4) - r33*s23*c4);
		c5 = r13*(-c1*s23) + r23*(-s1*s23) + r33*(-c23);
		th5_ = atan2(s5,c5);
	
		s6 = -r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4+c1*c4) + r31*(s23*s4);
		c6 = r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5);
		th6_ = atan2(s6,c6);
	
		th3_ += PI/2;
		th6_ -= PI;

		// Check wrist flip condition
        if (flipSign == 1)
        {
            th4_ += PI;
            th5_ = -th5_;
            th6_ += PI;
        }

        if(th1_ >= PI*1.01) th1_ -= 2*PI;
		else if(th1_ <= -PI*1.01) th1_ += 2*PI;
		if(th4_ >= PI*1.01) th4_ -= 2*PI;
		else if(th4_ <= -PI*1.01) th4_ += 2*PI;
		if(th6_ >= PI*1.01) th6_ -= 2*PI;
		else if(th6_ <= -PI*1.01) th6_ += 2*PI;

        if ((th2_ <= 0 && th2_ >= -PI) && (th3_ <= PI && th3_ >= 0) && (th5_ <= PI / 2 && th5_ >= -PI / 2))
        {
            if (ans == 0)
            { // First solution, calculate RMSE for comparing next solutions
                sol_num++;
                th_out[0] = th1_;
                th_out[1] = th2_;
                th_out[2] = th3_;
                th_out[3] = th4_;
                th_out[4] = th5_;
                th_out[5] = th6_;
                RMSE = sqrt(pow(currentpos[0] - th1_, 2) + pow(currentpos[3] - th4_, 2) + pow(currentpos[4] - th5_, 2) +
                            pow(currentpos[5] - th6_, 2));
                ans = 1;
                continue;
            }
            else if (RMSE > sqrt(pow(currentpos[0] - th1_, 2) + pow(currentpos[3] - th4_, 2) +
                                 pow(currentpos[4] - th5_, 2) + pow(currentpos[5] - th6_, 2)))
            { // When there is a solution exist, compare RMSE with current position
                th_out[0] = th1_;
                th_out[1] = th2_;
                th_out[2] = th3_;
                th_out[3] = th4_;
                th_out[4] = th5_;
                th_out[5] = th6_;
                RMSE = sqrt(pow(currentpos[0] - th1_, 2) + pow(currentpos[3] - th4_, 2) + pow(currentpos[4] - th5_, 2) +
                            pow(currentpos[5] - th6_, 2));
            }
            continue;
        }
        if (i == 8 && ans == 0)
        {
            for (int i = 0; i < 6; i++)
            {
                th_out[i] = currentpos[i];
            }
            cout << "Have no answer!!" << '\n';
        }
    }
}


void ArmControl::Inverse_Newton(VectorXd& target_pose_, double* currentpos_, VectorXd& pose_act_, double* th_out_)
{
	MatrixXd I_ = MatrixXd::Identity(6,6);
	MatrixXd I_damp = MatrixXd::Identity(6,6);

	Jacobian(currentpos_, J06);
	Angvel2Eul(eul06, J06);

	I_damp = damp_const_pow*I_;					// Damped least squares
	inv_Jaco_1 = J06*J06.transpose() + I_damp; 	// Moore-Penrose pseudo-inverse 1
	inv_Jaco = J06.transpose()*inv_Jaco_1.inverse();

    for (int i = 0; i < DoF; i++)
    {
        pose_error(i) = target_pose_(i) - pose_act_(i);
    }

    angle_error = inv_Jaco*pose_error;

	for(int i = 0; i < DoF ;i++)
	{
		if(isnan(angle_error(i))){
			angle_error(i) = 0;
		}
		
		th_out_[i] = currentpos_[i] + angle_error(i);
	}
}


void ArmControl::Inverse_Newton_Quat(VectorXd& target_pose_, double* currentpos_, double* th_out_)
{
	MatrixXd I_ = MatrixXd::Identity(6,6);
	MatrixXd I_damp = MatrixXd::Identity(6,6);
	forwardKinematics(currentpos_, Cur_HTM);
    R06 = Cur_HTM.block(0, 0, 3, 3);
    Ry180(R06);
	Rot2Qua(R06, q06);
	Qua2Euler(q06, eul06);
	Jacobian(currentpos_, J06);
	Angvel2Quat(q06, J06, J06_7);

	I_damp = damp_const_pow*I_;					// Damped least squares=
	inv_Jaco_1 = J06_7.transpose()*J06_7 + I_damp; 	// Moore-Penrose pseudo-inverse 2
	inv_Jaco = inv_Jaco_1.inverse()*J06_7.transpose();

	pose_error(0) = target_pose_(0) - Cur_HTM(0,3);
	pose_error(1) = target_pose_(1) - Cur_HTM(1,3);
	pose_error(2) = target_pose_(2) - Cur_HTM(2,3);
	pose_error(3) = target_pose_(3) - q06.w();
	pose_error(4) = target_pose_(4) - q06.x();
	pose_error(5) = target_pose_(5) - q06.y();
	pose_error(6) = target_pose_(6) - q06.z();

	angle_error = inv_Jaco*pose_error;

	for(int i = 0; i < DoF; i++)
	{
		if(isnan(angle_error(i))){
			angle_error(i) = 0;
		}
		
		th_out_[i] = currentpos_[i] + angle_error(i);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------Jacobian---------------------------------------//
//////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::Jacobian(double* currentpos_, MatrixXd& J06)
{
	double c1 = cos(currentpos_[0]), c2 = cos(currentpos_[1]), c3 = cos(currentpos_[2]), c4 = cos(currentpos_[3]), c5 = cos(currentpos_[4]), c6 = cos(currentpos_[5]);
	double s1 = sin(currentpos_[0]), s2 = sin(currentpos_[1]), s3 = sin(currentpos_[2]), s4 = sin(currentpos_[3]), s5 = sin(currentpos_[4]), s6 = sin(currentpos_[5]);
	// theta*
	J06 <<  d4*(c2*s1*(-c3) + s3*s1*s2) - a3*(c2*s3*s1 - s1*s2*(-c3)) - d6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) - a2*c2*s1,   d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + d6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) - a2*c1*s2,   d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + d6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)), -d6*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)), -d6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)), 0,
			a2*c1*c2 - d4*(c1*c2*(-c3) + c1*s3*s2) - d6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) - a3*(c1*s2*(-c3) - c1*c2*s3), - a3*(c2*s1*(-c3) + s3*s1*s2) - d4*(c2*s3*s1 - s1*s2*(-c3)) - d6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) - a2*s1*s2, - a3*(c2*s1*(-c3) + s3*s1*s2) - d4*(c2*s3*s1 - s1*s2*(-c3)) - d6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)),  d6*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))),  d6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)), 0,
			0,																																					  d4*(c2*(-c3) + s3*s2) - a3*(c2*s3 - s2*(-c3)) - a2*c2 + d6*(c5*(c2*(-c3) + s3*s2) + c4*s5*(c2*s3 - s2*(-c3))),                              d4*(c2*(-c3) + s3*s2) - a3*(c2*s3 - s2*(-c3)) + d6*(c5*(c2*(-c3) + s3*s2) + c4*s5*(c2*s3 - s2*(-c3))),                         -d6*s4*s5*(c2*(-c3) + s3*s2),			        d6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)), 							    0,
			0,                                                                                                                                                  - s1,																																		- s1,                                             																				- c1*c2*(-c3) - c1*s3*s2,                     - c4*s1 - s4*(c1*s2*(-c3) - c1*c2*s3), 	- s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - c5*(c1*c2*(-c3) + c1*s3*s2),
			0,                                                                                                                                                    c1,																																		  c1,                                             																				- c2*s1*(-c3) - s3*s1*s2,                       c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)), 	  s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2),
			1,                                                                                                                                                     0,																																		   0,                                                                 															  s2*(-c3) - c2*s3,                           - s4*(c2*(-c3) + s3*s2),                   	  c4*s5*(c2*(-c3) + s3*s2) - c5*(c2*s3 - s2*(-c3));
}

// Change angular velocity to euler angle velocity
void ArmControl::Angvel2Eul(Vector3d& eul, MatrixXd& J06)
{
	double r_ = eul(0);
	double p_ = eul(1);
	double y_ = eul(2);
	Matrix3d K_, inv_K_;
	MatrixXd Rot_J_ = MatrixXd::Identity(6,6);
	K_ << 	0,	-sin(r_),	cos(r_)*cos(p_),
			0,	cos(r_),	sin(r_)*cos(p_),
			1,	0,			-sin(p_);
	inv_K_ = K_.inverse();
	Rot_J_.block(3,3,3,3) = inv_K_;
	J06 = Rot_J_*J06;
}

// Change angular velocity to quaternion velocity
void ArmControl::Angvel2Quat(Quaterniond& q, MatrixXd& J06, MatrixXd& J06_7)
{
	double w_ = q.w();
	double x_ = q.x();
	double y_ = q.y();
	double z_ = q.z();
	MatrixXd K_(4,4);
	MatrixXd Rot_J_ = MatrixXd::Identity(7,7);
	MatrixXd J06_7_ = MatrixXd::Zero(7,6);
	K_ << 	0.5*w_,	-0.5*x_, -0.5*y_, -0.5*z_,
			0.5*x_,	 0.5*w_, -0.5*z_,  0.5*y_,
			0.5*y_,  0.5*z_,  0.5*w_, -0.5*x_,
			0.5*z_, -0.5*y_,  0.5*x_,  0.5*w_;
	Rot_J_.block(3,3,4,4) = K_;
	J06_7_.block(0,0,3,6) = J06.block(0,0,3,6);
	J06_7_.block(4,0,3,6) = J06.block(3,0,3,6);	

	J06_7 = Rot_J_*J06_7_;
}

////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------Dynamics----------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::RNEA(const double *q, const double *dq, const double *ddq, Ref<Matrix<double, 3, 1>> omega_b,
                      Ref<Matrix<double, 3, 1>> acc_b, Ref<Matrix<double, 3, 1>> grav_local, double *arm_torque)
{
	double q_[DoF] = {0, };
	// Vector3d grav;
	// grav << 0, 0, Gravity;
	Vector3d z_0;
    z_0 << 0, 0, 1;

	Vector3d w_i, wdot_i;
	Vector3d Pc_i;
	Matrix3d Ic_i;

	Matrix4d T_ = MatrixXd::Identity(4, 4);
	Matrix3d R_ = MatrixXd::Identity(3, 3);
	Matrix3d R_t_ = MatrixXd::Identity(3, 3);
	Vector3d P_ = MatrixXd::Zero(3, 1);

	Vector3d F_i = MatrixXd::Zero(3, 1);
	Vector3d N_i = MatrixXd::Zero(3, 1);
	Vector3d n_i = MatrixXd::Zero(3, 1);
	Vector3d ac_i = MatrixXd::Zero(3, 1);

	a_.block(0, 0, 3, 1) << 0, 0, g; // Gravity acc

	// Quadruped modi
	// omega_.block(0, 0, 3, 1) = omega_b;
	// a_.block(0, 0, 3, 1) = acc_b + scew(omega_b)*scew(omega_b)*P_body2armbase + grav_local;

	for (int i = 0; i < DoF; i++) {
		q_[i] = q[i] + th_offset[i];
	}

	// forward recursion
	for (int i = 0; i < DoF; i++)
	{
		T_ = T_craig(q_[i], d[i], alpha[i], a[i]); // link i to i+1
		R_ = T_.block(0, 0, 3, 3);
		R_t_ = R_.transpose();
		P_ = T_.block(0, 3, 3, 1);
		
		if (i > 0) {
			w_i = omega_.block(0, i, 3, 1);
			wdot_i = d_omega_.block(0, i, 3, 1);
			// warning index; i 0 = link 0
			omega_.block(0, i+1, 3, 1) = R_t_*w_i + dq[i]*z_0; // dq: index0 = joint1
			d_omega_.block(0, i+1, 3, 1) = R_t_*( wdot_i + (scew(w_i)*z_0*dq[i]) ) + ddq[i]*z_0;
			a_.block(0, i+1, 3, 1) = R_t_*( a_.block(0, i, 3, 1) + scew(wdot_i)*P_ + scew(w_i)*scew(w_i)*P_ );
		}
		else {
			omega_.block(0, i+1, 3, 1) = dq[i]*z_0;
			d_omega_.block(0, i+1, 3, 1) = ddq[i]*z_0;
			a_.block(0, i+1, 3, 1) = R_t_*a_.block(0, 0, 3, 1);
		}
	}

	// backward recursion
	for (int i = DoF; i >= 0; i--)
	{
		w_i = omega_.block(0, i, 3, 1);
		wdot_i = d_omega_.block(0, i, 3, 1);
		Pc_i = arm_CoM_pos_local.block(i, 0, 1, 3).transpose();
		Ic_i << I_xx[i],	I_xy[i],	0,
				I_xy[i],	I_yy[i],	0,
				0,			0,			I_zz[i];

		ac_i = a_.block(0, i, 3, 1) + scew(wdot_i)*Pc_i + scew(w_i)*scew(w_i)*Pc_i;
		F_i = mass[i]*ac_i;
		N_i = Ic_i*wdot_i + scew(w_i)*Ic_i*w_i;

		if (i < DoF) {
			T_ = T_craig(q_[i], d[i], alpha[i], a[i]); // link i to i+1
			R_ = T_.block(0, 0, 3, 3);
			P_ = T_.block(0, 3, 3, 1);

			f_.block(0, i, 3, 1) = R_*f_.block(0, i+1, 3, 1) + F_i;
			n_.block(0, i, 3, 1) = N_i + R_*n_.block(0, i+1, 3, 1) + scew(Pc_i)*F_i + scew(P_)*R_*f_.block(0, i+1, 3, 1);
		}
		else {
			f_.block(0, i, 3, 1) = F_i;
			n_.block(0, i, 3, 1) = N_i + scew(Pc_i)*F_i;
		}

		n_i = n_.block(0, i, 3, 1);
		
		if (i > 0) {
			arm_torque[i-1] = n_i.transpose()*z_0;
		}
	}
}

void ArmControl::MassMatrix(const double* th, MatrixXd& mass_matrix)
{
	double 	t1_, t4_, t5_;
	t1_ = th[0];
	t4_ = th[3];
	t5_ = th[4];
	double  s1, s2, s3, s4, s5, s6,
			c1, c2, c3, c4, c5, c6;
	s1 = sin(th[0]); s2 = sin(th[1]); s3 = sin(th[2]); s4 = sin(th[3]); s5 = sin(th[4]); s6 = sin(th[5]);
	c1 = cos(th[0]); c2 = cos(th[1]); c3 = cos(th[2]); c4 = cos(th[3]); c5 = cos(th[4]); c6 = cos(th[5]);

	mass_matrix(0,0) = I_zz[1] + I_zz[2] + I_zz[3] + I_zz[4] + I_zz[5] + I_zz[6] + m4*pow(x_b4*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + y_b4*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - a3*(c2*s3*s1 - s1*s2*(-c3)) + d4*(c2*s1*(-c3) + s3*s1*s2) + z_b4*(c2*s1*(-c3) + s3*s1*s2) - a2*c2*s1, 2) + m3*pow(x_b3*(c1*s2*(-c3) - c1*c2*s3) + y_b3*(c1*c2*(-c3) + c1*s3*s2) + z_b3*s1 - a2*c1*c2, 2) + m2*pow(z_b2*s1 - x_b2*c1*c2 + y_b2*c1*s2, 2) + m2*pow(z_b2*c1 + x_b2*c2*s1 - y_b2*s1*s2, 2) + m3*pow(x_b3*(c2*s3*s1 - s1*s2*(-c3)) - y_b3*(c2*s1*(-c3) + s3*s1*s2) + z_b3*c1 + a2*c2*s1, 2) + m6*pow(a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) + d6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + x_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) + y_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - a2*c1*c2, 2) + m4*pow(x_b4*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + y_b4*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - a3*(c1*s2*(-c3) - c1*c2*s3) - d4*(c1*c2*(-c3) + c1*s3*s2) - z_b4*(c1*c2*(-c3) + c1*s3*s2) + a2*c1*c2, 2) + m5*pow(z_b5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) - x_b5*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + a2*c2*s1, 2) + m6*pow(a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) + d6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + x_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + y_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + a2*c2*s1, 2) + m5*pow(z_b5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) - x_b5*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) - a2*c1*c2, 2) + m1*pow(y_b1*c1 + x_b1*s1, 2) + m1*pow(x_b1*c1 - y_b1*s1, 2);
	mass_matrix(0,1) = m5*(a3*(c1*c2*(-c3) + c1*s3*s2) - d4*(c1*s2*(-c3) - c1*c2*s3) - x_b5*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) - y_b5*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b5*s4*(c1*c2*(-c3) + c1*s3*s2) + a2*c1*s2)*(z_b5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) - x_b5*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + a2*c2*s1) - m6*(a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) + d6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + x_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + y_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + a2*c2*s1)*(d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + d6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) - x_b6*(c6*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) + s4*s6*(c1*c2*(-c3) + c1*s3*s2)) + y_b6*(s6*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) - c6*s4*(c1*c2*(-c3) + c1*s3*s2)) - a2*c1*s2) + m3*(x_b3*(c1*c2*(-c3) + c1*s3*s2) - y_b3*(c1*s2*(-c3) - c1*c2*s3) + a2*c1*s2)*(x_b3*(c2*s3*s1 - s1*s2*(-c3)) - y_b3*(c2*s1*(-c3) + s3*s1*s2) + z_b3*c1 + a2*c2*s1) - m4*(x_b4*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + y_b4*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - a3*(c2*s3*s1 - s1*s2*(-c3)) + d4*(c2*s1*(-c3) + s3*s1*s2) + z_b4*(c2*s1*(-c3) + s3*s1*s2) - a2*c2*s1)*(a3*(c1*c2*(-c3) + c1*s3*s2) - d4*(c1*s2*(-c3) - c1*c2*s3) - z_b4*(c1*s2*(-c3) - c1*c2*s3) + x_b4*c4*(c1*c2*(-c3) + c1*s3*s2) - y_b4*s4*(c1*c2*(-c3) + c1*s3*s2) + a2*c1*s2) + m3*(x_b3*(c2*s1*(-c3) + s3*s1*s2) + y_b3*(c2*s3*s1 - s1*s2*(-c3)) + a2*s1*s2)*(x_b3*(c1*s2*(-c3) - c1*c2*s3) + y_b3*(c1*c2*(-c3) + c1*s3*s2) + z_b3*s1 - a2*c1*c2) + m2*(y_b2*c1*c2 + x_b2*c1*s2)*(z_b2*c1 + x_b2*c2*s1 - y_b2*s1*s2) + m5*(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + x_b5*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b5*s4*(c2*s1*(-c3) + s3*s1*s2) + a2*s1*s2)*(z_b5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) - x_b5*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) - a2*c1*c2) + m2*(y_b2*c2*s1 + x_b2*s1*s2)*(z_b2*s1 - x_b2*c1*c2 + y_b2*c1*s2) - m4*(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + z_b4*(c2*s3*s1 - s1*s2*(-c3)) + x_b4*c4*(c2*s1*(-c3) + s3*s1*s2) - y_b4*s4*(c2*s1*(-c3) + s3*s1*s2) + a2*s1*s2)*(x_b4*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + y_b4*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - a3*(c1*s2*(-c3) - c1*c2*s3) - d4*(c1*c2*(-c3) + c1*s3*s2) - z_b4*(c1*c2*(-c3) + c1*s3*s2) + a2*c1*c2) + m6*(a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) + d6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + x_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) + y_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - a2*c1*c2)*(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + d6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) - x_b6*(c6*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) - s4*s6*(c2*s1*(-c3) + s3*s1*s2)) + y_b6*(s6*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) + c6*s4*(c2*s1*(-c3) + s3*s1*s2)) + a2*s1*s2);
	mass_matrix(0,2) = m3*(x_b3*(c2*s1*(-c3) + s3*s1*s2) + y_b3*(c2*s3*s1 - s1*s2*(-c3)))*(x_b3*(c1*s2*(-c3) - c1*c2*s3) + y_b3*(c1*c2*(-c3) + c1*s3*s2) + z_b3*s1 - a2*c1*c2) + m3*(x_b3*(c1*c2*(-c3) + c1*s3*s2) - y_b3*(c1*s2*(-c3) - c1*c2*s3))*(x_b3*(c2*s3*s1 - s1*s2*(-c3)) - y_b3*(c2*s1*(-c3) + s3*s1*s2) + z_b3*c1 + a2*c2*s1) + m4*(d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + z_b4*(c1*s2*(-c3) - c1*c2*s3) - x_b4*c4*(c1*c2*(-c3) + c1*s3*s2) + y_b4*s4*(c1*c2*(-c3) + c1*s3*s2))*(x_b4*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + y_b4*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - a3*(c2*s3*s1 - s1*s2*(-c3)) + d4*(c2*s1*(-c3) + s3*s1*s2) + z_b4*(c2*s1*(-c3) + s3*s1*s2) - a2*c2*s1) + m5*(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + x_b5*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b5*s4*(c2*s1*(-c3) + s3*s1*s2))*(z_b5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) - x_b5*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) - a2*c1*c2) - m4*(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + z_b4*(c2*s3*s1 - s1*s2*(-c3)) + x_b4*c4*(c2*s1*(-c3) + s3*s1*s2) - y_b4*s4*(c2*s1*(-c3) + s3*s1*s2))*(x_b4*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + y_b4*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - a3*(c1*s2*(-c3) - c1*c2*s3) - d4*(c1*c2*(-c3) + c1*s3*s2) - z_b4*(c1*c2*(-c3) + c1*s3*s2) + a2*c1*c2) + m6*(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + d6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) - x_b6*(c6*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) - s4*s6*(c2*s1*(-c3) + s3*s1*s2)) + y_b6*(s6*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) + c6*s4*(c2*s1*(-c3) + s3*s1*s2)))*(a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) + d6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + x_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) + y_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - a2*c1*c2) - m6*(d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + d6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) - x_b6*(c6*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) + s4*s6*(c1*c2*(-c3) + c1*s3*s2)) + y_b6*(s6*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) - c6*s4*(c1*c2*(-c3) + c1*s3*s2)))*(a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) + d6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + x_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + y_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + a2*c2*s1) - m5*(d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + x_b5*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) - z_b5*s4*(c1*c2*(-c3) + c1*s3*s2))*(z_b5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) - x_b5*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + a2*c2*s1);
	mass_matrix(0,3) = m4*(x_b4*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - y_b4*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)))*(x_b4*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + y_b4*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - a3*(c2*s3*s1 - s1*s2*(-c3)) + d4*(c2*s1*(-c3) + s3*s1*s2) + z_b4*(c2*s1*(-c3) + s3*s1*s2) - a2*c2*s1) - I_zz[5]*(c2*s3 - s2*(-c3)) - I_zz[6]*(c2*s3 - s2*(-c3)) - I_zz[4]*(c2*s3 - s2*(-c3)) - m6*(x_b6*(s6*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - c5*c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3))) + y_b6*(c6*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3))) - d6*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - z_b6*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)))*(a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) + d6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + x_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + y_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + a2*c2*s1) + m6*(x_b6*(s6*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)))) + y_b6*(c6*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + c5*s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)))) - d6*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - z_b6*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))))*(a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) + d6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + x_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) + y_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - a2*c1*c2) - m5*(z_b5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + x_b5*c5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - y_b5*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)))*(z_b5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) - x_b5*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + a2*c2*s1) - m4*(x_b4*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - y_b4*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))))*(x_b4*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + y_b4*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - a3*(c1*s2*(-c3) - c1*c2*s3) - d4*(c1*c2*(-c3) + c1*s3*s2) - z_b4*(c1*c2*(-c3) + c1*s3*s2) + a2*c1*c2) + m5*(z_b5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + x_b5*c5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - y_b5*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))))*(z_b5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) - x_b5*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) - a2*c1*c2);
	mass_matrix(0,4) = m5*(x_b5*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)))*(z_b5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) - x_b5*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + a2*c2*s1) - m6*(d6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) - x_b6*c6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b6*s6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)))*(a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) + d6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + x_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) + y_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - a2*c1*c2) - I_zz[5]*s4*(c2*(-c3) + s3*s2) - I_zz[6]*s4*(c2*(-c3) + s3*s2) + m6*(d6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) - x_b6*c6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + y_b6*s6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)))*(a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) + d6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + x_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + y_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + a2*c2*s1) - m5*(x_b5*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)))*(z_b5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) - x_b5*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) - a2*c1*c2);
	mass_matrix(0,5) = m6*(x_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - y_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))))*(a3*(c2*s3*s1 - s1*s2*(-c3)) - d4*(c2*s1*(-c3) + s3*s1*s2) + d6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + x_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + y_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) + a2*c2*s1) - I_zz[6]*(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2)) - m6*(x_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) - y_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))))*(a3*(c1*s2*(-c3) - c1*c2*s3) + d4*(c1*c2*(-c3) + c1*s3*s2) + d6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + x_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) + y_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - a2*c1*c2);

	mass_matrix(1,1) = m6*pow(z_b6*(c5*(c2*(-c3) + s3*s2) + c4*s5*(c2*s3 - s2*(-c3))) - a3*(c2*s3 - s2*(-c3)) + d4*(c2*(-c3) + s3*s2) - x_b6*(c6*(s5*(c2*(-c3) + s3*s2) - c4*c5*(c2*s3 - s2*(-c3))) + s4*s6*(c2*s3 - s2*(-c3))) + y_b6*(s6*(s5*(c2*(-c3) + s3*s2) - c4*c5*(c2*s3 - s2*(-c3))) - c6*s4*(c2*s3 - s2*(-c3))) - a2*c2 + d6*(c5*(c2*(-c3) + s3*s2) + c4*s5*(c2*s3 - s2*(-c3))), 2) + m6*pow(d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + d6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) - x_b6*(c6*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) + s4*s6*(c1*c2*(-c3) + c1*s3*s2)) + y_b6*(s6*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) - c6*s4*(c1*c2*(-c3) + c1*s3*s2)) - a2*c1*s2, 2) + m5*pow(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + x_b5*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b5*s4*(c2*s1*(-c3) + s3*s1*s2) + a2*s1*s2, 2) + m3*pow(x_b3*(c2*s3 - s2*(-c3)) - y_b3*(c2*(-c3) + s3*s2) + a2*c2, 2) + m5*pow(a3*(c1*c2*(-c3) + c1*s3*s2) - d4*(c1*s2*(-c3) - c1*c2*s3) - x_b5*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) - y_b5*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b5*s4*(c1*c2*(-c3) + c1*s3*s2) + a2*c1*s2, 2) + m3*pow(x_b3*(c1*c2*(-c3) + c1*s3*s2) - y_b3*(c1*s2*(-c3) - c1*c2*s3) + a2*c1*s2, 2) + m4*pow(a3*(c2*s3 - s2*(-c3)) - d4*(c2*(-c3) + s3*s2) - z_b4*(c2*(-c3) + s3*s2) + a2*c2 + x_b4*c4*(c2*s3 - s2*(-c3)) - y_b4*s4*(c2*s3 - s2*(-c3)), 2) + m4*pow(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + z_b4*(c2*s3*s1 - s1*s2*(-c3)) + x_b4*c4*(c2*s1*(-c3) + s3*s1*s2) - y_b4*s4*(c2*s1*(-c3) + s3*s1*s2) + a2*s1*s2, 2) + m5*pow(x_b5*(s5*(c2*(-c3) + s3*s2) - c4*c5*(c2*s3 - s2*(-c3))) + y_b5*(c5*(c2*(-c3) + s3*s2) + c4*s5*(c2*s3 - s2*(-c3))) - a3*(c2*s3 - s2*(-c3)) + d4*(c2*(-c3) + s3*s2) - a2*c2 - z_b5*s4*(c2*s3 - s2*(-c3)), 2) + c1*(I_yy[3]*c1 - I_xy[3]*s1) + m2*pow(y_b2*c1*c2 + x_b2*c1*s2, 2) + m6*pow(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + d6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) - x_b6*(c6*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) - s4*s6*(c2*s1*(-c3) + s3*s1*s2)) + y_b6*(s6*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) + c6*s4*(c2*s1*(-c3) + s3*s1*s2)) + a2*s1*s2, 2) - s1*(I_xy[3]*c1 - I_xx[3]*s1) + m2*pow(y_b2*c2*s1 + x_b2*s1*s2, 2) + m3*pow(x_b3*(c2*s1*(-c3) + s3*s1*s2) + y_b3*(c2*s3*s1 - s1*s2*(-c3)) + a2*s1*s2, 2) + I_yy[2]*pow(c1, 2)+ I_yy[4]*pow(c1, 2)+ I_yy[5]*pow(c1, 2)+ I_yy[6]*pow(c1, 2)+ m4*pow(a3*(c1*c2*(-c3) + c1*s3*s2) - d4*(c1*s2*(-c3) - c1*c2*s3) - z_b4*(c1*s2*(-c3) - c1*c2*s3) + x_b4*c4*(c1*c2*(-c3) + c1*s3*s2) - y_b4*s4*(c1*c2*(-c3) + c1*s3*s2) + a2*c1*s2, 2) + I_xx[2]*pow(s1, 2)+ I_xx[4]*pow(s1, 2)+ I_xx[5]*pow(s1, 2)+ I_xx[6]*pow(s1, 2)+ m2*pow(x_b2*c2 - y_b2*s2, 2);
	mass_matrix(1,2) = I_xx[3] + I_xx[4] + I_xx[5] + I_xx[6] + pow(a3, 2)*m4 + pow(a3, 2)*m5 + pow(a3, 2)*m6 + pow(d4, 2)*m4 + pow(d4, 2)*m5 + pow(d4, 2)*m6 + m3*pow(x_b3, 2) + m5*pow(x_b5, 2) + m6*pow(x_b6, 2) + m3*pow(y_b3, 2) + m4*pow(y_b4, 2) + m6*pow(y_b6, 2) + m4*pow(z_b4, 2) + m5*pow(z_b5, 2) - I_xx[3]*pow(c1,2) - I_xx[4]*pow(c1,2) + I_yy[3]*pow(c1,2) - I_xx[5]*pow(c1,2) + I_yy[4]*pow(c1,2) - I_xx[6]*pow(c1,2) + I_yy[5]*pow(c1,2) + I_yy[6]*pow(c1,2) - I_xy[3]*sin(2*t1_) + 2*d4*m4*z_b4 + pow(d6, 2)*m6*pow(c4, 2) + pow(d6, 2)*m6*pow(c5, 2) + m4*pow(x_b4, 2)*pow(c4, 2) - m5*pow(x_b5, 2)*pow(c5, 2) - m6*pow(x_b6, 2)*pow(c4, 2) - m4*pow(y_b4, 2)*pow(c4, 2) + m5*pow(y_b5, 2)*pow(c4, 2) + m5*pow(y_b5, 2)*pow(c5, 2) - m6*pow(y_b6, 2)*pow(c5, 2) - m5*pow(z_b5, 2)*pow(c4, 2) + m6*pow(z_b6, 2)*pow(c4, 2) + m6*pow(z_b6, 2)*pow(c5, 2) - m4*x_b4*y_b4*sin(2*t4_) + m5*x_b5*y_b5*sin(2*t5_) + m5*pow(x_b5, 2)*pow(c4, 2)*pow(c5, 2) + m6*pow(x_b6, 2)*pow(c4, 2)*pow(c6, 2) - m6*pow(x_b6, 2)*pow(c5, 2)*pow(c6, 2) - m5*pow(y_b5, 2)*pow(c4, 2)*pow(c5, 2) + m6*pow(y_b6, 2)*pow(c4, 2)*pow(c5, 2) - m6*pow(y_b6, 2)*pow(c4, 2)*pow(c6, 2) + m6*pow(y_b6, 2)*pow(c5, 2)*pow(c6, 2) - m6*pow(z_b6, 2)*pow(c4, 2)*pow(c5, 2) + a2*d4*m4*c3 + a2*d4*m5*c3 + a2*d4*m6*c3 + 2*d4*d6*m6*c5 + a2*a3*m4*s3 + a2*a3*m5*s3 + a2*a3*m6*s3 + 2*a3*m4*x_b4*c4 + a2*m3*y_b3*c3 + a2*m4*z_b4*c3 + 2*d4*m5*y_b5*c5 + 2*d4*m6*z_b6*c5 + a2*m3*x_b3*s3 - 2*a3*m4*y_b4*s4 + 2*a3*m5*z_b5*s4 + 2*d4*m5*x_b5*s5 + 2*d6*m6*z_b6*pow(c4, 2) + 2*d6*m6*z_b6*pow(c5, 2) - pow(d6, 2)*m6*pow(c4, 2)*pow(c5, 2) - 2*d6*m6*z_b6*pow(c4, 2)*pow(c5, 2) + a2*d6*m6*c3*c5 + m6*pow(x_b6, 2)*pow(c4, 2)*pow(c5, 2)*pow(c6, 2) - m6*pow(y_b6, 2)*pow(c4, 2)*pow(c5, 2)*pow(c6, 2) - 2*a3*d6*m6*c4*s5 + 2*a3*m5*x_b5*c4*c5 + a2*m5*y_b5*c3*c5 + a2*m6*z_b6*c3*c5 + a2*m4*x_b4*c4*s3 + a2*m5*x_b5*c3*s5 - 2*a3*m5*y_b5*c4*s5 + 2*a3*m6*y_b6*c6*s4 - 2*a3*m6*z_b6*c4*s5 - 2*d4*m6*x_b6*c6*s5 + 2*a3*m6*x_b6*s4*s6 - a2*m4*y_b4*s3*s4 + a2*m5*z_b5*s3*s4 + 2*d4*m6*y_b6*s5*s6 - 2*a3*m6*x_b6*c4*c5*c6 - a2*d6*m6*c4*s3*s5 + a2*m5*x_b5*c4*c5*s3 - a2*m6*x_b6*c3*c6*s5 + 2*a3*m6*y_b6*c4*c5*s6 - 2*d6*m6*x_b6*c5*c6*s5 - a2*m5*y_b5*c4*s3*s5 + a2*m6*y_b6*c6*s3*s4 + a2*m6*y_b6*c3*s5*s6 - a2*m6*z_b6*c4*s3*s5 + 2*d6*m6*y_b6*c5*s5*s6 + 2*m6*x_b6*y_b6*c4*c5*s4 + 2*m5*x_b5*z_b5*c4*c5*s4 - 2*m6*x_b6*z_b6*c5*c6*s5 + a2*m6*x_b6*s3*s4*s6 - 2*m5*y_b5*z_b5*c4*s4*s5 + 2*m6*y_b6*z_b6*c5*s5*s6 - 2*m5*x_b5*y_b5*pow(c4, 2)*c5*s5 - 2*m6*x_b6*y_b6*pow(c4, 2)*c6*s6 + 2*m6*x_b6*y_b6*pow(c5, 2)*c6*s6 - a2*m6*x_b6*c4*c5*c6*s3 + a2*m6*y_b6*c4*c5*s3*s6 - 2*d6*m6*y_b6*c4*c6*s4*s5 - 2*d6*m6*x_b6*c4*s4*s5*s6 - 2*m6*y_b6*z_b6*c4*c6*s4*s5 - 2*m6*x_b6*z_b6*c4*s4*s5*s6 + 2*d6*m6*x_b6*pow(c4, 2)*c5*c6*s5 - 2*d6*m6*y_b6*pow(c4, 2)*c5*s5*s6 - 4*m6*x_b6*y_b6*c4*c5*pow(c6, 2)*s4 + 2*m6*x_b6*z_b6*pow(c4, 2)*c5*c6*s5 - 2*m6*y_b6*z_b6*pow(c4, 2)*c5*s5*s6 - 2*m6*x_b6*y_b6*pow(c4, 2)*pow(c5, 2)*c6*s6 - 2*m6*pow(x_b6, 2)*c4*c5*c6*s4*s6 + 2*m6*pow(y_b6, 2)*c4*c5*c6*s4*s6;
	mass_matrix(1,3) = d4*m4*y_b4*c4 - d4*m5*z_b5*c4 + d4*m4*x_b4*s4 + m4*y_b4*z_b4*c4 - m5*x_b5*y_b5*s4 + m4*x_b4*z_b4*s4 - I_xx[4]*c1*c2*c3*s1 - I_xx[5]*c1*c2*c3*s1 + I_yy[4]*c1*c2*c3*s1 - I_xx[6]*c1*c2*c3*s1 + I_yy[5]*c1*c2*c3*s1 + I_yy[6]*c1*c2*c3*s1 + I_xx[4]*c1*s1*s2*s3 + I_xx[5]*c1*s1*s2*s3 - I_yy[4]*c1*s1*s2*s3 + I_xx[6]*c1*s1*s2*s3 - I_yy[5]*c1*s1*s2*s3 - I_yy[6]*c1*s1*s2*s3 - pow(d6, 2)*m6*c5*s4*s5 + m5*pow(x_b5, 2)*c5*s4*s5 - m5*pow(y_b5, 2)*c5*s4*s5 + m6*pow(y_b6, 2)*c5*s4*s5 - m6*pow(z_b6, 2)*c5*s4*s5 + a2*m4*y_b4*c3*c4 - a2*m5*z_b5*c3*c4 - d4*m6*y_b6*c4*c6 - d4*d6*m6*s4*s5 + a2*m4*x_b4*c3*s4 + d4*m5*x_b5*c5*s4 - d4*m6*x_b6*c4*s6 + d6*m6*x_b6*c6*s4 - m5*y_b5*z_b5*c4*c5 - d4*m5*y_b5*s4*s5 - d6*m6*y_b6*s4*s6 - d4*m6*z_b6*s4*s5 - m6*x_b6*y_b6*c4*s5 - m5*x_b5*z_b5*c4*s5 + m6*x_b6*z_b6*c6*s4 - m6*y_b6*z_b6*s4*s6 + 2*m5*x_b5*y_b5*pow(c5, 2)*s4 + m6*pow(x_b6, 2)*c4*c6*s5*s6 - m6*pow(y_b6, 2)*c4*c6*s5*s6 - a2*m6*y_b6*c3*c4*c6 - d6*m6*y_b6*c4*c5*c6 - a2*d6*m6*c3*s4*s5 + a2*m5*x_b5*c3*c5*s4 - a2*m6*x_b6*c3*c4*s6 - d4*m6*x_b6*c5*c6*s4 - d6*m6*x_b6*c4*c5*s6 - m6*y_b6*z_b6*c4*c5*c6 - a2*m5*y_b5*c3*s4*s5 - a2*m6*z_b6*c3*s4*s5 + d4*m6*y_b6*c5*s4*s6 - 2*d6*m6*z_b6*c5*s4*s5 - m6*x_b6*z_b6*c4*c5*s6 + m6*pow(x_b6, 2)*c5*pow(c6, 2)*s4*s5 - m6*pow(y_b6, 2)*c5*pow(c6, 2)*s4*s5 - 2*d6*m6*x_b6*pow(c5, 2)*c6*s4 + 2*d6*m6*y_b6*pow(c5, 2)*s4*s6 + 2*m6*x_b6*y_b6*c4*pow(c6, 2)*s5 - 2*m6*x_b6*z_b6*pow(c5, 2)*c6*s4 + 2*m6*y_b6*z_b6*pow(c5, 2)*s4*s6 - a2*m6*x_b6*c3*c5*c6*s4 + a2*m6*y_b6*c3*c5*s4*s6 - 2*m6*x_b6*y_b6*c5*c6*s4*s5*s6;
	mass_matrix(1,4) = I_xx[5]*c4 + I_xx[6]*c4 + pow(d6, 2)*m6*c4 + m5*pow(x_b5, 2)*c4 + m5*pow(y_b5, 2)*c4 + m6*pow(y_b6, 2)*c4 + m6*pow(z_b6, 2)*c4 - I_xx[5]*pow(c1, 2)*c4 - I_xx[6]*pow(c1, 2)*c4 + I_yy[5]*pow(c1, 2)*c4 + I_yy[6]*pow(c1, 2)*c4 - a3*d6*m6*s5 + a3*m5*x_b5*c5 + 2*d6*m6*z_b6*c4 - a3*m5*y_b5*s5 - a3*m6*z_b6*s5 + m6*pow(x_b6, 2)*c4*pow(c6, 2) - m6*pow(y_b6, 2)*c4*pow(c6, 2) + d4*d6*m6*c4*c5 - a3*m6*x_b6*c5*c6 + d4*m5*y_b5*c4*c5 + d4*m6*z_b6*c4*c5 - a2*d6*m6*s3*s5 + a2*m5*x_b5*c5*s3 + a3*m6*y_b6*c5*s6 + d4*m5*x_b5*c4*s5 - a2*m5*y_b5*s3*s5 - a2*m6*z_b6*s3*s5 + m6*x_b6*y_b6*c5*s4 + m5*x_b5*z_b5*c5*s4 - m5*y_b5*z_b5*s4*s5 - I_xx[5]*c1*c2*s1*s3*s4 - I_xx[5]*c1*c3*s1*s2*s4 - I_xx[6]*c1*c2*s1*s3*s4 - I_xx[6]*c1*c3*s1*s2*s4 + I_yy[5]*c1*c2*s1*s3*s4 + I_yy[5]*c1*c3*s1*s2*s4 + I_yy[6]*c1*c2*s1*s3*s4 + I_yy[6]*c1*c3*s1*s2*s4 + a2*d6*m6*c3*c4*c5 - m6*pow(x_b6, 2)*c5*c6*s4*s6 + m6*pow(y_b6, 2)*c5*c6*s4*s6 + a2*m5*y_b5*c3*c4*c5 + a2*m6*z_b6*c3*c4*c5 + a2*m5*x_b5*c3*c4*s5 - a2*m6*x_b6*c5*c6*s3 - d4*m6*x_b6*c4*c6*s5 + a2*m6*y_b6*c5*s3*s6 + d4*m6*y_b6*c4*s5*s6 - d6*m6*y_b6*c6*s4*s5 - 2*m6*x_b6*y_b6*c4*c6*s6 - d6*m6*x_b6*s4*s5*s6 - m6*y_b6*z_b6*c6*s4*s5 - m6*x_b6*z_b6*s4*s5*s6 - 2*m6*x_b6*y_b6*c5*pow(c6, 2)*s4 - a2*m6*x_b6*c3*c4*c6*s5 + a2*m6*y_b6*c3*c4*s5*s6;
	mass_matrix(1,5) = I_xx[6]*s4*s5 + m6*pow(x_b6, 2)*s4*s5 + m6*pow(y_b6, 2)*s4*s5 - I_xx[6]*pow(c1, 2)*s4*s5 + I_yy[6]*pow(c1, 2)*s4*s5 - d6*m6*y_b6*c4*c6 + a3*m6*y_b6*c6*s5 - d4*m6*x_b6*c6*s4 - d6*m6*x_b6*c4*s6 - m6*y_b6*z_b6*c4*c6 + a3*m6*x_b6*s5*s6 + d4*m6*y_b6*s4*s6 - m6*x_b6*z_b6*c4*s6 + I_xx[6]*c1*c5*s1*s2*s3 - I_yy[6]*c1*c5*s1*s2*s3 - d4*m6*y_b6*c4*c5*c6 - a2*m6*x_b6*c3*c6*s4 - d4*m6*x_b6*c4*c5*s6 - d6*m6*x_b6*c5*c6*s4 + a2*m6*y_b6*c3*s4*s6 + a2*m6*y_b6*c6*s3*s5 + d6*m6*y_b6*c5*s4*s6 - m6*x_b6*z_b6*c5*c6*s4 + a2*m6*x_b6*s3*s5*s6 + m6*y_b6*z_b6*c5*s4*s6 - I_xx[6]*c1*c2*c3*c5*s1 + I_yy[6]*c1*c2*c3*c5*s1 - a2*m6*y_b6*c3*c4*c5*c6 - a2*m6*x_b6*c3*c4*c5*s6 + I_xx[6]*c1*c2*c4*s1*s3*s5 + I_xx[6]*c1*c3*c4*s1*s2*s5 - I_yy[6]*c1*c2*c4*s1*s3*s5 - I_yy[6]*c1*c3*c4*s1*s2*s5;

	mass_matrix(2,2) = m5*pow(x_b5*(s5*(c2*(-c3) + s3*s2) - c4*c5*(c2*s3 - s2*(-c3))) + y_b5*(c5*(c2*(-c3) + s3*s2) + c4*s5*(c2*s3 - s2*(-c3))) - a3*(c2*s3 - s2*(-c3)) + d4*(c2*(-c3) + s3*s2) - z_b5*s4*(c2*s3 - s2*(-c3)), 2) + m4*pow(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + z_b4*(c2*s3*s1 - s1*s2*(-c3)) + x_b4*c4*(c2*s1*(-c3) + s3*s1*s2) - y_b4*s4*(c2*s1*(-c3) + s3*s1*s2), 2) + m3*pow(x_b3*(c1*c2*(-c3) + c1*s3*s2) - y_b3*(c1*s2*(-c3) - c1*c2*s3), 2) + m6*pow(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + d6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) - x_b6*(c6*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) - s4*s6*(c2*s1*(-c3) + s3*s1*s2)) + y_b6*(s6*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) + c6*s4*(c2*s1*(-c3) + s3*s1*s2)), 2) + m3*pow(x_b3*(c2*s1*(-c3) + s3*s1*s2) + y_b3*(c2*s3*s1 - s1*s2*(-c3)), 2) + m4*pow(d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + z_b4*(c1*s2*(-c3) - c1*c2*s3) - x_b4*c4*(c1*c2*(-c3) + c1*s3*s2) + y_b4*s4*(c1*c2*(-c3) + c1*s3*s2), 2) + m6*pow(z_b6*(c5*(c2*(-c3) + s3*s2) + c4*s5*(c2*s3 - s2*(-c3))) - a3*(c2*s3 - s2*(-c3)) + d4*(c2*(-c3) + s3*s2) - x_b6*(c6*(s5*(c2*(-c3) + s3*s2) - c4*c5*(c2*s3 - s2*(-c3))) + s4*s6*(c2*s3 - s2*(-c3))) + y_b6*(s6*(s5*(c2*(-c3) + s3*s2) - c4*c5*(c2*s3 - s2*(-c3))) - c6*s4*(c2*s3 - s2*(-c3))) + d6*(c5*(c2*(-c3) + s3*s2) + c4*s5*(c2*s3 - s2*(-c3))), 2) + c1*(I_yy[3]*c1 - I_xy[3]*s1) + m6*pow(d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + d6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) - x_b6*(c6*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) + s4*s6*(c1*c2*(-c3) + c1*s3*s2)) + y_b6*(s6*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) - c6*s4*(c1*c2*(-c3) + c1*s3*s2)), 2) + m5*pow(a3*(c2*s1*(-c3) + s3*s1*s2) + d4*(c2*s3*s1 - s1*s2*(-c3)) + x_b5*(s5*(c2*s3*s1 - s1*s2*(-c3)) + c4*c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(c5*(c2*s3*s1 - s1*s2*(-c3)) - c4*s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b5*s4*(c2*s1*(-c3) + s3*s1*s2), 2) + m3*pow(x_b3*(c2*s3 - s2*(-c3)) - y_b3*(c2*(-c3) + s3*s2), 2) - s1*(I_xy[3]*c1 - I_xx[3]*s1) + m5*pow(d4*(c1*s2*(-c3) - c1*c2*s3) - a3*(c1*c2*(-c3) + c1*s3*s2) + x_b5*(s5*(c1*s2*(-c3) - c1*c2*s3) - c4*c5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(c5*(c1*s2*(-c3) - c1*c2*s3) + c4*s5*(c1*c2*(-c3) + c1*s3*s2)) - z_b5*s4*(c1*c2*(-c3) + c1*s3*s2), 2) + I_yy[4]*pow(c1, 2)+ I_yy[5]*pow(c1, 2)+ I_yy[6]*pow(c1, 2)+ I_xx[4]*pow(s1, 2)+ I_xx[5]*pow(s1, 2)+ I_xx[6]*pow(s1, 2)+ m4*pow(d4*(c2*(-c3) + s3*s2) - a3*(c2*s3 - s2*(-c3)) + z_b4*(c2*(-c3) + s3*s2) - x_b4*c4*(c2*s3 - s2*(-c3)) + y_b4*s4*(c2*s3 - s2*(-c3)), 2);
	mass_matrix(2,3) = d4*m4*y_b4*c4 - d4*m5*z_b5*c4 + d4*m4*x_b4*s4 + m4*y_b4*z_b4*c4 - m5*x_b5*y_b5*s4 + m4*x_b4*z_b4*s4 - I_xx[4]*c1*c2*c3*s1 - I_xx[5]*c1*c2*c3*s1 + I_yy[4]*c1*c2*c3*s1 - I_xx[6]*c1*c2*c3*s1 + I_yy[5]*c1*c2*c3*s1 + I_yy[6]*c1*c2*c3*s1 + I_xx[4]*c1*s1*s2*s3 + I_xx[5]*c1*s1*s2*s3 - I_yy[4]*c1*s1*s2*s3 + I_xx[6]*c1*s1*s2*s3 - I_yy[5]*c1*s1*s2*s3 - I_yy[6]*c1*s1*s2*s3 - pow(d6, 2)*m6*c5*s4*s5 + m5*pow(x_b5, 2)*c5*s4*s5 - m5*pow(y_b5, 2)*c5*s4*s5 + m6*pow(y_b6, 2)*c5*s4*s5 - m6*pow(z_b6, 2)*c5*s4*s5 - d4*m6*y_b6*c4*c6 - d4*d6*m6*s4*s5 + d4*m5*x_b5*c5*s4 - d4*m6*x_b6*c4*s6 + d6*m6*x_b6*c6*s4 - m5*y_b5*z_b5*c4*c5 - d4*m5*y_b5*s4*s5 - d6*m6*y_b6*s4*s6 - d4*m6*z_b6*s4*s5 - m6*x_b6*y_b6*c4*s5 - m5*x_b5*z_b5*c4*s5 + m6*x_b6*z_b6*c6*s4 - m6*y_b6*z_b6*s4*s6 + 2*m5*x_b5*y_b5*pow(c5, 2)*s4 + m6*pow(x_b6, 2)*c4*c6*s5*s6 - m6*pow(y_b6, 2)*c4*c6*s5*s6 - d6*m6*y_b6*c4*c5*c6 - d4*m6*x_b6*c5*c6*s4 - d6*m6*x_b6*c4*c5*s6 - m6*y_b6*z_b6*c4*c5*c6 + d4*m6*y_b6*c5*s4*s6 - 2*d6*m6*z_b6*c5*s4*s5 - m6*x_b6*z_b6*c4*c5*s6 + m6*pow(x_b6, 2)*c5*pow(c6, 2)*s4*s5 - m6*pow(y_b6, 2)*c5*pow(c6, 2)*s4*s5 - 2*d6*m6*x_b6*pow(c5, 2)*c6*s4 + 2*d6*m6*y_b6*pow(c5, 2)*s4*s6 + 2*m6*x_b6*y_b6*c4*pow(c6, 2)*s5 - 2*m6*x_b6*z_b6*pow(c5, 2)*c6*s4 + 2*m6*y_b6*z_b6*pow(c5, 2)*s4*s6 - 2*m6*x_b6*y_b6*c5*c6*s4*s5*s6;
	mass_matrix(2,4) = I_xx[5]*c4 + I_xx[6]*c4 + pow(d6, 2)*m6*c4 + m5*pow(x_b5, 2)*c4 + m5*pow(y_b5, 2)*c4 + m6*pow(y_b6, 2)*c4 + m6*pow(z_b6, 2)*c4 - I_xx[5]*pow(c1, 2)*c4 - I_xx[6]*pow(c1, 2)*c4 + I_yy[5]*pow(c1, 2)*c4 + I_yy[6]*pow(c1, 2)*c4 - a3*d6*m6*s5 + a3*m5*x_b5*c5 + 2*d6*m6*z_b6*c4 - a3*m5*y_b5*s5 - a3*m6*z_b6*s5 + m6*pow(x_b6, 2)*c4*pow(c6, 2) - m6*pow(y_b6, 2)*c4*pow(c6, 2) + d4*d6*m6*c4*c5 - a3*m6*x_b6*c5*c6 + d4*m5*y_b5*c4*c5 + d4*m6*z_b6*c4*c5 + a3*m6*y_b6*c5*s6 + d4*m5*x_b5*c4*s5 + m6*x_b6*y_b6*c5*s4 + m5*x_b5*z_b5*c5*s4 - m5*y_b5*z_b5*s4*s5 - I_xx[5]*c1*c2*s1*s3*s4 - I_xx[5]*c1*c3*s1*s2*s4 - I_xx[6]*c1*c2*s1*s3*s4 - I_xx[6]*c1*c3*s1*s2*s4 + I_yy[5]*c1*c2*s1*s3*s4 + I_yy[5]*c1*c3*s1*s2*s4 + I_yy[6]*c1*c2*s1*s3*s4 + I_yy[6]*c1*c3*s1*s2*s4 - m6*pow(x_b6, 2)*c5*c6*s4*s6 + m6*pow(y_b6, 2)*c5*c6*s4*s6 - d4*m6*x_b6*c4*c6*s5 + d4*m6*y_b6*c4*s5*s6 - d6*m6*y_b6*c6*s4*s5 - 2*m6*x_b6*y_b6*c4*c6*s6 - d6*m6*x_b6*s4*s5*s6 - m6*y_b6*z_b6*c6*s4*s5 - m6*x_b6*z_b6*s4*s5*s6 - 2*m6*x_b6*y_b6*c5*pow(c6, 2)*s4;
	mass_matrix(2,5) = I_xx[6]*s4*s5 + m6*pow(x_b6, 2)*s4*s5 + m6*pow(y_b6, 2)*s4*s5 - I_xx[6]*pow(c1, 2)*s4*s5 + I_yy[6]*pow(c1, 2)*s4*s5 - d6*m6*y_b6*c4*c6 + a3*m6*y_b6*c6*s5 - d4*m6*x_b6*c6*s4 - d6*m6*x_b6*c4*s6 - m6*y_b6*z_b6*c4*c6 + a3*m6*x_b6*s5*s6 + d4*m6*y_b6*s4*s6 - m6*x_b6*z_b6*c4*s6 + I_xx[6]*c1*c5*s1*s2*s3 - I_yy[6]*c1*c5*s1*s2*s3 - d4*m6*y_b6*c4*c5*c6 - d4*m6*x_b6*c4*c5*s6 - d6*m6*x_b6*c5*c6*s4 + d6*m6*y_b6*c5*s4*s6 - m6*x_b6*z_b6*c5*c6*s4 + m6*y_b6*z_b6*c5*s4*s6 - I_xx[6]*c1*c2*c3*c5*s1 + I_yy[6]*c1*c2*c3*c5*s1 + I_xx[6]*c1*c2*c4*s1*s3*s5 + I_xx[6]*c1*c3*c4*s1*s2*s5 - I_yy[6]*c1*c2*c4*s1*s3*s5 - I_yy[6]*c1*c3*c4*s1*s2*s5;

	mass_matrix(3,3) = m6*pow(x_b6*(s6*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - c5*c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3))) + y_b6*(c6*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3))) - d6*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - z_b6*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)), 2) + m5*pow(z_b5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + x_b5*c5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - y_b5*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)), 2) + m4*pow(y_b4*c4*(c2*(-c3) + s3*s2) + x_b4*s4*(c2*(-c3) + s3*s2), 2) + I_xx[4]*pow(c1*c2*(-c3) + c1*s3*s2, 2) + I_xx[5]*pow(c1*c2*(-c3) + c1*s3*s2, 2) + I_xx[6]*pow(c1*c2*(-c3) + c1*s3*s2, 2) + m6*pow(x_b6*(s6*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)))) + y_b6*(c6*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + c5*s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)))) - d6*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - z_b6*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))), 2) + m5*pow(z_b5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + x_b5*c5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - y_b5*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))), 2) + I_yy[4]*pow(c2*s1*(-c3) + s3*s1*s2, 2) + I_yy[5]*pow(c2*s1*(-c3) + s3*s1*s2, 2) + I_yy[6]*pow(c2*s1*(-c3) + s3*s1*s2, 2) + m4*pow(x_b4*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - y_b4*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)), 2) + m5*pow(z_b5*c4*(c2*(-c3) + s3*s2) - x_b5*c5*s4*(c2*(-c3) + s3*s2) + y_b5*s4*s5*(c2*(-c3) + s3*s2), 2) + m4*pow(x_b4*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - y_b4*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))), 2) + m6*pow(x_b6*(c4*s6*(c2*(-c3) + s3*s2) + c5*c6*s4*(c2*(-c3) + s3*s2)) + y_b6*(c4*c6*(c2*(-c3) + s3*s2) - c5*s4*s6*(c2*(-c3) + s3*s2)) + d6*s4*s5*(c2*(-c3) + s3*s2) + z_b6*s4*s5*(c2*(-c3) + s3*s2), 2) + I_zz[4]*pow(c2*s3 - s2*(-c3), 2) + I_zz[5]*pow(c2*s3 - s2*(-c3), 2) + I_zz[6]*pow(c2*s3 - s2*(-c3), 2);
	mass_matrix(3,4) = I_zz[5]*c2*s2*s4 - I_yy[6]*c2*s2*s4 - I_yy[5]*c3*s3*s4 - I_yy[5]*c2*s2*s4 - I_yy[6]*c3*s3*s4 + I_zz[6]*c2*s2*s4 + I_zz[5]*c3*s3*s4 + I_zz[6]*c3*s3*s4 - m5*y_b5*z_b5*c5 - m6*x_b6*y_b6*s5 - m5*x_b5*z_b5*s5 + m6*pow(x_b6, 2)*c6*s5*s6 - m6*pow(y_b6, 2)*c6*s5*s6 - d6*m6*y_b6*c5*c6 - d6*m6*x_b6*c5*s6 - m6*y_b6*z_b6*c5*c6 - m6*x_b6*z_b6*c5*s6 - I_xx[5]*pow(c1, 2)*c2*s2*s4 - I_xx[6]*pow(c1, 2)*c2*s2*s4 - I_xx[5]*pow(c1, 2)*c3*s3*s4 + I_yy[5]*pow(c1, 2)*c2*s2*s4 - I_xx[6]*pow(c1, 2)*c3*s3*s4 + I_yy[6]*pow(c1, 2)*c2*s2*s4 + 2*I_yy[5]*c2*pow(c3, 2)*s2*s4 + I_yy[5]*pow(c1, 2)*c3*s3*s4 + 2*I_yy[5]*pow(c2, 2)*c3*s3*s4 + 2*I_yy[6]*c2*pow(c3, 2)*s2*s4 + I_yy[6]*pow(c1, 2)*c3*s3*s4 + 2*I_yy[6]*pow(c2, 2)*c3*s3*s4 - 2*I_zz[5]*c2*pow(c3, 2)*s2*s4 - 2*I_zz[5]*pow(c2, 2)*c3*s3*s4 - 2*I_zz[6]*c2*pow(c3, 2)*s2*s4 - 2*I_zz[6]*pow(c2, 2)*c3*s3*s4 + 2*m6*x_b6*y_b6*pow(c6, 2)*s5 + I_xx[5]*c1*c4*s1*s2*s3 + I_xx[6]*c1*c4*s1*s2*s3 - I_yy[5]*c1*c4*s1*s2*s3 - I_yy[6]*c1*c4*s1*s2*s3 + 2*I_xx[5]*pow(c1, 2)*c2*pow(c3, 2)*s2*s4 + 2*I_xx[5]*pow(c1, 2)*pow(c2, 2)*c3*s3*s4 + 2*I_xx[6]*pow(c1, 2)*c2*pow(c3, 2)*s2*s4 + 2*I_xx[6]*pow(c1, 2)*pow(c2, 2)*c3*s3*s4 - 2*I_yy[5]*pow(c1, 2)*c2*pow(c3, 2)*s2*s4 - 2*I_yy[5]*pow(c1, 2)*pow(c2, 2)*c3*s3*s4 - 2*I_yy[6]*pow(c1, 2)*c2*pow(c3, 2)*s2*s4 - 2*I_yy[6]*pow(c1, 2)*pow(c2, 2)*c3*s3*s4 - I_xx[5]*c1*c2*c3*c4*s1 - I_xx[6]*c1*c2*c3*c4*s1 + I_yy[5]*c1*c2*c3*c4*s1 + I_yy[6]*c1*c2*c3*c4*s1;
	mass_matrix(3,5) = I_xx[6]*(c1*c2*(-c3) + c1*s3*s2)*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + I_zz[6]*(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2))*(c2*s3 - s2*(-c3)) - m6*(x_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) - y_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))))*(x_b6*(s6*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)))) + y_b6*(c6*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + c5*s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)))) - d6*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - z_b6*s5*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)))) + m6*(x_b6*(s6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) + c6*s4*(c2*(-c3) + s3*s2)) + y_b6*(c6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) - s4*s6*(c2*(-c3) + s3*s2)))*(x_b6*(c4*s6*(c2*(-c3) + s3*s2) + c5*c6*s4*(c2*(-c3) + s3*s2)) + y_b6*(c4*c6*(c2*(-c3) + s3*s2) - c5*s4*s6*(c2*(-c3) + s3*s2)) + d6*s4*s5*(c2*(-c3) + s3*s2) + z_b6*s4*s5*(c2*(-c3) + s3*s2)) - m6*(x_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - y_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))))*(x_b6*(s6*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - c5*c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3))) + y_b6*(c6*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3))) - d6*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - z_b6*s5*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3))) - I_yy[6]*(c2*s1*(-c3) + s3*s1*s2)*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2));

	mass_matrix(4,4) = m5*pow(x_b5*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + y_b5*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)), 2) + m6*pow(d6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) - x_b6*c6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b6*s6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)), 2) + I_xx[5]*pow(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3), 2) + I_xx[6]*pow(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3), 2) + m6*pow(z_b6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) + d6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) + x_b6*c6*(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2)) - y_b6*s6*(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2)), 2) + I_yy[5]*pow(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)), 2) + I_yy[6]*pow(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)), 2) + m6*pow(d6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) - x_b6*c6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + y_b6*s6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)), 2) + m5*pow(x_b5*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b5*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)), 2) + m5*pow(x_b5*(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2)) - y_b5*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)), 2) + I_zz[5]*pow(s4, 2)*pow(c2*(-c3) + s3*s2, 2) + I_zz[6]*pow(s4, 2)*pow(c2*(-c3) + s3*s2, 2);
	mass_matrix(4,5) = m6*(x_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - y_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))))*(d6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) + z_b6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2)) - x_b6*c6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + y_b6*s6*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2))) - m6*(x_b6*(s6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) + c6*s4*(c2*(-c3) + s3*s2)) + y_b6*(c6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) - s4*s6*(c2*(-c3) + s3*s2)))*(z_b6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) + d6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) + x_b6*c6*(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2)) - y_b6*s6*(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2))) + I_xx[6]*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3))*(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2)) + m6*(x_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) - y_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))))*(d6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) + z_b6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2)) - x_b6*c6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + y_b6*s6*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2))) + I_yy[6]*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3)))*(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2)) + I_zz[6]*s4*(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2))*(c2*(-c3) + s3*s2);

	mass_matrix(5,5) = I_zz[6]*pow(c5*(c2*s3 - s2*(-c3)) - c4*s5*(c2*(-c3) + s3*s2), 2) + m6*pow(x_b6*(c6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) - s6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))) - y_b6*(s6*(c4*s1 + s4*(c1*s2*(-c3) - c1*c2*s3)) + c6*(c5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) - s5*(c1*c2*(-c3) + c1*s3*s2))), 2) + I_xx[6]*pow(s5*(s1*s4 - c4*(c1*s2*(-c3) - c1*c2*s3)) + c5*(c1*c2*(-c3) + c1*s3*s2), 2) + m6*pow(x_b6*(s6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) + c6*s4*(c2*(-c3) + s3*s2)) + y_b6*(c6*(s5*(c2*s3 - s2*(-c3)) + c4*c5*(c2*(-c3) + s3*s2)) - s4*s6*(c2*(-c3) + s3*s2)), 2) + I_yy[6]*pow(s5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) - c5*(c2*s1*(-c3) + s3*s1*s2), 2) + m6*pow(x_b6*(c6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) - s6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))) - y_b6*(s6*(c1*c4 + s4*(c2*s3*s1 - s1*s2*(-c3))) + c6*(c5*(c1*s4 - c4*(c2*s3*s1 - s1*s2*(-c3))) + s5*(c2*s1*(-c3) + s3*s1*s2))), 2);
	
	for(int i = 0; i < 6; i++) {
		for(int j = i+1; j < 6; j++) {
			mass_matrix(j, i) = mass_matrix(i, j);
		}
	}
}


void ArmControl::GravityTorque(double* currentpos, double* gravity_torque)
{
	double  s2, s23, s4, s5, s6,
			c2, c23, c4, c5, c6;
	s2 = sin(currentpos[1]); s23 = sin(currentpos[1] + currentpos[2]); s4 = sin(currentpos[3]); s5 = sin(currentpos[4]); s6 = sin(currentpos[5]);
	c2 = cos(currentpos[1]); c23 = cos(currentpos[1] + currentpos[2]); c4 = cos(currentpos[3]); c5 = cos(currentpos[4]); c6 = cos(currentpos[5]);
	
	gravity_torque[0] = 0;
	gravity_torque[1] = - g*m2*(x_b2*c2 - y_b2*s2) - g*m3*(y_b3*c23 + x_b3*s23 + a2*c2) - g*m5*(d4*c23 + a3*s23 + a2*c2 + x_b5*(c23*s5 + s23*c4*c5) + y_b5*(c23*c5 - s23*c4*s5) + z_b5*s23*s4) - g*m4*(d4*c23 + a3*s23 + z_b4*c23 + a2*c2 + x_b4*s23*c4 - y_b4*s23*s4) - g*m6*(d4*c23 - x_b6*(c6*(c23*s5 + s23*c4*c5) - s23*s4*s6) + y_b6*(s6*(c23*s5 + s23*c4*c5) + s23*c6*s4) + a3*s23 + a2*c2 + d6*(c23*c5 - s23*c4*s5) + z_b6*(c23*c5 - s23*c4*s5));
	gravity_torque[2] = - g*m6*(d4*c23 - x_b6*(c6*(c23*s5 + s23*c4*c5) - s23*s4*s6) + y_b6*(s6*(c23*s5 + s23*c4*c5) + s23*c6*s4) + a3*s23 + d6*(c23*c5 - s23*c4*s5) + z_b6*(c23*c5 - s23*c4*s5)) - g*m3*(y_b3*c23 + x_b3*s23) - g*m5*(d4*c23 + a3*s23 + x_b5*(c23*s5 + s23*c4*c5) + y_b5*(c23*c5 - s23*c4*s5) + z_b5*s23*s4) - g*m4*(d4*c23 + a3*s23 + z_b4*c23 + x_b4*s23*c4 - y_b4*s23*s4);
	gravity_torque[3] = g*m6*(x_b6*(c23*c4*s6 + c23*c5*c6*s4) + y_b6*(c23*c4*c6 - c23*c5*s4*s6) + d6*c23*s4*s5 + z_b6*c23*s4*s5) - g*m4*(y_b4*c23*c4 + x_b4*c23*s4) + g*m5*(z_b5*c23*c4 - x_b5*c23*c5*s4 + y_b5*c23*s4*s5);
	gravity_torque[4] = g*m6*(d6*(s23*s5 - c23*c4*c5) + z_b6*(s23*s5 - c23*c4*c5) + x_b6*c6*(s23*c5 + c23*c4*s5) - y_b6*s6*(s23*c5 + c23*c4*s5)) - g*m5*(x_b5*(s23*c5 + c23*c4*s5) - y_b5*(s23*s5 - c23*c4*c5));
	gravity_torque[5] = - g*m6*(x_b6*(s6*(s23*s5 - c23*c4*c5) - c23*c6*s4) + y_b6*(c6*(s23*s5 - c23*c4*c5) + c23*s4*s6));
}

////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------Trajectory Planning----------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::JointTrajectory(double* th_ini, double* th_cmd, MatrixXd& th_out)
{
	// th_out row: 6dof / column: interpolated position
	double vel_des = PI/2; // rad/s
	double max_th_error = 0;
	for (int i = 0; i < DoF; i++){
		if(max_th_error < fabs(th_cmd[i] - th_ini[i]))
			max_th_error = fabs(th_cmd[i] - th_ini[i]);
	}
	
	double Tf = max_th_error / vel_des; // Use maximum error of angle
	double step = round(Tf / SAMPLING_TIME_TRAJ);
	th_out.resize(step, th_out.cols());

	for (int i = 0; i < DoF; i++)
	{
		RowVectorXd inter_pos = RowVectorXd::LinSpaced(step, th_ini[i], th_cmd[i]);
		th_out.block(0,i,step,1) = inter_pos.transpose();
	}
}

void ArmControl::JointTrajectoryTrapezoidal(double* th_ini, double* th_cmd, MatrixXd& th_out, MatrixXd& vel_out)
{
	double vel_des = PI/2; // PI/2; // rad/s
	double acc_des = 4*PI;

	double max_th_error = 0;
	for (int i = 0; i < DoF; i++){
		if(max_th_error < fabs(th_cmd[i] - th_ini[i]))
			max_th_error = fabs(th_cmd[i] - th_ini[i]);
	}

	double Tf = max_th_error / vel_des; // Whole time
	double Tb; // blending time
	if (acc_des >= 4*max_th_error/pow(Tf,2)) {
		Tb = Tf/2 - sqrt(pow(acc_des*Tf,2) - 4*acc_des*max_th_error) / (2*acc_des);
	}
	else {
		Tb = Tf/2;
	}
	int lin_step = round((Tf - 2*Tb) / SAMPLING_TIME_TRAJ);
	int acc_step = round(Tb / SAMPLING_TIME_TRAJ);
	int step = lin_step + 2*acc_step + 1;
	th_out.resize(step, th_out.cols());
	vel_out.resize(step, vel_out.cols());

	for (int i = 0; i < DoF; i++)
	{
		double vel_lin = (th_cmd[i] - th_ini[i])/(Tf-Tb);
		double acc_joint = vel_lin/Tb;
		for (int j = 0; j < step; j++)
		{
			if (j <= acc_step) {
				th_out(j, i) = th_ini[i] + 0.5*acc_joint*pow(j*SAMPLING_TIME_TRAJ,2);
			}
			else if (j <= acc_step + lin_step) {
				th_out(j, i) = th_out(acc_step, i) + vel_lin*((j-acc_step)*SAMPLING_TIME_TRAJ);
			}
			else {
				th_out(j, i) = -0.5*acc_joint*pow(j*SAMPLING_TIME_TRAJ - Tf, 2) + th_cmd[i];
			}
			if (j > 0) {
				vel_out(j,i) = (th_out(j,i) - th_out(j-1,i))/SAMPLING_TIME_TRAJ;
			}
		}
	}
}

void ArmControl::CartesianTrajectoryEuler(VectorXd& pose_ini, VectorXd& pose_cmd, MatrixXd& pose_out) // , MatrixXd& vel_out)
{
	// pose_out row: 6dof / column: interpolated position
	double vel_des = 0.3; // m/s
	double xyz_distance = sqrt(pow(pose_ini(0)-pose_cmd(0), 2) + pow(pose_ini(1)-pose_cmd(1), 2) + pow(pose_ini(2)-pose_cmd(2), 2));
	
	double Tf = xyz_distance / vel_des; // Use maximum error of angle, add orientation error later
	double step = round(Tf / SAMPLING_TIME_TRAJ);
	pose_out.resize(step, pose_out.cols());

	for (int i = 0; i < DoF; i++)
	{
		RowVectorXd inter_pos = RowVectorXd::LinSpaced(step, pose_ini(i), pose_cmd(i));
		pose_out.block(0,i,step,1) = inter_pos.transpose();
		// double vel_ = (pose_cmd(i) - pose_ini(i))/Tf;
		// vel_out.block(0, i, step, 1) = VectorXd::Constant(step, vel_);
	}
}

void ArmControl::CartesianTrajectoryQuat(VectorXd& pose_quat_ini, VectorXd& pose_quat_cmd, MatrixXd& pose_quat_out)
{
	// pose_out row: 7dof(xyzqwqxqyqz) / column: interpolated position
	Quaterniond q_ini, q_cmd, q_slerp;
	q_ini.w() = pose_quat_ini(3); q_cmd.w() = pose_quat_cmd(3); 
	q_ini.x() = pose_quat_ini(4); q_cmd.x() = pose_quat_cmd(4);
	q_ini.y() = pose_quat_ini(5); q_cmd.y() = pose_quat_cmd(5);
	q_ini.z() = pose_quat_ini(6); q_cmd.z() = pose_quat_cmd(6);
	
	double vel_des = 0.4; // m/s
	double angvel_des = PI/4; // rad/s
	double xyz_distance = sqrt(pow(pose_quat_ini(0)-pose_quat_cmd(0), 2) + pow(pose_quat_ini(1)-pose_quat_cmd(1), 2) + pow(pose_quat_ini(2)-pose_quat_cmd(2), 2));
	double quat_angle = abs(acos(q_ini.dot(q_cmd) / (q_ini.norm()*q_cmd.norm())));
	double t = 0;

	double Tf = max(xyz_distance/vel_des, quat_angle/angvel_des); // Use maximum error of angle, add orientation error later
	double step = round(Tf / SAMPLING_TIME_TRAJ);
	pose_quat_out.resize(step, pose_quat_out.cols());

	// for (int i = 0; i < 7; i++)
	// {
	// 	RowVectorXd inter_pos = RowVectorXd::LinSpaced(step, pose_quat_ini(i), pose_quat_cmd(i));
	// 	pose_quat_out.block(0,i,step,1) = inter_pos.transpose();
	// }

	for (int i = 0; i < 3; i++)
	{
		RowVectorXd inter_pos = RowVectorXd::LinSpaced(step, pose_quat_ini(i), pose_quat_cmd(i));
		pose_quat_out.block(0,i,step,1) = inter_pos.transpose();
	}
	for (int i = 0; i < step; i++)
	{
		if(step == 1)
			t = 0;
		else
			t = i/(step-1);
		q_slerp = q_ini.slerp(t, q_cmd);
		pose_quat_out(i, 3) = q_slerp.w();
		pose_quat_out(i, 4) = q_slerp.x();
		pose_quat_out(i, 5) = q_slerp.y();
		pose_quat_out(i, 6) = q_slerp.z();
	}
}

void ArmControl::CartesianTrajectoryTrapezoidal(VectorXd& pose_quat_ini, VectorXd& pose_quat_cmd, MatrixXd& pose_quat_out)
{
	// pose_out row: 7dof(xyzqwqxqyqz) / column: interpolated position

	Quaterniond q_ini, q_cmd, q_slerp;
	q_ini.w() = pose_quat_ini(3); q_cmd.w() = pose_quat_cmd(3); 
	q_ini.x() = pose_quat_ini(4); q_cmd.x() = pose_quat_cmd(4);
	q_ini.y() = pose_quat_ini(5); q_cmd.y() = pose_quat_cmd(5);
	q_ini.z() = pose_quat_ini(6); q_cmd.z() = pose_quat_cmd(6);
	
	double vel_des = 0.4; // m/s
	double angvel_des = PI/4; // rad/s
	double xyz_distance = sqrt(pow(pose_quat_ini(0)-pose_quat_cmd(0), 2) + pow(pose_quat_ini(1)-pose_quat_cmd(1), 2) + pow(pose_quat_ini(2)-pose_quat_cmd(2), 2));
	double quat_angle = abs(acos(q_ini.dot(q_cmd) / (q_ini.norm()*q_cmd.norm())));

	double Tf = max(xyz_distance/vel_des, quat_angle/angvel_des); // Use maximum error of angle, add orientation error later
	double Tb; // blending time
	if (angvel_des >= 4/pow(Tf,2)) {
		Tb = Tf/2 - sqrt(pow(angvel_des*Tf,2) - 4*angvel_des) / (2*angvel_des);
	}
	else {
		Tb = Tf/2;
	}
	int lin_step = round((Tf - 2*Tb) / SAMPLING_TIME_TRAJ);
	int acc_step = round(Tb / SAMPLING_TIME_TRAJ);
	int step = lin_step + 2*acc_step + 1;
	double vel_lin = 1/(Tf-Tb);  
	double acc_joint = vel_lin/Tb;
	VectorXd S_out = VectorXd::Zero(step);
	for (int i = 0; i < step; i++)
		{
			if (i <= acc_step) {
				S_out(i) = 0.5*acc_joint*pow(i*SAMPLING_TIME_TRAJ,2); 
			}
			else if (i <= acc_step + lin_step) {
				S_out(i) = S_out(acc_step) + vel_lin*((i-acc_step)*SAMPLING_TIME_TRAJ);
			}
			else {
				S_out(i) = 1-0.5*acc_joint*pow(i*SAMPLING_TIME_TRAJ - Tf, 2);
			}
		}

	pose_quat_out.resize(step, pose_quat_out.cols());

	Lerp(S_out, pose_quat_ini, pose_quat_cmd, step, pose_quat_out);

	for (int i = 0; i < step; i++)
	{
		q_slerp = q_ini.slerp(S_out(i), q_cmd);
		pose_quat_out(i, 3) = q_slerp.w();
		pose_quat_out(i, 4) = q_slerp.x();
		pose_quat_out(i, 5) = q_slerp.y();
		pose_quat_out(i, 6) = q_slerp.z();
	}

	// Slerp(S_out, pose_quat_ini, pose_quat_cmd, step, quat_angle, pose_quat_out);
}

void ArmControl::Lerp(VectorXd& S_out, VectorXd& pose_quat_ini, VectorXd& pose_quat_cmd, int step, MatrixXd& pose_quat_out)
{
	for (int i = 0; i < step; i++){
		for(int j=0; j<3; j++){
		pose_quat_out(i, j) = (1-S_out(i))*pose_quat_ini(j)+S_out(i)*pose_quat_cmd(j);
		}
	}
}

// void ArmControl::Slerp(VectorXd& S_out, VectorXd& pose_quat_ini, VectorXd& pose_quat_cmd, int step, double quat_angle, MatrixXd& pose_quat_out)
// {
// 	for (int i = 0; i < step; i++){
// 		for(int j=3; j<7; j++){
// 		pose_quat_out(i, j) = sin(1-S_out(i))/sin(quat_angle)*pose_quat_ini(j) + sin(S_out(i))/sin(quat_angle)*pose_quat_cmd(j);
// 		}
// 	}
// }

////////////////////////////////////////////////////////////////////////////////////////////
//---------------------------------------Controller---------------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::PIDController(double* targetpos_, double* currentpos, double* currentvel_, double* PDtorque_)
{
	double error_old[DoF] = {0, };
    int torque_limit = 100;
    for (int i = 0; i < DoF; i++)
    {
        arm_errorp[i] = targetpos_[i] - currentpos[i];

        P_term[i] = Kp[i] * arm_errorp[i];
        I_term[i] += Ki[i] * arm_errorp[i];
        D_term[i] = Kd[i] * (0 - arm_jointv[i]);

        PDtorque_[i] = P_term[i] + D_term[i];

        if (PDtorque_[i] > torque_limit)
        {
            PDtorque_[i] = torque_limit;
        }
        else if (PDtorque_[i] < -torque_limit)
        {
            PDtorque_[i] = -torque_limit;
        }

        error_old[i] = arm_errorp[i];
    }
}


////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------Mobile Robot Trajectory Planning----------------------------------//
////////////////////////////////////////////////////////////////////////////////////////////
void ArmControl::MobileTrajectory(double* Mobile_ini, double* Mobile_cmd, MatrixXd& Mobile_out)
{
    double vel_des = 1; // m/s

	double delta_X = Mobile_cmd[0] - Mobile_ini[0];
	double delta_Y = Mobile_cmd[1] - Mobile_ini[1];

    double diff_distance = fabs(sqrt(pow((delta_X),2)+pow((delta_Y),2)));
	
    double Tf = diff_distance / vel_des;
    int step = round(Tf / SAMPLING_TIME_TRAJ);

	if(step ==0){
		step = 1;
	}

    Mobile_out.resize(step,Mobile_out.cols());

    RowVectorXd inter_pos_x = RowVectorXd::LinSpaced(step, Mobile_ini[0], Mobile_cmd[0]);
    RowVectorXd inter_pos_y = RowVectorXd::LinSpaced(step, Mobile_ini[1], Mobile_cmd[1]);
    Mobile_out.block(0,0,step,1) = inter_pos_x.transpose();
    Mobile_out.block(0,1,step,1) = inter_pos_y.transpose();
}

void ArmControl::MobileTrajectory_Z(double* Mobile_ini, double* Mobile_cmd, MatrixXd& Mobile_out)
{
	double angvel_des = PI;

	double delta_X = Mobile_cmd[0] - Mobile_ini[0];
	double delta_Y = Mobile_cmd[1] - Mobile_ini[1];

	if(delta_X == 0 && delta_Y == 0){
		Mobile_out.row(0) = RowVector3d({Mobile_ini[0], Mobile_ini[1], Mobile_ini[2]});
	}
	
	else{
	double diff_angle = atan2(delta_Y, delta_X);
	
    double Tf = fabs(diff_angle / angvel_des);

    int step = round(Tf / SAMPLING_TIME_TRAJ);

	if(step ==0){
		step = 1;
	}

    Mobile_out.resize(step,Mobile_out.cols());

	RowVectorXd inter_pos_z = RowVectorXd::LinSpaced(step, Mobile_ini[2], diff_angle);
	Mobile_out.block(0,2,step,1) = inter_pos_z.transpose();
	}
}
