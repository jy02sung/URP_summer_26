#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <array>
#include <algorithm>
#include <map>
#include <memory>

namespace {
constexpr int N=15;
const std::array<std::string,N> names={{"Waist_joint","Head_yaw_joint","Head_pitch_joint","L_shoulder_pitch_joint","L_shoulder_roll_joint","L_shoulder_yaw_joint","L_elbow_joint","L_wrist_yaw_joint","L_wrist_pitch_joint","R_shoulder_pitch_joint","R_shoulder_roll_joint","R_shoulder_yaw_joint","R_elbow_joint","R_wrist_yaw_joint","R_wrist_pitch_joint"}};
const std::array<int,N> ctrl_to_pin={{0,3,4,5,6,7,8,9,10,11,12,13,14,1,2}};
const std::array<double,N> kp={{10,1,5,10,10,10,10,2,2,10,10,10,10,2,2}};
const std::array<double,N> kd={{3,.2,1.5,4,4,4,4,.8,.8,4,4,4,4,.8,.8}};
const std::array<double,N> limit={{30,5,10,15,15,15,15,4,4,15,15,15,15,4,4}};
// 500Hz wall loop 기준 한 tick에 허용할 최대 토크 변화. 어깨/팔꿈치는
// 0.20Nm, 손목은 0.06Nm로 제한해 GUI scheduling gap이 있어도 kick을 막는다.
const std::array<double,N> torque_step={{.30,.05,.10,.20,.20,.20,.20,.06,.06,.20,.20,.20,.20,.06,.06}};
}

class GravityPd {
 public:
  GravityPd() {
    pinocchio::urdf::buildModel(ros::package::getPath("dual_arm")+"/urdf/dual_arm.urdf",model_);
    left_grip_=model_.getFrameId("L_grip_frame"); right_grip_=model_.getFrameId("R_grip_frame");
    data_.reset(new pinocchio::Data(model_)); q_.setZero(N);dq_.setZero(N);target_.setZero(N);last_tau_.fill(0.0);
    // spawn_model의 -J 적용 전에 잠깐 발행되는 영점 상태를 목표로 래치하지 않는다.
    // 팔을 몸에서 떼고 팔꿈치를 굽힌, Mission 4의 비특이 안전 시드를 시작 목표로 사용한다.
    target_(3)=-0.2; target_(4)=0.35; target_(6)=-0.9;
    target_(9)=-0.2; target_(10)=-0.35; target_(12)=-0.9;
    for(int i=0;i<N;++i){index_[names[i]]=i; pubs_[i]=nh_.advertise<std_msgs::Float64>("/dual_arm/joint"+std::to_string(i+1)+"_effort_controller/command",1);}
    state_=nh_.subscribe("/dual_arm/joint_states",1,&GravityPd::stateCb,this);
    target_sub_=nh_.subscribe("/dual_arm/gravity_pd_target",1,&GravityPd::targetCb,this);
    squeeze_sub_=nh_.subscribe("/dual_arm/squeeze_force_targets",1,&GravityPd::squeezeCb,this);
  }
  void run(){ros::WallRate rate(500);while(ros::ok()){ros::spinOnce();if(ready_)publish();rate.sleep();}}
 private:
  void stateCb(const sensor_msgs::JointState::ConstPtr&m){
    last_state_wall_=ros::WallTime::now();
    for(size_t j=0;j<m->name.size();++j){auto it=index_.find(m->name[j]);if(it==index_.end()||j>=m->position.size()||j>=m->velocity.size())continue;int i=it->second;q_(i)=m->position[j];dq_(i)=m->velocity[j];seen_[i]=true;}
    if(!ready_&&std::all_of(seen_.begin(),seen_.end(),[](bool x){return x;})){ready_=true;ROS_INFO("Gravity PD ready; safe bent-arm target enabled.");}
  }
  void targetCb(const std_msgs::Float64MultiArray::ConstPtr&m){if(ready_&&m->data.size()==N)for(int i=0;i<N;++i)target_(i)=m->data[i];}
  void squeezeCb(const std_msgs::Float64MultiArray::ConstPtr&m){
    if(m->data.size()!=2&&m->data.size()!=4)return;
    squeeze_force_left_=std::max(0.0,std::min(m->data[0],12.0));
    squeeze_force_right_=std::max(0.0,std::min(m->data[1],12.0));
    common_force_x_=m->data.size()==4?std::max(-5.0,std::min(m->data[2],5.0)):0.0;
    common_force_z_=m->data.size()==4?std::max(-2.0,std::min(m->data[3],2.0)):0.0;
  }
  void publish(){
    const Eigen::VectorXd g=pinocchio::computeGeneralizedGravity(model_,*data_,q_);std::array<double,N> tau{};
    Eigen::VectorXd squeeze_tau=Eigen::VectorXd::Zero(N);
    if(squeeze_force_left_>0.0||squeeze_force_right_>0.0){
      pinocchio::computeJointJacobians(model_,*data_,q_);pinocchio::updateFramePlacements(model_,*data_);
      pinocchio::Data::Matrix6x JL(6,N);JL.setZero();pinocchio::Data::Matrix6x JR(6,N);JR.setZero();
      pinocchio::getFrameJacobian(model_,*data_,left_grip_,pinocchio::LOCAL_WORLD_ALIGNED,JL);
      pinocchio::getFrameJacobian(model_,*data_,right_grip_,pinocchio::LOCAL_WORLD_ALIGNED,JR);
      Eigen::Matrix<double,6,1> FL=Eigen::Matrix<double,6,1>::Zero(),FR=Eigen::Matrix<double,6,1>::Zero();
      // Always squeeze along the line connecting both palm centers. This keeps the
      // two contact forces collinear and opposite even when compliant wrists tilt.
      Eigen::Vector3d palm_axis=data_->oMf[right_grip_].translation()-data_->oMf[left_grip_].translation();
      const double palm_distance=palm_axis.norm();
      if(palm_distance>1e-6)palm_axis/=palm_distance;else palm_axis=Eigen::Vector3d(0.0,-1.0,0.0);
      FL.head<3>()=squeeze_force_left_*palm_axis;
      FR.head<3>()=-squeeze_force_right_*palm_axis;
      FL(0)=FR(0)=0.5*common_force_x_; FL(2)=FR(2)=0.5*common_force_z_;
      FL(0)+=squeeze_force_left_*palm_axis.x(); FR(0)-=squeeze_force_right_*palm_axis.x();
      FL(2)+=squeeze_force_left_*palm_axis.z(); FR(2)-=squeeze_force_right_*palm_axis.z();
      squeeze_tau=JL.transpose()*FL+JR.transpose()*FR;
      squeeze_tau.head<3>().setZero();
    }
    // Gazebo pause 중에는 joint state가 끊기므로 마지막 비영 속도를 계속 감쇠항에
    // 사용하면 unpause 첫 tick에 반대 방향 토크가 튀어 나간다. 20ms 이상 상태가
    // 갱신되지 않았을 때는 정지 상태로 보고 D항의 속도를 0으로 사용한다.
    // GUI 부하에서 실측된 정상 message gap(44ms)보다 충분히 큰 200ms를 사용한다.
    const bool state_fresh=(ros::WallTime::now()-last_state_wall_).toSec()<0.200;
    const bool squeezing=squeeze_force_left_>0.0||squeeze_force_right_>0.0;
    for(int i=0;i<N;++i){
      const double velocity=state_fresh?dq_(i):0.0;
      const bool wrist=(i==7||i==8||i==13||i==14);
      const double p_gain=(squeezing&&wrist)?0.2:kp[i];
      const double d_gain=(squeezing&&wrist)?1.2:kd[i];
      const double desired=std::max(-limit[i],std::min(g(i)+p_gain*(target_(i)-q_(i))-d_gain*velocity+squeeze_tau(i),limit[i]));
      const double delta=std::max(-torque_step[i],std::min(desired-last_tau_[i],torque_step[i]));
      tau[i]=last_tau_[i]+delta; last_tau_[i]=tau[i];
    }
    for(int c=0;c<N;++c){std_msgs::Float64 m;m.data=tau[ctrl_to_pin[c]];pubs_[c].publish(m);}
  }
  ros::NodeHandle nh_;ros::Subscriber state_,target_sub_,squeeze_sub_;std::array<ros::Publisher,N> pubs_;std::map<std::string,int>index_;
  pinocchio::Model model_;std::unique_ptr<pinocchio::Data> data_;Eigen::VectorXd q_,dq_,target_;std::array<bool,N>seen_{};std::array<double,N>last_tau_{};bool ready_=false;ros::WallTime last_state_wall_;pinocchio::FrameIndex left_grip_,right_grip_;double squeeze_force_left_=0.0,squeeze_force_right_=0.0,common_force_x_=0.0,common_force_z_=0.0;
};
int main(int argc,char**argv){ros::init(argc,argv,"gravity_pd_controller");GravityPd c;c.run();}
