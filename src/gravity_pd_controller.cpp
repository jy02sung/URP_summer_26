#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
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
}

class GravityPd {
 public:
  GravityPd() {
    pinocchio::urdf::buildModel(ros::package::getPath("dual_arm")+"/urdf/dual_arm.urdf",model_);
    data_.reset(new pinocchio::Data(model_)); q_.setZero(N);dq_.setZero(N);target_.setZero(N);
    // spawn_model의 -J 적용 전에 잠깐 발행되는 영점 상태를 목표로 래치하지 않는다.
    // 팔을 몸에서 떼고 팔꿈치를 굽힌, Mission 4의 비특이 안전 시드를 시작 목표로 사용한다.
    target_(3)=-0.2; target_(4)=0.35; target_(6)=-0.9;
    target_(9)=-0.2; target_(10)=-0.35; target_(12)=-0.9;
    for(int i=0;i<N;++i){index_[names[i]]=i; pubs_[i]=nh_.advertise<std_msgs::Float64>("/dual_arm/joint"+std::to_string(i+1)+"_effort_controller/command",1);}
    state_=nh_.subscribe("/dual_arm/joint_states",1,&GravityPd::stateCb,this);
    target_sub_=nh_.subscribe("/dual_arm/gravity_pd_target",1,&GravityPd::targetCb,this);
  }
  void run(){ros::WallRate rate(500);while(ros::ok()){ros::spinOnce();if(ready_)publish();rate.sleep();}}
 private:
  void stateCb(const sensor_msgs::JointState::ConstPtr&m){
    last_state_wall_=ros::WallTime::now();
    for(size_t j=0;j<m->name.size();++j){auto it=index_.find(m->name[j]);if(it==index_.end()||j>=m->position.size()||j>=m->velocity.size())continue;int i=it->second;q_(i)=m->position[j];dq_(i)=m->velocity[j];seen_[i]=true;}
    if(!ready_&&std::all_of(seen_.begin(),seen_.end(),[](bool x){return x;})){ready_=true;ROS_INFO("Gravity PD ready; safe bent-arm target enabled.");}
  }
  void targetCb(const std_msgs::Float64MultiArray::ConstPtr&m){if(ready_&&m->data.size()==N)for(int i=0;i<N;++i)target_(i)=m->data[i];}
  void publish(){
    const Eigen::VectorXd g=pinocchio::computeGeneralizedGravity(model_,*data_,q_);std::array<double,N> tau{};
    // Gazebo pause 중에는 joint state가 끊기므로 마지막 비영 속도를 계속 감쇠항에
    // 사용하면 unpause 첫 tick에 반대 방향 토크가 튀어 나간다. 20ms 이상 상태가
    // 갱신되지 않았을 때는 정지 상태로 보고 D항의 속도를 0으로 사용한다.
    const bool state_fresh=(ros::WallTime::now()-last_state_wall_).toSec()<0.020;
    for(int i=0;i<N;++i){
      const double velocity=state_fresh?dq_(i):0.0;
      tau[i]=std::max(-limit[i],std::min(g(i)+kp[i]*(target_(i)-q_(i))-kd[i]*velocity,limit[i]));
    }
    for(int c=0;c<N;++c){std_msgs::Float64 m;m.data=tau[ctrl_to_pin[c]];pubs_[c].publish(m);}
  }
  ros::NodeHandle nh_;ros::Subscriber state_,target_sub_;std::array<ros::Publisher,N> pubs_;std::map<std::string,int>index_;
  pinocchio::Model model_;std::unique_ptr<pinocchio::Data> data_;Eigen::VectorXd q_,dq_,target_;std::array<bool,N>seen_{};bool ready_=false;ros::WallTime last_state_wall_;
};
int main(int argc,char**argv){ros::init(argc,argv,"gravity_pd_controller");GravityPd c;c.run();}
