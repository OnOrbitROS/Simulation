#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include "geometry_msgs/Pose.h"
#include <thread>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"


#include "orbit_robot_pkg/OrbitLink.h"

namespace gazebo
{
  class Orbit_robot_pkg_plugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      physics::WorldPtr _world =_parent->GetWorld();
      this->orbitReference = _world->ModelByName("orbitReference");
      this->referenceSpaceCraft = this->orbitReference->GetLink("referenceSpaceCraft");
      poseReferenceSpaceCraft.Pos().X()=0.0;
      poseReferenceSpaceCraft.Pos().Y()=0.0;
      poseReferenceSpaceCraft.Pos().Z()=0.0;

      ignition::math::v6::Vector3d g(0.0,0.0,0.0);
      _world->SetGravity(g);
      _world->SetAtmosphereEnabled(false); 
      _world->SetMagneticField(g);
      _world->SetWindEnabled(false);	
      // Crate a topic name 
      std::string orbitPositionTopicName = "/OrbitPosition";
      
      this->rosNode.reset(new ros::NodeHandle("orbitRobotPlugin_rosnode"));

      if (!this->rosNode->getParam("/angular_velocity",angularVelocity))
      {
        angularVelocity = 0;
      }

      ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose>(
        orbitPositionTopicName,
        1,
        boost::bind(&Orbit_robot_pkg_plugin::OnPositionUpdate,this,_1),
        ros::VoidPtr(), &this->rosOrbitPosQueue);
      this->rosOrbitPositionSub = this->rosNode->subscribe(so);

      this->rosOrbitPosQueueThread = std::thread(std::bind(&Orbit_robot_pkg_plugin::QueueThread,this));

      // Store the pointer to the model
      this->model = _parent;
      std::string topic_name;
      for (auto &workingLink : this->model->GetLinks() )
      {
        //Antes descartaba base_link
        if (workingLink->GetName()!= "dummy_base_link")
        {
          OrbitLink link; 
          Eigen::Matrix<double, 3, 3> I;
          workingLink->ResetPhysicsStates();
          I(0,0)= workingLink->GetInertial()->IXX();
          I(0,1)= workingLink->GetInertial()->IXY();
          I(0,2)= workingLink->GetInertial()->IXZ();
          I(1,0)= 0.0;
          I(1,1)= workingLink->GetInertial()->IYY();
          I(1,2)= workingLink->GetInertial()->IYZ();
          I(2,0)= 0.0;
          I(2,1)= 0.0;
          I(2,2)= workingLink->GetInertial()->IZZ();
          link.OrbitLinkInitialize(angularVelocity,workingLink->GetInertial()->Mass(),I);
          link.SetLinkSatellite(workingLink);
          topic_name = "gravityGradient/" + workingLink->GetName();
          link.SetRosLinkPublisher(this->rosNode->advertise<geometry_msgs::Wrench>(topic_name,1));
          orbitLinkVector.push_back(link);    
        }
      }
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&Orbit_robot_pkg_plugin::OnUpdate, this));

      
      this->now_secs =ros::Time::now().toSec();
      this->old_secs = now_secs;
      
      ROS_INFO("Iniciando la simulacion de satelites v1.0");
      ROS_INFO("Loaded ModelPush Plugin with parent...%s", this->model->GetName().c_str());
      
      for (auto &workingLink : orbitLinkVector)
      {
        ROS_INFO("Loaded link %s", workingLink.GetLinkSatellite()->GetName().c_str());
      }
    }

    public: void OnUpdate()
    {
      bool publish = false; 
      this->now_secs = ros::Time::now().toSec() ;
      publish =  ((this->now_secs - this->old_secs) > 1.0 ); 
      
      
      if (this->now_secs > 600.0)
      {

     

        for (auto &workingSatellite : orbitLinkVector)
        {
          position = workingSatellite.GetLinkSatellite()->WorldPose();
          velocity = workingSatellite.GetLinkSatellite()->RelativeLinearVel();
          workingSatellite.SetPosition(position.Pos().X(), position.Pos().Y(), position.Pos().Z() );
          workingSatellite.SetQuaternions(position.Rot().W(),position.Rot().X(),position.Rot().Y(),position.Rot().Z(),
                                          orbitQ.w(), orbitQ.x(),orbitQ.y(),orbitQ.z()); 
          workingSatellite.SetVelocity(velocity.X(),velocity.Y(),velocity.Z());
          //std::cout << "workingSatellite.GetForcesIg()" << workingSatellite.GetForcesIg() << std::endl ;
          workingSatellite.GetLinkSatellite()->AddForce(workingSatellite.GetForcesIg());
          //std::cout << "workingSatellite.GetTorqueIg()" << workingSatellite.GetTorqueIg() << std::endl ;
          workingSatellite.GetLinkSatellite()->AddTorque(workingSatellite.GetTorqueIg());
          //std::cout << "RelativeTorque" << workingSatellite.GetLinkSatellite()->RelativeTorque();
          
          if (publish)
          {
            geometry_msgs::Wrench w;
            w.force.x = workingSatellite.GetForcesIg().X();
            w.force.y = workingSatellite.GetForcesIg().Y();
            w.force.z = workingSatellite.GetForcesIg().Z();
            w.torque.x = workingSatellite.GetTorqueIg().X();
            w.torque.y = workingSatellite.GetTorqueIg().Y();
            w.torque.z = workingSatellite.GetTorqueIg().Z();
            workingSatellite.GetRosLinkPublisher().publish(w);
            this->old_secs = this->now_secs;
          }   
        }

      }
    }

    public: void OnPositionUpdate(const geometry_msgs::PoseConstPtr &_msg)
    {
      orbitQ.w()= _msg->orientation.w;
      //std::cout <<  "orbitQ.w()" << orbitQ.w() << std::endl ; 
      orbitQ.x()= _msg->orientation.x;
      //std::cout <<  "orbitQ.x()" << orbitQ.x() << std::endl ; 
      orbitQ.y()= _msg->orientation.y;
      //std::cout <<  "orbitQ.y()" << orbitQ.y() << std::endl ; 
      orbitQ.z()= _msg->orientation.z; 
      //std::cout <<  "orbitQ.z()" << orbitQ.z() << std::endl ; 
      poseReferenceSpaceCraft.Rot().W() = orbitQ.w();
      poseReferenceSpaceCraft.Rot().X() = orbitQ.x();
      poseReferenceSpaceCraft.Rot().Y() = orbitQ.y();
      poseReferenceSpaceCraft.Rot().Z() = orbitQ.z();
      
      this->referenceSpaceCraft->SetWorldPose(poseReferenceSpaceCraft);  
    }
    
    private: void QueueThread()
    {
      static const double timeout = 2.0;
      while (this->rosNode->ok())
      {
        this->rosOrbitPosQueue.callAvailable(ros::WallDuration(timeout));
      }
    } 
    private: 
            physics::ModelPtr model;
            physics::ModelPtr orbitReference;
            physics::LinkPtr referenceSpaceCraft;  
            ignition::math::Pose3d posReference; 
            std::vector<OrbitLink> orbitLinkVector; 
            
    private: 
            event::ConnectionPtr updateConnection;
            // Time memory 
            double old_secs;
            double now_secs;
            // Angular velocity
            double angularVelocity;
            // Ros
            std::unique_ptr<ros::NodeHandle> rosNode; 
            ros::Subscriber rosOrbitPositionSub;
            ros::CallbackQueue rosOrbitPosQueue;
            std::thread rosOrbitPosQueueThread;
            geometry_msgs::Pose orbitPos;

            // Position memory
            ignition::math::v6::Vector3d velocity;
            ignition::math::Pose3d position;
            ignition::math::Pose3d poseReferenceSpaceCraft;
            Eigen::Quaternion<double, 0> orbitQ;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Orbit_robot_pkg_plugin)
}