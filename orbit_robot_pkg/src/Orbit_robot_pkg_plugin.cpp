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
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      ReadParameters(_sdf);

      physics::WorldPtr _world =_model->GetWorld();

      // Override physical properties for the space environment
      ignition::math::v6::Vector3d g(0.0,0.0,0.0);
      _world->SetGravity(g);
      _world->SetAtmosphereEnabled(false); 
      _world->SetMagneticField(g);
      _world->SetWindEnabled(false);	

      this->target = _world->ModelByName(target_name);
      this->chaser = _world->ModelByName(chaser_name);

      // Create the node to interact with ROS
      this->rosNode.reset(new ros::NodeHandle("/" + chaser_name  + "/orbit_plugin_node"));

      if (!this->rosNode->getParam("/" + chaser_name  + "/angular_velocity",angularVelocity))
      {
        angularVelocity = 0;
      }

      CreateSubsCribers();
     
      // Store the pointer to the model
      this->model = _model;
      // Link the base of the target spacecraft
      this->target_base_link = this->target->GetLink("base");
      // Create the link vector
      std::string topic_name;
      for (auto &workingLink : this->chaser->GetLinks() )
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
        topic_name = "/" + chaser_name  + "/gravityGradient/" + workingLink->GetName();
        link.SetRosLinkPublisher(this->rosNode->advertise<geometry_msgs::Wrench>(topic_name,1));
        chaser_link_vector.push_back(link);    
      }
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&Orbit_robot_pkg_plugin::OnUpdate, this));

      
      this->now_secs =ros::Time::now().toSec();
      this->old_secs = now_secs;
      
      ROS_INFO("Iniciando la simulacion de satelites v1.0");
      ROS_INFO("Loaded ModelPush Plugin with parent...%s", this->model->GetName().c_str());
      
      for (auto &workingLink : chaser_link_vector)
      {
        ROS_INFO("Loaded link %s", workingLink.GetLinkSatellite()->GetName().c_str());
      }
    }

    public: void OnUpdate()
    {
      bool publish = false; 
      this->now_secs = ros::Time::now().toSec() ;
      if ( publish_rate > 0.0)
      {
         publish =  ((this->now_secs - this->old_secs) > publish_rate ); 
      }
     
      for (auto &workingSatellite : chaser_link_vector)
      { 
        orbit_relative_position.Pos().X() = workingSatellite.GetLinkSatellite()->WorldCoGPose().Pos().X()/1000.0;
        orbit_relative_position.Pos().Y() = workingSatellite.GetLinkSatellite()->WorldCoGPose().Pos().Y()/1000.0;
        orbit_relative_position.Pos().Z() = workingSatellite.GetLinkSatellite()->WorldCoGPose().Pos().Z()/1000.0;
        orbit_relative_position.Rot()= workingSatellite.GetLinkSatellite()->WorldCoGPose().Rot();
         
        orbit_relative_velocity.X() = workingSatellite.GetLinkSatellite()->WorldCoGLinearVel().X()/1000.0;
        orbit_relative_velocity.Y() = workingSatellite.GetLinkSatellite()->WorldCoGLinearVel().Y()/1000.0;
        orbit_relative_velocity.Z() = workingSatellite.GetLinkSatellite()->WorldCoGLinearVel().Z()/1000.0;
        
        workingSatellite.SetOrbitRelativePosition(orbit_relative_position.Pos().X(), orbit_relative_position.Pos().Y(), orbit_relative_position.Pos().Z() );
        workingSatellite.SetQuaternions(orbit_relative_position.Rot().W(),orbit_relative_position.Rot().X(),orbit_relative_position.Rot().Y(),orbit_relative_position.Rot().Z(),
                                        orbit_position.Rot().W(), orbit_position.Rot().X(),orbit_position.Rot().Y(),orbit_position.Rot().Z()); 
        workingSatellite.SetOrbitRelativeVelocity(orbit_relative_velocity.X(),orbit_relative_velocity.Y(),orbit_relative_velocity.Z());
        workingSatellite.SetOrbitPosition(orbit_position.Pos().X(),orbit_position.Pos().Y(),orbit_position.Pos().Z());
        workingSatellite.SetOrbitVelocity(orbit_velocity.X(),orbit_velocity.Y(),orbit_velocity.Z());
        workingSatellite.GetLinkSatellite()->AddForce(workingSatellite.GetForcesIg());
        workingSatellite.GetLinkSatellite()->AddTorque(workingSatellite.GetTorqueIg());
 
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

    public: void OnPositionUpdate(const geometry_msgs::PoseConstPtr &_msg)
    {
      orbit_position.Rot().W()= _msg->orientation.w;
      orbit_position.Rot().X()= _msg->orientation.x;
      orbit_position.Rot().Y()= _msg->orientation.y;
      orbit_position.Rot().Z()= _msg->orientation.z; 
      orbit_position.Pos().X()= _msg->position.x;
      orbit_position.Pos().Y()= _msg->position.x;
      orbit_position.Pos().Z()= _msg->position.x;
      simulation_pose_reference_spacecraft.Rot().W() = _msg->orientation.w;
      simulation_pose_reference_spacecraft.Rot().X() = _msg->orientation.x;
      simulation_pose_reference_spacecraft.Rot().Y() = _msg->orientation.y;
      simulation_pose_reference_spacecraft.Rot().Z() = _msg->orientation.z;
      simulation_pose_reference_spacecraft.Pos().X() = 0.0;
      simulation_pose_reference_spacecraft.Pos().Y() = 0.0;
      simulation_pose_reference_spacecraft.Pos().Z() = 0.0;
      
      this->target_base_link->SetWorldPose(simulation_pose_reference_spacecraft);  
    }

    public: void OnVelocityUpdate(const geometry_msgs::Vector3ConstPtr &_msg)
    {
      orbit_velocity.X() = _msg->x;
      orbit_velocity.Y() = _msg->y;
      orbit_velocity.Z() = _msg->z;
    }

    private: void ReadParameters(sdf::ElementPtr _sdf)
    {
      if (_sdf->HasElement("orbit_reference"))
      {
        orbit_name = _sdf->Get<std::string>("orbit_reference");
      }
      if (_sdf->HasElement("publish_rate"))
      {
        publish_rate = _sdf->Get<double>("publish_rate");
      } else
      {
        publish_rate = 0.0; 
      }
      if (_sdf->HasElement("target_name"))
      {
        target_name = _sdf->Get<std::string>("target_name");
      }
      if (_sdf->HasElement("chaser_name"))
      {
        chaser_name = _sdf->Get<std::string>("chaser_name");
      }
    }

    private: void CreateSubsCribers()
    {
       // Crate a topic subscription for position 
      std::string topic_name_position = orbit_name + "/OrbitPosition";
      ros::SubscribeOptions so_p = ros::SubscribeOptions::create<geometry_msgs::Pose>(
        topic_name_position,
        1,
        boost::bind(&Orbit_robot_pkg_plugin::OnPositionUpdate,this,_1),
        ros::VoidPtr(), &this->rosOrbitPosQueue);
      this->rosOrbitPositionSub = this->rosNode->subscribe(so_p);

       // Crate a topic subscription for velocity  
      std::string topic_name_velocity = orbit_name + "/OrbitVelocity";
      ros::SubscribeOptions so_v = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
        topic_name_velocity,
        1,
        boost::bind(&Orbit_robot_pkg_plugin::OnVelocityUpdate,this,_1),
        ros::VoidPtr(), &this->rosOrbitVelQueue);
      this->rosOrbitVelocitySub = this->rosNode->subscribe(so_v);                

      this->rosOrbitPosQueueThread = std::thread(std::bind(&Orbit_robot_pkg_plugin::QueueThreadPosition,this));
      this->rosOrbitVelQueueThread = std::thread(std::bind(&Orbit_robot_pkg_plugin::QueueThreadVelocity,this));

    }
    
    private: void QueueThreadPosition()
    {
      static const double timeout = 2.0;
      while (this->rosNode->ok())
      {
        this->rosOrbitPosQueue.callAvailable(ros::WallDuration(timeout));
      }
    } 
    private: void QueueThreadVelocity()
    {
      static const double timeout = 2.0;
      while (this->rosNode->ok())
      {
        this->rosOrbitVelQueue.callAvailable(ros::WallDuration(timeout));
      }
    } 
   
    private: 
            std::string orbit_name;
            std::string target_name;
            std::string chaser_name;
            physics::ModelPtr model;
            physics::ModelPtr target;
            physics::ModelPtr chaser; 

            //physics::ModelPtr orbitReference;
            physics::LinkPtr target_base_link;  
            ignition::math::Pose3d posReference; 
            std::vector<OrbitLink> chaser_link_vector; 
            
    private: 
            event::ConnectionPtr updateConnection;
            // Time memory 
            double old_secs;
            double now_secs;
            double publish_rate; 
            // Angular velocity
            double angularVelocity;
            // Ros
            std::unique_ptr<ros::NodeHandle> rosNode; 
            ros::Subscriber rosOrbitPositionSub;
            ros::Subscriber rosOrbitVelocitySub; 
            ros::CallbackQueue rosOrbitPosQueue;
            ros::CallbackQueue rosOrbitVelQueue;
            std::thread rosOrbitPosQueueThread;
            std::thread rosOrbitVelQueueThread; 
            geometry_msgs::Pose orbitPos;

            // Position memory
            ignition::math::v6::Vector3d orbit_relative_velocity;
            ignition::math::Pose3d orbit_relative_position;
            ignition::math::v6::Vector3d orbit_velocity;
            ignition::math::Pose3d orbit_position;
            ignition::math::Pose3d simulation_pose_reference_spacecraft;
            //Eigen::Quaternion<double, 0> orbitQ;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Orbit_robot_pkg_plugin)
}