#ifndef ORBITLINK_H
#define ORBITLINK_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <Eigen/Geometry>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>


class OrbitLink
{
    public:
        OrbitLink();
        virtual ~OrbitLink();
        void OrbitLinkInitialize(double ReferenceAngularVelocity, double mass, Eigen::Matrix<double, 3, 3> i);
        
        //Getters
        double GetMass(){ return Mass; }
        Eigen::Matrix<double, 3, 3> GetInertia(){ return Inertia; }
        Eigen::Matrix<double, 3, 1> GetCoM(){ return CoM; }
        Eigen::Matrix<double, 3, 1> GetVelocityIni() { return VelocityIni; }
        Eigen::Matrix<double, 3, 3> GetLvLhRrLvLhIni() { return LvLhRrLvLhIni;}
        Eigen::Matrix<double, 3, 1> GetAngularRate() { return AngularRate; }
        Eigen::Matrix<double, 3, 1> GetPosition() { return Position; }
        Eigen::Matrix<double, 3, 1> GetVelocity() { return Velocity; }
        Eigen::Matrix<double, 3, 1> GetForces() { return Forces; }
        ignition::math::v6::Vector3d GetForcesIg();
        Eigen::Matrix<double, 3, 3> GetBodyRrLvLh() { return BodyRrLvLh; }
        Eigen::Matrix<double, 3, 3> GetBodyRrEci(){return Body_Rr_Eci; }
        Eigen::Quaternion<double, 0> GetLinkQ() {return linkQ; }
        Eigen::Quaternion<double, 0> GetOrbitQ() {return orbitQ; }
        Eigen::Matrix<double, 3, 1> GetThrust() {return Thrust; }
        Eigen::Matrix<double, 3, 1> GetTorque() {return Torque; }
        ignition::math::v6::Vector3d GetTorqueIg();
        Eigen::Matrix<double, 3, 1> GetControlTorque() {return ControlTorque; }
        Eigen::Matrix<double, 3, 6> GetCW(){return CW; }
        gazebo::physics::LinkPtr GetLinkSatellite(){return LinkSatellite;}
        ros::Publisher GetRosLinkPublisher() { return rosLinkPublisher;}
        //Setters
        void Setn0 (double val){ n0 = val; }
        void SetMass(double val) { Mass = val; }
        void SetInertia(Eigen::Matrix<double, 3, 3> val) { Inertia = val; }
        void SetCoM( Eigen::Matrix<double, 3, 1> val) { CoM = val;  }
        void SetVelocityIni( Eigen::Matrix<double, 3, 1> val) { VelocityIni = val; }
        void SetLvLhRrLvLhIni( Eigen::Matrix<double, 3, 3> val) { LvLhRrLvLhIni = val; }
        void SetAngularRate ( Eigen::Matrix<double, 3, 1> val) { AngularRate = val; }
        void SetPosition( Eigen::Matrix<double, 3, 1> val) { Position = val; }
        void SetPosition( double x, double y, double z);
        void SetVelocity( Eigen::Matrix<double, 3, 1> val) { Velocity = val; }
        void SetVelocity (double Vx, double Vy, double Vz );
        void SetForces( Eigen::Matrix<double, 3, 1> val) { Forces = val; }
        void SetBodyRrLvLh( Eigen::Matrix<double, 3, 3> val) { BodyRrLvLh = val; }
        void SetQuaternions( Eigen::Quaternion<double , 0> link, Eigen::Quaternion<double , 0> orbit ) { linkQ = link; orbitQ = orbit; }
        void SetQuaternions( double linkW, double linkX, double linkY, double linkZ, double orbitW, double orbitX, double orbitY, double orbitZ);
        void SetThrust( Eigen::Matrix<double, 3, 1> val) { Thrust = val; }
        void SetTorque( Eigen::Matrix<double, 3, 1> val) { Torque = val; }
        void SetControlTorque( Eigen::Matrix<double, 3, 1> val) { ControlTorque = val; }
        void SetCW( Eigen::Matrix<double, 3, 6> val) { CW = val; }
        void SetLinkSatellite( gazebo::physics::LinkPtr val) { LinkSatellite = val;}
        void SetRosLinkPublisher(ros::Publisher val) { rosLinkPublisher = val;}

        //Dynamics
        void DynamicsCalcForces();
        void DynamicsCalcTorque(); 

    private:
        //Aux 
        Eigen::Matrix<double, 3, 3> AuxMatrixVectorProduct ( Eigen::Matrix<double, 3, 1>);
        ignition::math::Vector3d FromEtoI(Eigen::Matrix<double, 3, 1> val);
        Eigen::Matrix<double, 3, 3> CalcRrFromQ(Eigen::Quaternion<double, 0> Q);
    
    private:
        double n0;
        double Mass;
        Eigen::Matrix<double, 3, 3> Inertia;
        Eigen::Matrix<double, 3, 1> CoM;
        Eigen::Matrix<double, 3, 1> VelocityIni;
        Eigen::Matrix<double, 3, 3> LvLhRrLvLhIni; 
        Eigen::Matrix<double, 3, 1> AngularRate;
        Eigen::Matrix<double, 3, 1> Position;
        Eigen::Matrix<double, 3, 1> Velocity;
        Eigen::Matrix<double, 3, 1> Forces; 
        Eigen::Matrix<double, 3, 3> BodyRrLvLh;
        Eigen::Quaternion<double, 0> linkQ;
        Eigen::Quaternion<double, 0> orbitQ; 
        Eigen::Matrix<double, 3, 1> Thrust;
        Eigen::Matrix<double, 3, 1> Torque;
        Eigen::Matrix<double, 3, 1> ControlTorque; 
        Eigen::Matrix<double, 3, 6> CW; 
        Eigen::Matrix<double, 3, 3> Body_Rr_Eci;
        Eigen::Matrix<double, 3, 3> LvLh_Rr_Eci;
        Eigen::Matrix<double, 3, 1> TH; 
        Eigen::Matrix<double, 3, 6> J2;
        Eigen::Matrix<double, 6, 1> PositionVelocity;
        gazebo::physics::LinkPtr LinkSatellite;
        ros::Publisher rosLinkPublisher;
};

#endif // ORBITLINK_H
