#include "orbit_robot_pkg/OrbitLink.h"

OrbitLink::OrbitLink()
{ 
}

OrbitLink::~OrbitLink()
{
}

void OrbitLink::OrbitLinkInitialize(double orbitReferenceAngularVelocity, double m, Eigen::Matrix<double, 3, 3> i)
{
    n0 = orbitReferenceAngularVelocity;
    Mass = m;
    Inertia = i;
    Inertia(1,0) = i(0,1);
    Inertia(2,0) = i(0,2);
    Inertia(2,1) = i(1,2);
}

 Eigen::Matrix<double, 3, 3> OrbitLink::AuxMatrixVectorProduct ( Eigen::Matrix<double, 3, 1> vector)
 {
    Eigen::Matrix<double, 3, 3> result; 
    result << 0.0 , -vector(2), vector(1),
              vector(2), 0.0, -vector(0), 
              -vector(1), vector(0), 0.0;

     return result; 
 }

 ignition::math::v6::Vector3d OrbitLink::FromEtoI(Eigen::Matrix<double, 3, 1> val)
 {
    ignition::math::v6::Vector3d res(val(0),val(1),val(2));
    return res;
 }

Eigen::Matrix<double, 3, 3> OrbitLink::CalcRrFromQ(Eigen::Quaternion<double, 0> Q)
{
    Eigen::Matrix<double, 3, 3> RrMatrix;
    return RrMatrix;
}

void OrbitLink::SetOrbitPosition( double x, double y, double z) 
{ 
    this->orbit_position.x()= x;
    this->orbit_position.y()= y;
    this->orbit_position.z()= z;
}

void OrbitLink::SetOrbitRelativePosition( double x, double y, double z) 
{ 
    this->orbit_relative_position.x()= x;
    this->orbit_relative_position.y()= y;
    this->orbit_relative_position.z()= z;
}
void OrbitLink::SetOrbitVelocity (double Vx, double Vy, double Vz ) 
{ 
    this->orbit_velocity.x()=Vx; 
    this->orbit_velocity.y()=Vy; 
    this->orbit_velocity.z()=Vz; 
}
void OrbitLink::SetOrbitRelativeVelocity (double Vx, double Vy, double Vz ) 
{ 
    this->orbit_relative_velocity.x()=Vx; 
    this->orbit_relative_velocity.y()=Vy; 
    this->orbit_relative_velocity.z()=Vz; 
}
void OrbitLink::SetQuaternions( double linkW, double linkX, double linkY, double linkZ, 
                                double orbitW, double orbitX, double orbitY, double orbitZ)
{
    this->linkQ.w() = linkW;
    this->linkQ.x() = linkX;
    this->linkQ.y() = linkY;
    this->linkQ.z() = linkZ;
    this->orbitQ.w() = orbitW;
    this->orbitQ.x() = orbitX;
    this->orbitQ.y() = orbitY;
    this->orbitQ.z() = orbitZ;

    Body_Rr_Eci = this->linkQ;
    LvLh_Rr_Eci = this->orbitQ;
    BodyRrLvLh = Body_Rr_Eci * LvLh_Rr_Eci.transpose() ; 
    
}

void OrbitLink::DynamicsCalcForces()
{
    //In Km
    double R_ = orbit_position.norm();
    double R_dot_V = orbit_position.dot(orbit_velocity); 
    double h = (orbit_position.cross(orbit_velocity)).norm();

    Forces(0,0) = Mass * ((2*kMUe/pow(R_,3) + pow(h,2)/pow(R_,4)) * orbit_relative_position(0,0) - (2 * R_dot_V/(pow(R_,4) * h * orbit_relative_position(1,0)) + (2 * h)/(pow(R_,2) * orbit_relative_velocity(1,0))) );
    Forces(1,0) = Mass * (-(kMUe/pow(R_,3) - pow(h,2)/pow(R_,4)) * orbit_relative_position(1,0) + (2 * R_dot_V/(pow(R_,4) * h * orbit_relative_position(0,0))) - ((2 * h)/pow(R_,2) * orbit_relative_velocity(0,0)) );
    Forces(2,0) = Mass * (-kMUe/(pow(R_,3) * orbit_relative_position(2,0)));
    // To apply the forces in the ECI frame. 
    Forces = BodyRrLvLh * Forces;
}

ignition::math::v6::Vector3d OrbitLink::GetForcesIg()
{
    DynamicsCalcForces();
    return (FromEtoI(Forces));
}

ignition::math::v6::Vector3d OrbitLink::GetTorqueIg()
{
    DynamicsCalcTorque();
    return (FromEtoI(Torque));
}

void OrbitLink::DynamicsCalcTorque()
{
        // Extractt from the rotation matrix the X vector. 
        Eigen::Matrix<double, 3, 1> RadialDirectionLvLh ( 1.0, 0.0, 0.0);
        Eigen::Matrix<double, 3, 3> RadialDirectionRrBodyEx;
        Eigen::Matrix<double, 3, 1> RadialDirectionRrBody = BodyRrLvLh * RadialDirectionLvLh;
        RadialDirectionRrBodyEx = AuxMatrixVectorProduct ( RadialDirectionRrBody);
        Torque =  3.0 * (n0 * n0) * (RadialDirectionRrBodyEx * (Inertia * RadialDirectionRrBody));
}