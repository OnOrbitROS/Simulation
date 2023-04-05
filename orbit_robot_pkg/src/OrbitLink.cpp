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
    
    CoM << 0.0, 0.0, 0.0;
    CW << (3.0 * pow(n0,2)), 0.0, 0.0, 0.0, (2.0 * n0), 0.0,
           0.0 , 0.0,  0.0,((-2.0) * n0), 0.0, 0.0,
           0.0 , 0.0,  -(pow(n0,2)), 0.0, 0.0, 0.0;
    // To be implemented
    J2 << 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0,
          0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0,
          0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0;
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

void OrbitLink::SetPosition( double x, double y, double z) 
{ 
    this->Position.x()= x;
    this->Position.y()= y;
    this->Position.z()= z;
}

void OrbitLink::SetVelocity (double Vx, double Vy, double Vz ) 
{ 
    this->Velocity.x()=Vx; 
    this->Velocity.y()=Vy; 
    this->Velocity.z()=Vz; 
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
    //if (Mass > 1000.0)  
    //std::cout << "Body_Rr_Eci" << std::endl << Body_Rr_Eci << std::endl;
    LvLh_Rr_Eci = this->orbitQ;
    //if (Mass > 1000.0)  
    //std::cout << "LvLh_Rr_Eci" << std::endl << LvLh_Rr_Eci << std::endl;
    BodyRrLvLh = Body_Rr_Eci * LvLh_Rr_Eci.transpose() ; 
    //if (Mass > 1000.0)  
    //std::cout << "BodyRrLvLh" << std::endl << BodyRrLvLh << std::endl;
    //std::cout << "Mass = "  << Mass << std::endl << "BodyRrLvLh" << std::endl << BodyRrLvLh << std::endl;
}

void OrbitLink::DynamicsCalcForces()
{
    PositionVelocity << Position, Velocity;
    Forces = (CW * PositionVelocity)* Mass + (J2 * PositionVelocity) * Mass ;
    Forces << 0.0 , 0.0 , 0.0 ;   
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
    /* if (Mass > 1000.0)
    { */
 
    
        Eigen::Matrix<double, 3, 1> RadialDirectionLvLh ( 1.0, 0.0, 0.0);
        Eigen::Matrix<double, 3, 3> RadialDirectionRrBodyEx;
        Eigen::Matrix<double, 3, 1> RadialDirectionRrBody = BodyRrLvLh * RadialDirectionLvLh;
        //if (Mass > 1000.0)  
        //std::cout << "BodyRrLvLh" << std::endl << BodyRrLvLh << std::endl;
        //if (Mass > 1000.0)  
        //std::cout << "RadialDirectionRrBody" << std::endl << RadialDirectionRrBody << std::endl;
        Eigen::Matrix<double, 3, 1> GravityGradiant;
        Eigen::Matrix<double, 3, 1> EnviromentalTorque;
        //if (Mass > 1000.0)  
        //std::cout << "Inertia" << std::endl << Inertia << std::endl;
        Inertia = Inertia -  Mass * (CoM.transpose() * CoM * Eigen::MatrixXd::Identity(3,3) - CoM * CoM.transpose());
        //if (Mass > 1000.0)  
        //std::cout << "Inertia" << std::endl << Inertia << std::endl;
        RadialDirectionRrBodyEx = AuxMatrixVectorProduct ( RadialDirectionRrBody);
        //if (Mass > 1000.0)  
        //std::cout << "RadialDirectionRrBodyEx" << std::endl << RadialDirectionRrBodyEx << std::endl; 
        GravityGradiant =  RadialDirectionRrBodyEx * (Inertia * RadialDirectionRrBody);
        //if (Mass > 1000.0)  
        //std::cout << "GravityGradiant" << std::endl << GravityGradiant << std::endl;
        EnviromentalTorque = 3.0 * (n0 * n0) * GravityGradiant;
        //if (Mass > 1000.0)  
        //std::cout << "EnviromentalTorque" << std::endl << EnviromentalTorque << std::endl;  
        
        Torque = EnviromentalTorque ;
}
