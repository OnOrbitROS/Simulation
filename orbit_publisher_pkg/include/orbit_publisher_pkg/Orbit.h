#ifndef ORBIT_H
#define ORBIT_H

#include <iostream>
#include <time.h>
#include <string>
#include <Eigen/Dense>
#include "ros/ros.h"



class Orbit
{
    public:
        Orbit();
        Orbit(ros::NodeHandle nh, std::string n);
        virtual ~Orbit();
        
        //Getters
        std::string GetName() { return name; }
        double GetEccentricity() { return eccentricity; }
        double GetSemiMajorAxis() {return semi_major_axis; }
        double GetInclination() {return inclination; }
        double GetRateOfRightAscension() { return rate_of_right_ascension; }
        double GetRightAscensionIni(){return  right_ascension_ini; }
        double GetArgumentOfPerigeeIni() {return argument_of_perigee_ini; }
        double GetRateArgumentOfPerigee() {return rate_argument_of_perigee; }
        double GetMeanAnomalyIni() { return mean_anomaly_ini; }
        time_t GetTimePassPerigee() {return time_pass_perigee; }
        Eigen::Matrix<double, 3, 1> GetPositionEciIni() { return position_eci_ini ; }
        Eigen::Matrix<double, 3, 1> GetVelocityEciIni() { return velocity_eci_ini; }
        Eigen::Matrix<double, 3, 3> GetLvLhRotationToEciIni() { return lvlh_rotation_to_eci_ini; }
        Eigen::Matrix<double, 3, 1> GetPositionEci() { return position_eci; }
        Eigen::Matrix<double, 3, 1> GetVelEph() { return vel_eph; };
        double GetPositionEciX() { return position_eci(0); }
        double GetPositionEciY() { return position_eci(1); }
        double GetPositionEciZ() { return position_eci(2); }
        Eigen::Matrix<double, 3, 1> GetVelocityEci() { return velocity_eci; }
        double GetVelocityEciX(){ return velocity_eci(0); }
        double GetVelocityEciY(){ return velocity_eci(1); }
        double GetVelocityEciZ(){ return velocity_eci(2); }
        Eigen::Matrix<double, 3, 3> GetLvLhRotationToEci() { return lvlh_rotation_to_eci; }
        double GetAngularVelocity() { return angular_velocity; }
        double GetPeriod(){ return period; }
        double GetMeanMotion(){return mean_motion; }
        double GetEccentricAnomaly(){return eccentric_anomaly; }

        //Setters
        void SetName(std::string val) { name = val; }
        void SetEccentricity(double val) { eccentricity = val; }
        void SetSemiMajorAxis (double val) { semi_major_axis = val; }
        void SetInclination ( double val ) { inclination = val; }
        void SetRateOfRightAscension (double val) { rate_of_right_ascension = val; }
        void SetRightAscensionIni (double val) { right_ascension_ini = val; }
        void SetArgumentOfPerigeeIni (double val) { argument_of_perigee_ini = val; }
        void SetRateArgumentOfPerigee (double val) { rate_argument_of_perigee = val; }
        void SetMeanAnomalyIni  (double val) { mean_anomaly_ini = val; }
        void SetTimePassPerigee (time_t val) { time_pass_perigee = val; }
        void SetPosEciIni (Eigen::Matrix<double, 3, 1> val )  {position_eci_ini = val; }
        void SetValEciIni (Eigen::Matrix<double, 3, 1> val) { velocity_eci_ini = val; }
        void SetLvLhRotationToEciIni (Eigen::Matrix<double, 3, 3> val) { lvlh_rotation_to_eci_ini = val; }
        void SetPosEci (Eigen::Matrix<double, 3, 1> val) { position_eci = val; }
        void SetValEci (Eigen::Matrix<double, 3, 1> val) { velocity_eci = val; }
        void SetLvLhRrEci (Eigen::Matrix<double, 3, 3> val)  { lvlh_rotation_to_eci = val; }
        void SetAngularVelocity (double val) { angular_velocity = val; }
        void SetPeriod ( double val) {period = val; }
        void SetMeanMotion(double val) {mean_motion = val;}
        void SetEccentricAnomaly(double val) {eccentric_anomaly = val;}
        
        //Const
        const double kMUe = 398600.5; //Earth's gravitational constant (Km^3/s^2)
        const double kRe = 6378; // Earth radius (Km)
        const double kOMe = 2.0 * M_PI / (3600.0 * 24.0) ; // Earth's rate (rad/sec)
        const double kJ2 = 1.082e-3;
        const double kg0 = 9.81 ; // m/s
        
        void KeplerianToEci(double time );



    private:
        std::string name;
        double eccentricity;
        double semi_major_axis;
        double inclination;
        double rate_of_right_ascension;
        double right_ascension_ini;
        double argument_of_perigee_ini;
        double rate_argument_of_perigee;
        double mean_anomaly_ini;
        time_t time_pass_perigee;
        double s_from_perigee_to_simulation;
        double angular_velocity; 
        double period;
        double mean_motion;
        double eccentric_anomaly;
        double mu_divided_h;
       
        Eigen::Matrix<double, 3, 1> position_eci_ini;
        Eigen::Matrix<double, 3, 1> velocity_eci_ini;
        Eigen::Matrix<double, 3, 3> lvlh_rotation_to_eci_ini;
        Eigen::Matrix<double, 3, 1> position_eci;
        Eigen::Matrix<double, 3, 1> velocity_eci;
        Eigen::Matrix<double, 3, 3> lvlh_rotation_to_eci;
        Eigen::Matrix<double, 3, 1> vel_eph;
        Eigen::Matrix<double, 3, 3> ephemeris_rotation_plane;
        Eigen::Matrix<double, 3, 1> velocity_plane;
        Eigen::Matrix<double, 3, 3> plane_rotation_to_eci;
        Eigen::Matrix<double, 3, 3> right_ascension_rotation;
        Eigen::Matrix<double, 3, 3> inclination_rotation;
        Eigen::Matrix<double, 3, 3> true_anomaly_rotation;

        double CalcEccenAnom(double eccentricity, double MeanAnomaly);
        double KeplerStart(double e, double M);
        double ThirdOrderApproximation(double e, double M, double x);
        void J2Effect();
};

#endif // ORBIT_H
