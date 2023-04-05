#include "orbit_publisher_pkg/Orbit.h"

Orbit::Orbit()
{
    // Default parameter from de ISS (source https://www.heavens-above.com/)
    struct tm iniDate = {0}; 
    iniDate.tm_sec = 15; 
    iniDate.tm_min = 52; 
    iniDate.tm_hour = 4; 
    iniDate.tm_mday = 20; 
    iniDate.tm_mon = 2; 
    iniDate.tm_year = 122;  
    time_pass_perigee =  mktime(&iniDate);
    time_t start;
    time(&start);
    s_from_perigee_to_simulation = difftime(start,time_pass_perigee);
    eccentricity = 0.0004003;
    semi_major_axis = 13592;
    inclination = 51.6427 * M_PI / 180.0;
    rate_of_right_ascension = 0;
    right_ascension_ini = 0;
    argument_of_perigee_ini = 282.2950 * M_PI / 180.0;
    rate_argument_of_perigee = 0 ;
    mean_anomaly_ini = 51.7017 * M_PI / 180.0;  
    angular_velocity = sqrt( kMUe / pow( semi_major_axis , 3));
    period = 2.0 * M_PI / angular_velocity ;
    mean_motion = sqrt(kMUe/pow(semi_major_axis,3));
    mu_divided_h = kMUe/(sqrt(kMUe * semi_major_axis * ( 1 - pow(eccentricity,2))));
    J2Effect();
    KeplerianToEci(0.0);
    lvlh_rotation_to_eci_ini = lvlh_rotation_to_eci;
}

Orbit::~Orbit()
{
    //dtor
}

Orbit::Orbit(ros::NodeHandle nh, std::string n)
{
   name = n;
   struct tm ini_date = {0};
   struct tm start_struct = {0};
   bool start_actual = false;
   bool perigee_at_start = false; 
   time_t start;

   if (!nh.getParam("/" + name + "/eccentricity",eccentricity))
   {
       eccentricity = 	0.0004003;
   }
   if (!nh.getParam("/" + name + "/semi_major_axis",semi_major_axis))
   {
       semi_major_axis = 12000;
   }
   if (!nh.getParam("/" + name + "/inclination",inclination))
   {
       inclination = 51.6427 * M_PI / 180.0;
   }
   if (!nh.getParam("/" + name + "/rate_of_right_ascension",rate_of_right_ascension))
   {
       rate_of_right_ascension = 0;
   }
   if (!nh.getParam("/" + name + "/right_ascension_ini",right_ascension_ini))
   {
       right_ascension_ini = 0;
   }
   if (!nh.getParam("/" + name + "/argument_of_perigee_ini",argument_of_perigee_ini))
   {
       argument_of_perigee_ini = 0;
   }
   if (!nh.getParam("/" + name + "/rate_argument_of_perigee",rate_argument_of_perigee))
   {
        rate_argument_of_perigee = 0 ;
   }
   if (!nh.getParam("/" + name + "/mean_anomaly_ini", mean_anomaly_ini))
   {
        mean_anomaly_ini = 60.0 * M_PI / 180.0;
   } 
   if (!nh.getParam("/" + name + "/time_pass_perigee/sec",ini_date.tm_sec))
   {
      perigee_at_start = true; 
   }
   if (!nh.getParam("/" + name + "/time_pass_perigee/min",ini_date.tm_min))
   {
      perigee_at_start = true;
   }
   if (!nh.getParam("/" + name + "/time_pass_perigee/hour",ini_date.tm_hour))
   {
      perigee_at_start = true;
   } 
   if (!nh.getParam("/" + name + "/time_pass_perigee/mday",ini_date.tm_mday))
   {
      perigee_at_start = true;
   }
   if (!nh.getParam("/" + name + "/time_pass_perigee/mon",ini_date.tm_mon))
   {
      perigee_at_start = true;
   }
   if (!nh.getParam("/" + name + "/time_pass_perigee/year",ini_date.tm_year))
   {
      perigee_at_start = true;
   }
   if (perigee_at_start)
   {
       time(&time_pass_perigee);
   }else
   {
       time_pass_perigee = mktime(&ini_date);
   } 
   if (!nh.getParam("/" + name + "/time_start/sec",start_struct.tm_sec))
   {   
      start_actual = true;    
   }
   if (!nh.getParam("/" + name + "/time_start/min",start_struct.tm_min))
   {
       start_actual = true;    
    }
    if (!nh.getParam("/" + name + "/time_start/hour",start_struct.tm_hour))
    {
        start_actual = true;     
    } 
    if (!nh.getParam("/" + name + "/time_start/mday",start_struct.tm_mday))
    {
        start_actual = true;    
    }
    if (!nh.getParam("/" + name + "/time_start/mon",start_struct.tm_mon))
    {
       start_actual = true;    
    }
    if (!nh.getParam("/" + name + "/time_start/year",start_struct.tm_year))
    {
        start_actual = true;    
    }   
    if (start_actual)
    {
        time(&start);
    }else
    {
       start = mktime(&start_struct);
    }
   if (!nh.getParam("/" + name + "/angular_velocity",angular_velocity))
   {
       angular_velocity = sqrt( kMUe / pow( semi_major_axis , 3));
       nh.setParam("/" + name + "/angular_velocity",angular_velocity);
   }
   if (!nh.getParam("/" + name + "/period",period))
   {
       period = 2.0 * M_PI / angular_velocity ;
   }
   s_from_perigee_to_simulation = difftime(start,time_pass_perigee);
   mean_motion = sqrt(kMUe/pow(semi_major_axis,3));
   mu_divided_h = kMUe/(sqrt(kMUe * semi_major_axis * ( 1 - pow(eccentricity,2))));
   J2Effect();
   KeplerianToEci(0.0);
   lvlh_rotation_to_eci_ini = lvlh_rotation_to_eci;
}

double Orbit::CalcEccenAnom(double eccentricity, double mean_anomaly)
{
    //Marc A. Murison A Practical Method for Solving the Kepler Equation
    double tol;
	if (eccentricity < 0.8) tol = 1e-14;
	else tol = 1e-13;

	double Mnorm = fmod(mean_anomaly, 2.*M_PI);
	double E0 = KeplerStart(eccentricity, Mnorm);
	double dE = tol + 1;
	double E;
	int iteration = 0;
	while ((dE > tol) && (iteration < 100))
	{
		E = E0 - ThirdOrderApproximation(eccentricity, Mnorm, E0);
		dE = abs(E-E0);
		E0 = E;
		iteration++;
	}
	return E;
}

double Orbit::KeplerStart(double e, double M)
{
	double t34 = e*e;
	double t35 = e*t34;
	double t33 = cos(M);
	return M + (-0.5*t35 + e + (t34 + 1.5*t33*t35)*t33)*sin(M);
}

double Orbit::ThirdOrderApproximation(double e, double M, double x)
{
	double t1 = cos(x);
	double t2 = -1 + e*t1;
	double t3 = sin(x);
	double t4 = e*t3;
	double t5 = -x + t4 + M;
	double t6 = t5/(0.5*t5*t4/t2+t2);

	return t5/((0.5*t3 - 1/6*t1*t6)*e*t6+t2);
}

void Orbit::KeplerianToEci(double time )
{
    double time_k = time + s_from_perigee_to_simulation;
    double mean_anomaly_time_k = mean_anomaly_ini + mean_motion * time_k;
    eccentric_anomaly = CalcEccenAnom(eccentricity,mean_anomaly_time_k);
    double sin_true_anomaly = (sqrt((1- pow(eccentricity,2))) * sin(eccentric_anomaly))  / ( 1 - eccentricity * cos(eccentric_anomaly));
    double cos_true_anomaly = ( cos(eccentric_anomaly) - eccentricity) / ( 1 - eccentricity * cos(eccentric_anomaly));
    double true_anomaly = atan2( sin_true_anomaly , cos_true_anomaly );

    double arg_of_latitude = true_anomaly + argument_of_perigee_ini + rate_argument_of_perigee * time_k ;
    double radius = semi_major_axis * ( 1 - eccentricity * cos(eccentric_anomaly));
    Eigen::Matrix<double, 3, 1> pos_plane;
    pos_plane(0,0) = radius * cos(arg_of_latitude);
    pos_plane(1,0) = radius * sin(arg_of_latitude);
    pos_plane(2,0) = 0.0;
    
    double right_ascension = right_ascension_ini + rate_of_right_ascension * time_k;

    right_ascension_rotation << cos(right_ascension) , -(sin(right_ascension)) , 0,
                                sin(right_ascension) ,    cos(right_ascension) , 0,
                                                   0 ,                       0 , 1;
    inclination_rotation << 1 ,               0 ,                   0,
                            0 , cos(inclination), -(sin(inclination)),
                            0 , sin(inclination),   cos(inclination) ;

    plane_rotation_to_eci = right_ascension_rotation * inclination_rotation;

    position_eci = plane_rotation_to_eci * pos_plane ;
    
    vel_eph(0,0) = mu_divided_h * -( sin(true_anomaly));
    vel_eph(1,0) = mu_divided_h * (cos(true_anomaly) + eccentricity);
    vel_eph(2,0) = 0;

    double ArgumentOfPerigee = argument_of_perigee_ini + rate_argument_of_perigee * time_k ;
   
    ephemeris_rotation_plane <<(cos(ArgumentOfPerigee)) , -(sin(ArgumentOfPerigee)), 0 ,
                                 sin(ArgumentOfPerigee) ,   cos(ArgumentOfPerigee) , 0 ,
                                                      0 ,                        0 , 1 ;
     
    velocity_plane = ephemeris_rotation_plane * vel_eph;
    
    true_anomaly_rotation << (cos(true_anomaly)) , -(sin(true_anomaly)) , 0,
                              sin(true_anomaly)  ,   cos(true_anomaly)  , 0,
                                               0 ,                   0  , 1;

    velocity_eci = plane_rotation_to_eci * velocity_plane ;
    
    lvlh_rotation_to_eci << plane_rotation_to_eci * ephemeris_rotation_plane * true_anomaly_rotation;
    
    //eci_rotation_lvlh = lvlh_rotation_to_eci.transpose();
}

void Orbit::J2Effect()
{
    double p = semi_major_axis * (1 - pow(eccentricity,2));
    double n0 = sqrt(kMUe/pow(semi_major_axis,3));

    rate_of_right_ascension = (-3.0/2.0)* kJ2 * (pow((kRe/p),2)) * (cos(inclination)) * n0;

    if (eccentricity <= 1.e-4)
    {
        rate_argument_of_perigee = 0 ;
    }else
    {
        rate_argument_of_perigee = (3.0 / 2.0) * kJ2 / eccentricity * pow((kRe/p),2) * (2.0 - 5.0 / 2.0 * pow(sin(inclination),2)) * n0;
    }
}
