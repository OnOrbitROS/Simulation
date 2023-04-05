#include "iostream"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

const int POINTS = 5; 

struct trajectory {
 double timIni;
 double timFin;
 double posIni;
 double posFin;  
 double deltaY; 
 bool clacOk;  
};

int timeRange(trajectory trajectory[], double timNow)
{
    int pos = POINTS -1;
    for (int i = 0; i < POINTS ; i++)
    {
        if ( timNow < trajectory[i].timFin )
        {
            pos = i ; 
            i = POINTS;
        }
    }
    return pos; 
}

void calcPendiente(trajectory &traj)
{
    if (traj.timFin > traj.timIni)
    {
        traj.deltaY = (traj.posFin- traj.posIni) / (traj.timFin-traj.timIni) ;
        traj.clacOk = true; 
    }
}

double calcPosition (trajectory trajectory[], double timNow)
{ 
    double pos = 0.0;

    int i = timeRange(trajectory , timNow);
    std::cout << "valor de i" << i << std::endl ; 

    if (! trajectory[i].clacOk)
    {
        calcPendiente(trajectory[i]);
    }
    pos = ( timNow - trajectory[i].timIni) * trajectory[i].deltaY + trajectory[i].posIni ;
    std::cout << "posicion" << pos << std::endl ;    
    return pos;
}

int main(int argc, char **argv)
{

    trajectory joint1_trajectory[POINTS];
    trajectory joint2_trajectory[POINTS];
    trajectory joint3_trajectory[POINTS];
    trajectory joint4_trajectory[POINTS];
    trajectory joint5_trajectory[POINTS];
    trajectory joint6_trajectory[POINTS];

    //joint1_trajectory initializing
    joint1_trajectory[0].timIni = 0.0;
    joint1_trajectory[0].timFin = 682.5;
    joint1_trajectory[0].posIni = 0.0;
    joint1_trajectory[0].posFin = 0.0;

    joint1_trajectory[1].timIni = 682.5;
    joint1_trajectory[1].timFin = 705.1;
    joint1_trajectory[1].posIni = 0.0;
    joint1_trajectory[1].posFin = 0.0;
    
    joint1_trajectory[2].timIni = 705.1;
    joint1_trajectory[2].timFin = 744.5;
    joint1_trajectory[2].posIni = 0.0;
    joint1_trajectory[2].posFin = 0.0;
    
    joint1_trajectory[3].timIni = 744.5;
    joint1_trajectory[3].timFin = 767.1;
    joint1_trajectory[3].posIni = 0.0;
    joint1_trajectory[3].posFin = 0.0;
    
    joint1_trajectory[4].timIni = 767.1;
    joint1_trajectory[4].timFin = 850.0;
    joint1_trajectory[4].posIni = 0.0;
    joint1_trajectory[4].posFin = 0.0;
    
    //joint2_trajectory initializing
    joint2_trajectory[0].timIni = 0.0;
    joint2_trajectory[0].timFin = 682.5;
    joint2_trajectory[0].posIni = 0.0;
    joint2_trajectory[0].posFin = 0.0;

    joint2_trajectory[1].timIni = 682.5;
    joint2_trajectory[1].timFin = 705.1;
    joint2_trajectory[1].posIni = 0.0;
    joint2_trajectory[1].posFin = 0.263828840417748;
    
    joint2_trajectory[2].timIni = 705.1;
    joint2_trajectory[2].timFin = 744.5;
    joint2_trajectory[2].posIni = 0.263828840417748;
    joint2_trajectory[2].posFin = 0.263828840417748;
    
    joint2_trajectory[3].timIni = 744.5;
    joint2_trajectory[3].timFin = 767.1;
    joint2_trajectory[3].posIni = 0.263828840417748;
    joint2_trajectory[3].posFin = 0.0;
    
    joint2_trajectory[4].timIni = 767.1;
    joint2_trajectory[4].timFin = 850.0;
    joint2_trajectory[4].posIni = 0.0;
    joint2_trajectory[4].posFin = 0.0;
    
    //joint3_trajectory initializing
    joint3_trajectory[0].timIni = 0.0;
    joint3_trajectory[0].timFin = 682.5;
    joint3_trajectory[0].posIni = 0.0;
    joint3_trajectory[0].posFin = 0.0;

    joint3_trajectory[1].timIni = 682.5;
    joint3_trajectory[1].timFin = 705.1;
    joint3_trajectory[1].posIni = 0.0;
    joint3_trajectory[1].posFin = -0.320653513738493;
    
    joint3_trajectory[2].timIni = 705.1;
    joint3_trajectory[2].timFin = 744.5;
    joint3_trajectory[2].posIni = -0.320653513738493;
    joint3_trajectory[2].posFin = -0.324712418975689;
    
    joint3_trajectory[3].timIni = 744.5;
    joint3_trajectory[3].timFin = 767.1;
    joint3_trajectory[3].posIni = -0.324712418975689;
    joint3_trajectory[3].posFin = 0.0;
    
    joint3_trajectory[4].timIni = 767.1;
    joint3_trajectory[4].timFin = 850.0;
    joint3_trajectory[4].posIni = 0.0;
    joint3_trajectory[4].posFin = 0.0;
    
    //joint4_trajectory initializing
    joint4_trajectory[0].timIni = 0.0;
    joint4_trajectory[0].timFin = 682.5;
    joint4_trajectory[0].posIni = 0.0;
    joint4_trajectory[0].posFin = 0.0;

    joint4_trajectory[1].timIni = 682.5;
    joint4_trajectory[1].timFin = 705.1;
    joint4_trajectory[1].posIni = 0.0;
    joint4_trajectory[1].posFin = 0.0568246733207494;
    
    joint4_trajectory[2].timIni = 705.1;
    joint4_trajectory[2].timFin = 744.5;
    joint4_trajectory[2].posIni = 0.0568246733207494;
    joint4_trajectory[2].posFin = 0.0568246733207494;
    
    joint4_trajectory[3].timIni = 744.5;
    joint4_trajectory[3].timFin = 767.1;
    joint4_trajectory[3].posIni = 0.0568246733207494;
    joint4_trajectory[3].posFin = 0.0;
    
    joint4_trajectory[4].timIni = 767.1;
    joint4_trajectory[4].timFin = 850.0;
    joint4_trajectory[4].posIni = 0.0;
    joint4_trajectory[4].posFin = 0.0;

    //joint5_trajectory initializing
    joint5_trajectory[0].timIni = 0.0;
    joint5_trajectory[0].timFin = 682.5;
    joint5_trajectory[0].posIni = 0.0;
    joint5_trajectory[0].posFin = 0.0;

    joint5_trajectory[1].timIni = 682.5;
    joint5_trajectory[1].timFin = 705.1;
    joint5_trajectory[1].posIni = 0.0;
    joint5_trajectory[1].posFin = 0.0;
    
    joint5_trajectory[2].timIni = 705.1;
    joint5_trajectory[2].timFin = 744.5;
    joint5_trajectory[2].posIni = 0.0;
    joint5_trajectory[2].posFin = 0.0;
    
    joint5_trajectory[3].timIni = 744.5;
    joint5_trajectory[3].timFin = 767.1;
    joint5_trajectory[3].posIni = 0.0;
    joint5_trajectory[3].posFin = 0.0;
    
    joint5_trajectory[4].timIni = 767.1;
    joint5_trajectory[4].timFin = 850.0;
    joint5_trajectory[4].posIni = 0.0;
    joint5_trajectory[4].posFin = 0.0;

    //joint6_trajectory initializing
    joint6_trajectory[0].timIni = 0.0;
    joint6_trajectory[0].timFin = 682.5;
    joint6_trajectory[0].posIni = 0.0;
    joint6_trajectory[0].posFin = 0.0;

    joint6_trajectory[1].timIni = 682.5;
    joint6_trajectory[1].timFin = 705.1;
    joint6_trajectory[1].posIni = 0.0;
    joint6_trajectory[1].posFin = 0.0;
    
    joint6_trajectory[2].timIni = 705.1;
    joint6_trajectory[2].timFin = 744.5;
    joint6_trajectory[2].posIni = 0.0;
    joint6_trajectory[2].posFin = 0.0;
    
    joint6_trajectory[3].timIni = 744.5;
    joint6_trajectory[3].timFin = 767.1;
    joint6_trajectory[3].posIni = 0.0;
    joint6_trajectory[3].posFin = 0.0;
    
    joint6_trajectory[4].timIni = 767.1;
    joint6_trajectory[4].timFin = 850.0;
    joint6_trajectory[4].posIni = 0.0;
    joint6_trajectory[4].posFin = 0.0;

    // ros::Time::now().toSec();

    ros::init(argc, argv, "trajectory");

    ros::NodeHandle n;

    ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("joint1_position_controller/command", 1000);
    ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("joint2_position_controller/command", 1000);
    ros::Publisher joint3_pub = n.advertise<std_msgs::Float64>("joint3_position_controller/command", 1000);
    ros::Publisher joint4_pub = n.advertise<std_msgs::Float64>("joint4_position_controller/command", 1000);
    ros::Publisher joint5_pub = n.advertise<std_msgs::Float64>("joint5_position_controller/command", 1000);
    ros::Publisher joint6_pub = n.advertise<std_msgs::Float64>("joint6_position_controller/command", 1000);


    //Frecuencia. 
    ros::Rate loop_rate(50);
    
    std_msgs::Float64 joint1;
    std_msgs::Float64 joint2;
    std_msgs::Float64 joint3;
    std_msgs::Float64 joint4;
    std_msgs::Float64 joint5;
    std_msgs::Float64 joint6;
    double time; 

    ROS_INFO("Iniciando control de trayectorias");
    while (ros::ok())
    {
        time = ros::Time::now().toSec(); 
        std::cout << "Calculando trajectoria 1" << std::endl ;
        joint1.data = calcPosition(joint1_trajectory, time );
        std::cout << "Calculando trajectoria 2" << std::endl ;
        joint2.data = calcPosition(joint2_trajectory, time );
        std::cout << "Calculando trajectoria 3" << std::endl ;
        joint3.data = calcPosition(joint3_trajectory, time );
        std::cout << "Calculando trajectoria 4" << std::endl ;
        joint4.data = calcPosition(joint4_trajectory, time );
        std::cout << "Calculando trajectoria 5" << std::endl ;
        joint5.data = calcPosition(joint5_trajectory, time );
        std::cout << "Calculando trajectoria 6" << std::endl ;
        joint6.data = calcPosition(joint6_trajectory, time );
       

        //ROS_INFO("f%",value.data);
        joint1_pub.publish(joint1);
        joint2_pub.publish(joint2);
        joint3_pub.publish(joint3);
        joint4_pub.publish(joint4);
        joint5_pub.publish(joint5);
        joint6_pub.publish(joint6);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}