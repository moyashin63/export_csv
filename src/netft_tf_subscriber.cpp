#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <string>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <stdio.h>

double Fx, Fz, My, x, y, z, roll, pitch, yaw;
const char* FileName;

void netftCallback(const geometry_msgs::WrenchStamped& netft){
    Fx = netft.wrench.force.x;
    Fz = netft.wrench.force.z;
    My = netft.wrench.torque.y;
}

/*------ Quaternion --> Euler  rad --> deg ------*/
void QuartaniontoEuler(double qx, double qy, double qz, double qw){
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    roll = roll * ( 180 / M_PI );
    pitch = pitch * (180 / M_PI );
    yaw = yaw * (180 / M_PI );
}

/*------ write CSV ------*/
int saveCSV(unsigned int sec, unsigned int nsec){
    std::ofstream ofs(FileName, std::ios::app);
    if (!ofs){
        std::cout << "file open error" << std::endl;
        return 0;
    }
    ofs << sec << "." << nsec << "," << x <<","<< y << ","<< z << ","<< roll << ","<< pitch << ","<< yaw << ","<< Fx << ","<< Fz << ","<< My << "," << std::endl;
}

int main(int argc, char** argv){
    //------ csv file ------
    time_t timer;
    struct tm* tm;
    char datetime[20];
    timer = time(NULL);
    tm = localtime(&timer);
    strftime(datetime, 20, "%Y%m%d%H%M", tm);
    std::string tempstr = "/home/hcm/" + std::string(datetime) + ".csv";
    const char* dt = tempstr.c_str();
    FileName = dt;
    std::ofstream firstline(FileName);
    firstline << "t,x,y,z,roll,pitch,yaw,Fx,Fz,My" << std::endl;

    //------ main ------
    ros::init(argc, argv, "netft_tf_subscriber");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Subscriber netft_sub = nh.subscribe("netft_data", 100, netftCallback);
    tf::TransformListener tflistener;

    ros::WallTime wall_begin = ros::WallTime::now();

    while (ros::ok()){
        geometry_msgs::PoseStamped source_pose;
        source_pose.header.frame_id="bucket_link";
        source_pose.pose.orientation.w=1.0;
        geometry_msgs::PoseStamped target_pose;
        try{
            tflistener.waitForTransform("base_link", "bucket_link", ros::Time(0), ros::Duration(8.0));
            tflistener.transformPose("base_link",ros::Time(0),source_pose,"bucket_link",target_pose);
        }
        catch(...){
            ROS_INFO("tf error");
        }
        ros::spinOnce();
        loop_rate.sleep();

        x = target_pose.pose.position.x;
        y = target_pose.pose.position.y;
        z = target_pose.pose.position.z;

        /*------ Quaternion --> Euler  rad --> deg  local-> world ------*/
        QuartaniontoEuler(target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);

        //add func Fx and Fz from local coordinate to world coordinate

        ros::WallDuration wall_duration = ros::WallTime::now() - wall_begin;
        saveCSV(wall_duration.sec, wall_duration.nsec);
    }
    return 0;
}
