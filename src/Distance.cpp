/*

A node that checks the relative distance between turtle1 and turtle2 and:
- publish on a topic the distance (you can use a std_msgs/Float32 for
that)
- stops the moving turtle if the two turtles are “too close” (you may
set a threshold to monitor that)
- stops the moving turtle if the position is too close to the boundaries

*/




#include <string>
#include "ros/ros.h"
#include "turtlesim/Kill.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"




class Turtle{

public:




std::string turtle_name;
float x;
float y;
float theta;
ros::NodeHandle n;

void kill();
void move_or_not_turtle(float V_x ,float Theta_dot,int timer);
void get_pose_callback(const turtlesim::Pose::ConstPtr& msg);


Turtle(ros::NodeHandle &n_,std::string name ){
turtle_name=name;
n=n_;
std::string topic="/"+turtle_name+"pose";
ros::Subscriber pose_subscriber=n.subscribe(topic,1000,&Turtle::get_pose_callback,this);
}
private:
geometry_msgs::Twist turtle_cmd_vel ;
};

//-----------------------------------------------------------------------------------------//
// methods of the Class Turtle 

void Turtle::get_pose_callback(const turtlesim::Pose::ConstPtr& msg){

x = msg->x;
y = msg->y;
theta = msg->theta;

}


/*this will move the turle for any amount of time the user chooses */
void Turtle::move_or_not_turtle(float V_x ,float Theta_dot,int timer){
    
}









int main(int argc ,char **argv)

{


    ros::init(argc,argv,"tutrtlesim_excercise_1");

    ros::NodeHandle nh;








}
