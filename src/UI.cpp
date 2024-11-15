/*

This node 

-Spawns a new turtle in the environment: turtle2

-Implements a simple textual interface to retrieve the user command
(i.e., you can use cin (c++) or input (python). The user should be able to
select the robot they want to control (turtle1 or turtle2), and the velocity
of the robot.

-The command should be sent for 1 second, and then the robot should
stop, and the user should be able again to insert the command.

*/

#include <string>
#include "ros/ros.h"
#include "turtlesim/Kill.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"



/*intializing some global variables and objects*/



class Turtle{

public:




std::string turtle_name;
_Float32 x;
_Float32 y;
_Float32 theta;
ros::NodeHandle *n;


Turtle(ros::NodeHandle *n_,std::string name ,_Float32 x_pos ,_Float32 y_pos, _Float32 theta_){


turtle_name=name;
x=x_pos;
y=y_pos;
theta=theta_;
n=n_;
Spawn_turt.request.x=x_pos;
Spawn_turt.request.y=y_pos;
Spawn_turt.request.theta=theta_;
Spawn_turt.request.name=turtle_name;
spawn();

}

void kill();
void move_turtle(_Float32 V_x ,_Float32 Theta_dot,int timer);


private:

turtlesim::Kill   kill_turt ;
turtlesim::Spawn   Spawn_turt ;
geometry_msgs::Twist turtle_cmd_vel ;

void spawn();

};




//-----------------------------------------------------------------------------------------//



// methods of the Class Turtle 



/* Spawn which cannot be called by the user fearing the user might use the same object to create multiple turtle 
which completly destroy what we are trying to acheive 
*/
void Turtle::spawn(){

    static ros::ServiceClient spawn_client = n->serviceClient<turtlesim::Spawn>("/spawn");
    spawn_client.call(Spawn_turt);

}



/*this kill the turtle if the user chooses to (-_-) */

void Turtle::kill(){

    static ros::ServiceClient  kill_client= n->serviceClient<turtlesim::Kill>("/kill");
    kill_turt.request.name=turtle_name;
    kill_client.call(kill_turt);

}

/*this will move the turle for any amount of time the user chooses */
void Turtle::move_turtle(_Float32 V_x ,_Float32 Theta_dot,int timer){
    std::string turtle_velocity_commander = "/" + turtle_name + "/cmd_vel";

    ros::Publisher cmd_vel =n->advertise<geometry_msgs::Twist>(turtle_velocity_commander,10);



    turtle_cmd_vel.linear.x=V_x;
    turtle_cmd_vel.linear.y=0;
    turtle_cmd_vel.linear.z=0;
    turtle_cmd_vel.angular.x=0;
    turtle_cmd_vel.angular.y=0;
    turtle_cmd_vel.angular.z=Theta_dot;
    ros::Time start_time = ros::Time::now();

   while(ros::ok()){ 

    if ((ros::Time::now() - start_time).toSec() < timer) {
            // Publish the velocity message for the first second

            std::cout<<"this is what i am publishing and the topic is  \t"<<turtle_velocity_commander<<"\t"<<turtle_cmd_vel.linear.x<<"\n";
            cmd_vel.publish(turtle_cmd_vel);
        } 
        else {
            // After 1 second, stop the turtle by publishing zero velocity
            turtle_cmd_vel.linear.x = 0.0;
            turtle_cmd_vel.angular.z = 0.0;
            cmd_vel.publish(turtle_cmd_vel);
            break;  // Exit the loop after stopping the turtle
        }    
    
    

   }
}





/*---------------------------------------------------------------------------------------------------------------------------*/







int main(int argc , char **argv)
{
    ros::init(argc,argv,"tutrtlesim_excercise_1");

    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    std::vector<Turtle> turtles;
    Turtle turtle_1= Turtle(&nh,"turtle1",1,1,0);
    turtles.push_back(turtle_1);
    Turtle turtle_2= Turtle(&nh,"turtle2",1,4,0);
    turtles.push_back(turtle_2);

    struct turtle_attr
    {
        std::string name;
        _Float32 x;
        _Float32 y;
        _Float32 theta;
    }new_turtle ;
    



    int choice=0;
    _Float32 speeds[2]={};
    int timer_to_move=0;

    

while(ros::ok()){


    std::cout<<"0-exit\n1-control turtle\n2-kill turtle\n3-add turtle\n";
    std::cin>>choice;



    if (choice==0){
        break;
    }
    if(choice == 2){

    /* choose the number of the turtle which is not an efficient way given this is a vector and we kill one
    the others shift and the indices change but as it's not really our scope to implement a function 
    that searches for the turtle by it's name this shall work given the user is careful what he writes*/    
    std::cout<<"which turtle to kill : ";
    std::cin>>choice;
        if (choice > 0 && choice <= turtles.size()) {
                turtles[choice - 1].kill();
                turtles.erase(turtles.begin() + choice - 1);
            } else {
                std::cerr << "Invalid turtle selection.\n";
            }

    }
    





    else if(choice==1){

    /*this will move the turtle of the number chosen */
    std::cout<<"which turtle to move : ";
    std::cin>>choice;
    std::cout<<"Enter linear velocity &angular velocity & time to move:\n";
    std::cin>>speeds[0]>>speeds[1]>>timer_to_move;
    turtles[choice-1].move_turtle(speeds[0],speeds[1],timer_to_move);

    }



    else if(choice ==3){
    std::cout<<"enter the name of the new turtle and x,y,theta :\n";
    std::cin>>new_turtle.name>>new_turtle.x>>new_turtle.y>>new_turtle.theta;

    //Turtle new_turtle_=Turtle();
    turtles.emplace_back(&nh,new_turtle.name,new_turtle.x,new_turtle.y,new_turtle.theta);

    }
        std::vector<std::string> turtle_names;
        for (const auto& turtle : turtles) {
            turtle_names.push_back(turtle.turtle_name);
        }
        nh.setParam("turtle_names", turtle_names);

        ros::spinOnce();
        loop_rate.sleep();


    nh.setParam("turtle_names",turtle_names);


    ros::spinOnce();
    loop_rate.sleep();
    }



return 0;

}







    



