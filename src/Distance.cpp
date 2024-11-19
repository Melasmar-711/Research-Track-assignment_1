#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include "turtlesim/Pose.h"

// Class to manage each turtle's pose
class Turtle {
public:
    ros::NodeHandle n;
    std::string turtle_name;
    float r[2] = {0.0, 0.0}; // Position vector [x, y]

    Turtle(ros::NodeHandle &nh, const std::string &name) {
        n = nh;
        turtle_name = name;

        ROS_ASSERT(!name.empty()); // Assert to ensure name is not empty
        std::string topic = "/" + turtle_name + "/pose";
        pose_subscriber = n.subscribe(topic, 10, &Turtle::pose_callback, this);
        ROS_INFO("Subscribed to %s", topic.c_str());
        n.setParam("/turtle_movement_status/" + turtle_name, true); // Initialize movement status
    }

    // Callback to update turtle's position
    void pose_callback(const turtlesim::Pose::ConstPtr &msg) {
        r[0] = msg->x; // Update x-coordinate
        r[1] = msg->y; // Update y-coordinate


        if (r[0]>10 || r[0] <1 ||r[1]>10||r[1]<1)
        {
        n.setParam("/turtle_movement_status/" +turtle_name, false);

        }
        ROS_INFO(" i am turtle %s my x:%f  y :%f", turtle_name.c_str(),r[0],r[1]);
    }

private:
    ros::Subscriber pose_subscriber;
};

// Helper function to calculate Euclidean distance between two turtles
float calculate_distance( Turtle* t1,  Turtle* &t2) {
    return std::sqrt(std::pow(t1->r[0] - t2->r[0], 2) + std::pow(t1->r[1] - t2->r[1], 2));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_turtle_manager");

    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    std::vector<Turtle*> turtles; // Vector of pointers to dynamically manage memory
    std::vector<std::string> previous_turtle_names;
    std::vector<std::string> current_turtle_names;

    double distance_threshold = 1.0; // Minimum allowed distance between turtles
    

    nh.getParam("turtle_names", current_turtle_names);
    

    while (ros::ok()) {
        // Check the current state of the "turtle_names" parameter
        if (nh.getParam("turtle_names", current_turtle_names)) {
            // Add new turtles
            for (const auto &name : current_turtle_names) {
                if (std::find(previous_turtle_names.begin(), previous_turtle_names.end(), name) == previous_turtle_names.end()) {
                    ROS_INFO("Adding turtle: %s", name.c_str());
                    Turtle* new_turtle= new Turtle(nh, name);
                    turtles.emplace_back(new_turtle);
                }
            }

            // Remove turtles that are no longer in the parameter
            for (auto it = turtles.begin(); it != turtles.end();) {
                if (std::find(current_turtle_names.begin(), current_turtle_names.end(), (*it)->turtle_name) == current_turtle_names.end()) {
                    ROS_INFO("Removing turtle: %s", (*it)->turtle_name.c_str());
                    delete *it; // Free memory
                    it = turtles.erase(it); // Remove from vector
                } else {
                    ++it;
                }
            }

            // Update the previous names list
            previous_turtle_names = current_turtle_names;
        }

        // Check distances between turtles and update movement status
        for (size_t i = 0; i < turtles.size(); i++) {
            bool can_move_i = true; // Assume turtle i can move

            for (size_t j = i + 1; j < turtles.size(); j++) {
                float distance = calculate_distance(turtles[i], turtles[j]);
                if (distance < distance_threshold) {
                    ROS_WARN("Turtles %s and %s are too close! Distance: %.2f",
                            turtles[i]->turtle_name.c_str(),
                            turtles[j]->turtle_name.c_str(),
                            distance);

                    // Stop both turtles
                    can_move_i = false;
                    nh.setParam("/turtle_movement_status/" + turtles[j]->turtle_name, false);
                }
            }

    // Update the movement status for turtle i
    nh.setParam("/turtle_movement_status/" + turtles[i]->turtle_name, can_move_i);
}



        ros::spinOnce();
        loop_rate.sleep();
    }

    // Clean up dynamically allocated turtles
    for (auto &turtle : turtles) {
        delete turtle;
    }

    return 0;
}
