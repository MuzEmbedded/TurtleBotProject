#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_movement");
    ros::NodeHandle nh;

    // Create a publisher to publish velocity commands
    ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("/turtlebot1/cmd_vel", 1000);

    // Seed for random number generation
    std::srand(std::time(0));

    ros::Rate rate(1);  // Set the publishing rate (1 Hz in this case)

    while (ros::ok()) {
        // Generate random linear and angular velocities
        double linearVel = (std::rand() % 10) / 10.0;  // Random value between 0 and 1
        double angularVel = (std::rand() % 10 - 5) / 5.0;  // Random value between -1 and 1

        // Create a Twist message with the random velocities
        geometry_msgs::Twist velMsg;
        velMsg.linear.x = linearVel;
        velMsg.angular.z = angularVel;

        // Publish the Twist message
        cmdVelPub.publish(velMsg);

        // Sleep to maintain the publishing rate
        rate.sleep();
    }

    return 0;
}
