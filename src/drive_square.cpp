#include <iostream>
#include <cmath>

#include <mbot_bridge/robot.h>
#include <wall_follower/common/utils.h>


int main(int argc, const char *argv[])
{
    // Initialize the robot.
    mbot_bridge::MBot robot;

    
    /**
     * TODO: (P1.1) Write code to make the robot drive in a square. Then,
     * modify your code so that the robot drives in a square 3 times.
     *
     * HINT: A function to send velocity commands to the robot is provided. To
     * use it, use the following code:
     *
     *      robot.drive(vx, vy, wz);
     *
     * Replace vx, vy, and wz with the velocity in the x direction (vx), y
     * direction (vy), and the angular velocity (wz). You can also use this code:
     *
     *      sleepFor(secs);
     *
     * to sleep for "secs" seconds (replace with desired number of seconds).
     * 
     * From John's understanding, vx is moving forward, vy is moving sideways, and wz is turning the robot (negative = clockwise; positive = counter-clockwise)
     */

    // Robot turns in square
    const double forward_v = 0.5;
    const double forward_t = 2.0;
    const double turn_w = 2.40;
    const double turn_t = 1.0;

    for(int i = 0; i < 4; i++){
        // Robot moves forward, the stops
        robot.drive(forward_v,0.0,0.0);
        sleepFor(forward_t);
        robot.stop();
        sleepFor(0.5);

        // Robot turns then stops
        robot.drive(0.0,0.0,turn_w);
        sleepFor(turn_t);
        robot.stop();
        sleepFor(0.5);
    }

    // Stop the robot.
    std::cout << "Stopping the robot!!" << std::endl;
    robot.stop();
    return 0;
}