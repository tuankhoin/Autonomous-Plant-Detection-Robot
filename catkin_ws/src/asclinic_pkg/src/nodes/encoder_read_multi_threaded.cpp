// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
//
// This file is part of ASClinic-System.
//    
// See the root of the repository for license details.
//
// ----------------------------------------------------------------------------
//     _    ____   ____ _ _       _          ____            _                 
//    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
//   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
//  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
// /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// Node for monitoring GPIO pins connected to wheel encoders
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>

#include <gpiod.h>

#include <cmath>
#include <iostream>
// Includes required for threading
// > Mutex for handling shared variable
#include <mutex>
// > Include only one of the following two options:
#include <thread>
//#include <boost/thread/thread.hpp>

// Include the asclinic message types
#include "asclinic_pkg/LeftRightInt32.h"
#include "asclinic_pkg/LeftRightFloat32.h"

// find modulo of floating point numbers using library function.
#include <bits/stdc++.h>
using namespace std;

#include <cstdlib>

// Namespacing the package
//using namespace asclinic_pkg;





// MEMBER VARIABLES FOR THIS NODE:
// > Publisher and timer for the current
//   encoder counts
ros::Publisher m_encoder_counts_publisher;
ros::Timer m_timer_for_publishing;

// > Publisher for the current pose
ros::Publisher m_pose_publisher;

// > Publisher for the measured wheel velocities
ros::Publisher m_measured_wheel_velocity_publisher;

// > Publisher for the target wheel velocities
ros::Publisher m_target_wheel_velocity_publisher;

// > For sharing the encoder counts between nodes
int m_encoder_counts_for_motor_left_a  = 0;
int m_encoder_counts_for_motor_left_b  = 0;
int m_encoder_counts_for_motor_right_a = 0;
int m_encoder_counts_for_motor_right_b = 0;

// > Mutex for preventing multiple-access of shared variables
std::mutex m_counting_mutex;

// > The "gpiochip" number
int m_gpiochip_number = 1;

// > The line numbers to read
int m_line_number_for_motor_left_channel_a  = 105;
int m_line_number_for_motor_left_channel_b  = 106;
int m_line_number_for_motor_right_channel_a =  84;
int m_line_number_for_motor_right_channel_b = 130;

// > Boolean flag for when to stop counting
bool encoder_thread_should_count = true;

// > The "delta t" used for the frequency of publishing encoder counts
float m_delta_t_for_publishing_counts = 0.1;

// > NOTE: The following variables are purely for
//   the convenience of testing.
float m_time_in_seconds_to_drive_motors = 1.0;
ros::Publisher m_motor_pwm_publisher;
float m_drive_motor_target_speed = 20.0;

// Target left and right velocities
float target_velocity_left = 0.0;
float target_velocity_right = 0.0;

// PID variables
float kp_left = 5;
float kp_right = 5;
float ki_left = 13;
float ki_right = 10;
float e_left = 0;
float e_right = 0;
float u_left = 0;
float u_right = 0;
float p_left = 0;
float p_right = 0;
float i_left = 0;
float i_right = 0;
float lim_min = -100;
float lim_max = 100;
float lim_min_i_left;
float lim_max_i_left;
float lim_min_i_right;
float lim_max_i_right;

// Odometry update constants 
float r = 0.07176; // metres
float b = 0.2157; // metres
float cpr = 1120; // counts per revolution


// Respond to timer callback
void timerCallbackForPublishing(const ros::TimerEvent&)
{
	// Get the current counts into a local variable
	// > And reset the shared counts variable to zero
	int counts_motor_left_a_local_copy;
	int counts_motor_left_b_local_copy;
	int counts_motor_right_a_local_copy;
	int counts_motor_right_b_local_copy;
	m_counting_mutex.lock();
	counts_motor_left_a_local_copy  = m_encoder_counts_for_motor_left_a;
	counts_motor_left_b_local_copy  = m_encoder_counts_for_motor_left_b;
	counts_motor_right_a_local_copy = m_encoder_counts_for_motor_right_a;
	counts_motor_right_b_local_copy = m_encoder_counts_for_motor_right_b;
	m_encoder_counts_for_motor_left_a  = 0;
	m_encoder_counts_for_motor_left_b  = 0;
	m_encoder_counts_for_motor_right_a = 0;
	m_encoder_counts_for_motor_right_b = 0;
	m_counting_mutex.unlock();

	// Publish encoder message
	asclinic_pkg::LeftRightInt32 encoder_msg;
	encoder_msg.left  = counts_motor_left_a_local_copy;//  + counts_motor_left_b_local_copy;
	encoder_msg.right = counts_motor_right_a_local_copy;// + counts_motor_right_b_local_copy;
	m_encoder_counts_publisher.publish(encoder_msg);

	// Pose message
	geometry_msgs::Twist pose_msg;
	// Odometry update
	static float delta_s;
	static float delta_phi;
	static float x = 0.0;
	static float y = 0.0;
	static float phi = 0.0;
	static float delta_theta_l;
	static float delta_theta_r;
	delta_theta_l = (float(encoder_msg.left)/float(cpr))*2*M_PI;
	delta_theta_r = (float(encoder_msg.right)/float(cpr))*2*M_PI;
	delta_s = (r/2)*(delta_theta_l + delta_theta_r);
	delta_phi = (r/b)*(delta_theta_r - delta_theta_l);
	x += delta_s*cos(phi + delta_phi/2);
	y += delta_s*sin(phi + delta_phi/2);
	phi += delta_phi;
	// Pose message
	pose_msg.linear.x  = x;
	pose_msg.linear.y = y;
	// Range from 0 to 2*pi
	if (phi < 0) {
		pose_msg.angular.z  = (360 - fmod((abs(phi)/(M_PI))*180, 360.0)) * M_PI/180;
	} else {
		pose_msg.angular.z  = (fmod((phi/(M_PI))*180, 360.0) * M_PI/180);
	}
	// Range from -pi (exclusive) to pi (inclusive)
	if (pose_msg.angular.z <= M_PI) {
		pose_msg.angular.z = pose_msg.angular.z;
	} else if (pose_msg.angular.z > M_PI) {
		pose_msg.angular.z = pose_msg.angular.z - 2*M_PI;
	}
	// Publish pose message
	m_pose_publisher.publish(pose_msg);


	// Measured wheel velocity (radians per second)
	asclinic_pkg::LeftRightFloat32 measured_wheel_velocity_msg;
	measured_wheel_velocity_msg.left = delta_theta_l / m_delta_t_for_publishing_counts;
	measured_wheel_velocity_msg.right = delta_theta_r / m_delta_t_for_publishing_counts;
	// Publish measured wheel velocity
	m_measured_wheel_velocity_publisher.publish(measured_wheel_velocity_msg);




	// Compute PID control signal u (PWM)
	// Taken from: https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl/SpeedControl.ino
	e_left = target_velocity_left - measured_wheel_velocity_msg.left;
	e_right = target_velocity_right - measured_wheel_velocity_msg.right;
	ROS_INFO_STREAM("Left error = " << e_left << ", Right error = " << e_right);

	// Proportional component of PID
	p_left = kp_left * e_left;
	p_right = kp_right * e_right;

	// Integral component of PID
	i_left += ki_left * e_left * m_delta_t_for_publishing_counts;
	i_right += ki_right * e_right * m_delta_t_for_publishing_counts;


    // Anti-wind-up via Dynamic Integrator Clamping
	// Taken from: https://github.com/BanaanKiamanesh/Anti-Windup-PID-Controller/blob/main/PID.cpp
    
	// Left Anti-wind-up
	if (lim_max > p_left)
        lim_max_i_left = lim_max - p_left;
    else
        lim_max_i_left = 0.0f;
    if (lim_min < p_left)
        lim_min_i_left = lim_min - p_left;
    else
        lim_min_i_left = 0.0f;
    if (i_left > lim_max_i_left)
        i_left = lim_max_i_left;
    else if (i_left < lim_min_i_left)
        i_left = lim_min_i_left;

	// Right Anti-wind-up
	if (lim_max > p_right)
        lim_max_i_right = lim_max - p_right;
    else
        lim_max_i_right = 0.0f;
    if (lim_min < p_right)
        lim_min_i_right = lim_min - p_right;
    else
        lim_min_i_right = 0.0f;
    if (i_right > lim_max_i_right)
        i_right = lim_max_i_right;
    else if (i_right < lim_min_i_right)
        i_right = lim_min_i_right;


	// Control signal u
	u_left = p_left + i_left;
	u_right = p_right + i_right; 

	// Publish message to run the motors
	asclinic_pkg::LeftRightFloat32 pwm_msg;
	// Crosstalk workaround in case one motor is not supposed to be moving but encoder is incrementing/decrementing due to interference
	if (target_velocity_left == 0) {
		pwm_msg.left = 0;
	} else {
		pwm_msg.left = u_left;
	}
	if (target_velocity_right == 0) {
		pwm_msg.right = 0;
	} else {
		pwm_msg.right = u_right;
	}
	m_motor_pwm_publisher.publish(pwm_msg);





	// NOTE: the remainder of this function is
	// purely for the convenience of testing.
	static bool did_start_motors = false;
	static bool did_break = false;
	static bool did_reverse = false;
	static bool did_finish_test = false;
	static bool did_display_cum_sum = false;
	static float elapsed_time_in_seconds = 0.0;
	static int cum_sum_left_a = 0;
	static int cum_sum_left_b = 0;
	static int cum_sum_right_a = 0;
	static int cum_sum_right_b = 0;
	// Add the counts
	cum_sum_left_a  += counts_motor_left_a_local_copy;
	cum_sum_left_b  += counts_motor_left_b_local_copy;
	cum_sum_right_a += counts_motor_right_a_local_copy;
	cum_sum_right_b += counts_motor_right_b_local_copy;
	// Increment the time
	elapsed_time_in_seconds += m_delta_t_for_publishing_counts;
	// Start the motors after a few seconds
	if ( !(did_start_motors) && (elapsed_time_in_seconds>=2.0) )
	{
		// Publish message to start the motors
		asclinic_pkg::LeftRightFloat32 target_speed_msg;
		//target_speed_msg.left  = m_drive_motor_target_speed;
		//target_speed_msg.right = m_drive_motor_target_speed;
		//m_motor_pwm_publisher.publish(target_speed_msg);
		// Update the flag
		did_start_motors = true;
	}
	// Take a break after time to drive motors
	if ( !(did_break) && (elapsed_time_in_seconds>=(2.0+m_time_in_seconds_to_drive_motors)) )
	{
		// Publish message to stop the motors
		asclinic_pkg::LeftRightFloat32 target_speed_msg;
		target_speed_msg.left  = 0.0;
		target_speed_msg.right = 0.0;
		//m_motor_pwm_publisher.publish(target_speed_msg);
		// Update the flag
		did_break = true;
	}
	// Reverse the motors after 1 second of breaking
	if ( !(did_reverse) && (elapsed_time_in_seconds>=(2.0+m_time_in_seconds_to_drive_motors+1.0)) )
	{
		// Publish message to start the motors
		asclinic_pkg::LeftRightFloat32 target_speed_msg;
		//target_speed_msg.left  = - m_drive_motor_target_speed;
		//target_speed_msg.right = - m_drive_motor_target_speed;
		//m_motor_pwm_publisher.publish(target_speed_msg);
		// Update the flag
		did_reverse = true;
	}
	// Stop the motors after time to drive motors
	if ( !(did_finish_test) && (elapsed_time_in_seconds>=(2.0+m_time_in_seconds_to_drive_motors+1.0+m_time_in_seconds_to_drive_motors)) )
	{
		// Publish message to stop the motors
		asclinic_pkg::LeftRightFloat32 target_speed_msg;
		target_speed_msg.left  = 0.0;
		target_speed_msg.right = 0.0;
		//m_motor_pwm_publisher.publish(target_speed_msg);
		// Update the flag
		did_finish_test = true;
	}
	// Display the cumulative cum
	if ( !(did_display_cum_sum) && (elapsed_time_in_seconds>=(2.0+2.0+m_time_in_seconds_to_drive_motors)) )
	{
		//ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] cumulative sum left (A,B) = ( " << cum_sum_left_a << " , " << cum_sum_left_b << " ), right (A,B) = ( " << cum_sum_right_a << " , " << cum_sum_right_b << " )");
		//ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] cumulative sum left (A+B) = ( " << cum_sum_left_a + cum_sum_left_b << " ), right (A+B) = ( " << cum_sum_right_a + cum_sum_right_b << " )");
		ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] cumulative sum left (A+B) = ( " << cum_sum_left_a << " ), right (A+B) = ( " << cum_sum_right_a << " )");
		// Update the flag
		//did_display_cum_sum = true;
		// Update the flag to end the encoder thread
		//encoder_thread_should_count = false;
	}
}


void targetRobotVelocitySubscriberCallback(const geometry_msgs::Twist& msg)
{
	ROS_INFO_STREAM("[I2C FOR MOTORS] Message received with linear velocity = " << msg.linear.x << ", angular velocity = " << msg.angular.z);

	// Convert robot velocity to wheel velocity
	target_velocity_left = (msg.linear.x/r) - (b*msg.angular.z)/(2.0*r);
	target_velocity_right = (msg.linear.x/r) + (b*msg.angular.z)/(2.0*r);

	// Target wheel velocity (radians per second)
	asclinic_pkg::LeftRightFloat32 target_wheel_velocity_msg;
	target_wheel_velocity_msg.left = target_velocity_left;
	target_wheel_velocity_msg.right = target_velocity_right;
	// Publish target wheel velocity
	m_target_wheel_velocity_publisher.publish(target_wheel_velocity_msg);

	// Clip the data to be in the range [-100.0,100.0]
	// > For the left value
/* 	if (target_velocity_left < -100.0f)
		target_velocity_left = -100.0f;
	if (target_velocity_left > 100.0f)
		target_velocity_left = 100.0f;
	// > For the right value
	if (target_velocity_right < -100.0f)
		target_velocity_right = -100.0f;
	if (target_velocity_right > 100.0f)
		target_velocity_right = 100.0f; */

}


void encoderCountingThreadMain()
{
	// Specify the chip name of the GPIO interface
	// > Note: for the 40-pin header of the Jetson SBCs, this
	//   is "/dev/gpiochip1"
	std::stringstream temp_string_stream;
	temp_string_stream << "/dev/gpiochip" << m_gpiochip_number;
	const char * gpio_chip_name = temp_string_stream.str().c_str();

	// Make a local copy of the line number member variables
	int line_number_left_a  = m_line_number_for_motor_left_channel_a;
	int line_number_left_b  = m_line_number_for_motor_left_channel_b;
	int line_number_right_a = m_line_number_for_motor_right_channel_a;
	int line_number_right_b = m_line_number_for_motor_right_channel_b;

	// Initialise a GPIO chip, line, and event objects
	struct gpiod_chip *chip;
	struct gpiod_line *line_left_a;
	struct gpiod_line *line_left_b;
	struct gpiod_line *line_right_a;
	struct gpiod_line *line_right_b;
	struct gpiod_line_bulk line_bulk;
	struct gpiod_line_event event;
	struct gpiod_line_bulk event_bulk;

	// Specify the timeout specifications
	// > The first entry is seconds
	// > The second entry is nano-seconds
	struct timespec timeout_spec = { 0, 10000000 };
	
	// Intialise a variable for the flags returned
	// by GPIO calls
	int returned_wait_flag;
	int returned_read_flag;

	// Get and print the value of the GPIO line
	// > Note: the third argument to "gpiod_ctxless_get_value"
	//   is an "active_low" boolean input argument.
	//   If true, this indicate to the function that active state
	//   of this line is low.
	int value;
	// > For left motor channel A
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_a, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_a << " returned value = " << value);
	// > For left motor channel B
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_b, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_b << " returned value = " << value);
	// > For right motor channel A
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_a, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_a << " returned value = " << value);
	// > For right motor channel B
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_b, false, "foobar");
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_b << " returned value = " << value);

	// Open the GPIO chip
	chip = gpiod_chip_open(gpio_chip_name);
	// Retrieve the GPIO lines
	line_left_a  = gpiod_chip_get_line(chip,line_number_left_a);
	line_left_b  = gpiod_chip_get_line(chip,line_number_left_b);
	line_right_a = gpiod_chip_get_line(chip,line_number_right_a);
	line_right_b = gpiod_chip_get_line(chip,line_number_right_b);
	// Initialise the line bulk
	gpiod_line_bulk_init(&line_bulk);
	// Add the lines to the line bulk
	gpiod_line_bulk_add(&line_bulk, line_left_a);
	gpiod_line_bulk_add(&line_bulk, line_left_b);
	gpiod_line_bulk_add(&line_bulk, line_right_a);
	gpiod_line_bulk_add(&line_bulk, line_right_b);

	// Display the status
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Chip " << gpio_chip_name << " opened and lines " << line_number_left_a << ", " << line_number_left_b << ", " << line_number_right_a << " and " << line_number_right_b << " retrieved");

	// Request the line events to be mointored
	// > Note: only one of these should be uncommented
	//gpiod_line_request_bulk_rising_edge_events(&line_bulk, "foobar");
	gpiod_line_request_bulk_falling_edge_events(&line_bulk, "foobar");
	//gpiod_line_request_bulk_both_edges_events(&line_bulk, "foobar");

	// Display the line event values for rising and falling
	//ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] The constants defined for distinguishing line events are:, GPIOD_LINE_EVENT_RISING_EDGE = " << GPIOD_LINE_EVENT_RISING_EDGE << ", and GPIOD_LINE_EVENT_FALLING_EDGE = " << GPIOD_LINE_EVENT_FALLING_EDGE);

	int forward = 0;
	int total = 0;
	float percent_forward = 0.0;
	
	// Enter a loop that endlessly monitors the encoders
	while (encoder_thread_should_count)
	{

		// Monitor for the requested events on the GPIO line bulk
		// > Note: the function "gpiod_line_event_wait" returns:
		//    0  if wait timed out
		//   -1  if an error occurred
		//    1  if an event occurred.
		returned_wait_flag = gpiod_line_event_wait_bulk(&line_bulk, &timeout_spec, &event_bulk);

		// Respond based on the the return flag
		if (returned_wait_flag == 1)
		{
			// Get the number of events that occurred
			int num_events_during_wait = gpiod_line_bulk_num_lines(&event_bulk);

			// Lock the mutex while before counting the events
			m_counting_mutex.lock();

			// Iterate over the event
			for (int i_event = 0; i_event < num_events_during_wait; i_event++)
			{
				// Get the line handle for this event
				struct gpiod_line *line_handle = gpiod_line_bulk_get_line(&event_bulk, i_event);

				// Get the number of this line
				unsigned int this_line_number = gpiod_line_offset(line_handle);

				// Read the event on the GPIO line
				// > Note: the function "gpiod_line_event_read" returns:
				//    0  if the event was read correctly
				//   -1  if an error occurred
				returned_read_flag = gpiod_line_event_read(line_handle,&event);

				// Respond based on the the return flag
				if (returned_read_flag == 0)
				{
					
					// Increment the respective count
					if (this_line_number == line_number_left_a) {
						// > For left motor channel B
						//value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_b, false, "foobar");
						value = gpiod_line_get_value(line_left_b);
						if (value == 1) {
							//ROS_INFO_STREAM("line " << line_number_left_b << " returned value = " << value);
							//ROS_INFO_STREAM("Left Forward");
							//ROS_INFO_STREAM("1");
							m_encoder_counts_for_motor_left_a++;
							total++;
							forward++;
						} else if (value == 0) {
							//ROS_INFO_STREAM("Left Backward");
							//ROS_INFO_STREAM("0");
							m_encoder_counts_for_motor_left_a--;
							total++;
						}
						//ROS_INFO_STREAM("line " << line_number_left_b << " returned value = " << value);
					} else if (this_line_number == line_number_left_b) {
						//m_encoder_counts_for_motor_left_b++;
					} else if (this_line_number == line_number_right_a) {
						// > For right motor channel B
						//value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_b, false, "foobar");
						value = gpiod_line_get_value(line_right_b);
						if (value == 1) {
							//ROS_INFO_STREAM("Right Backward");
							//ROS_INFO_STREAM("0");
							//ROS_INFO_STREAM("line " << line_number_right_b << " returned value = " << value);
							m_encoder_counts_for_motor_right_a--;
							total++;
						} else if (value == 0) {
							//ROS_INFO_STREAM("Right Forward");
							//ROS_INFO_STREAM("1");
							m_encoder_counts_for_motor_right_a++;
							total++;
							forward++;
						}
						//ROS_INFO_STREAM("line " << line_number_right_b << " returned value = " << value);
					} else if (this_line_number == line_number_right_b) {
						//m_encoder_counts_for_motor_right_b++;
					}

				} // END OF: "if (returned_read_flag == 0)"

			} // END OF: "for (int i_event = 0; i_event < num_events_during_wait; i_event++)"

			// Unlock the mutex
			m_counting_mutex.unlock();

		} // END OF: "if (returned_wait_flag == 1)"
	} // END OF: "while (true)"

	// Release the lines
	gpiod_line_release_bulk(&line_bulk);  
	// Close the GPIO chip
	gpiod_chip_close(chip);
	// Inform the user
	percent_forward = float(forward)/float(total);
	ROS_INFO_STREAM("Percentage forward counts: " << percent_forward);
	ROS_INFO("[ENCODER READ MULTI THREADED] Lines released and GPIO chip closed");
}


int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "encoder_read_multi_threaded");
	ros::NodeHandle nodeHandle("~");

	// Get the GPIO line number to monitor
	// Notes:
	// > If you look at the "encoder.launch" file located in
	//   the "launch" folder, you see the following lines of code:
	//       <param
	//           name   = "line_number_for_motor_left_channel_a"
	//           value  = 105
	//       />
	// > These lines of code add a parameter named to this node
	//   with the parameter name: "line_number_for_motor_left_channel_a"
	// > Thus, to access this parameter, we first get a handle to
	//   this node within the namespace that it was launched.
	//

	// Get the "gpiochip" number parameter:
	if ( !nodeHandle.getParam("gpiochip_number", m_gpiochip_number) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"gpiochip_number\" parameter. Using default value instead.");
	}

	// Get the line number parameters:
	// > For channel A of the left side motor
	if ( !nodeHandle.getParam("line_number_for_motor_left_channel_a", m_line_number_for_motor_left_channel_a) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_left_channel_a\" parameter. Using default value instead.");
	}
	// > For channel B of the left side motor
	if ( !nodeHandle.getParam("line_number_for_motor_left_channel_b", m_line_number_for_motor_left_channel_b) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_left_channel_b\" parameter. Using default value instead.");
	}
	// > For channel A of the right side motor
	if ( !nodeHandle.getParam("line_number_for_motor_right_channel_a", m_line_number_for_motor_right_channel_a) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_right_channel_a\" parameter. Using default value instead.");
	}
	// > For channel A of the right side motor
	if ( !nodeHandle.getParam("line_number_for_motor_right_channel_b", m_line_number_for_motor_right_channel_b) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_motor_right_channel_b\" parameter. Using default value instead.");
	}
	// > Display the line numbers being monitored
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Will monitor line_numbers = " << m_line_number_for_motor_left_channel_a << ", " << m_line_number_for_motor_left_channel_b << ", " << m_line_number_for_motor_right_channel_a << ", and " << m_line_number_for_motor_right_channel_b);

	// Get the "detla t" parameter for the publishing frequency
	if ( !nodeHandle.getParam("delta_t_for_publishing_counts", m_delta_t_for_publishing_counts) )
	{
		// Display an error message
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"delta_t_for_publishing_counts\" parameter. Using default value instead.");
	}

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Initialise a subscriber for the duty cycle of the main drive motors
	ros::Subscriber target_robot_velocity_subscriber = nh_for_group.subscribe("cmd_vel", 1, targetRobotVelocitySubscriberCallback);

	// Initialise a publisher for the encoder counts
	// > Note, the second is the size of our publishing queue. We choose to
	//   buffer encoder counts messages because we want to avoid losing counts.
	m_encoder_counts_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightInt32>("encoder_counts", 100, false);

	// Initialise a publisher for the pose
	// > Note, the second is the size of our publishing queue. We choose to
	//   buffer encoder counts messages because we want to avoid losing counts.
	m_pose_publisher = nh_for_group.advertise<geometry_msgs::Twist>("odom_pose", 100, false);

	// Initialise a publisher for the measured wheel velocity
	// > Note, the second is the size of our publishing queue. We choose to
	//   buffer encoder counts messages because we want to avoid losing counts.
	m_measured_wheel_velocity_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("measured_wheel_velocity", 100, false);

	// Initialise a publisher for the target wheel velocity
	// > Note, the second is the size of our publishing queue. We choose to
	//   buffer encoder counts messages because we want to avoid losing counts.
	m_target_wheel_velocity_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("target_wheel_velocity", 100, false);

	// Initialise a timer
	m_timer_for_publishing = nodeHandle.createTimer(ros::Duration(m_delta_t_for_publishing_counts), timerCallbackForPublishing, false);

	// Get the parameter for how long to drive the motors
	// NOTE: this parameter is purely for the convenience of testing.
	if ( !nodeHandle.getParam("time_in_seconds_to_drive_motors", m_time_in_seconds_to_drive_motors) )
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"time_in_seconds_to_drive_motors\" parameter. Using default value instead.");

	// Initialise a publisher for commanding the motors
	// NOTE: this publisher and what it is used for is
	// purely for the convenience of testing.
	m_motor_pwm_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("set_motor_duty_cycle", 100, false);

	// Get the parameter for target speed when driving the motors
	// NOTE: this parameter is purely for the convenience of testing.
	if ( !nodeHandle.getParam("drive_motor_target_speed", m_drive_motor_target_speed) )
		ROS_INFO("[ENCODER READ MULTI THREADED] FAILED to get \"drive_motor_target_speed\" parameter. Using default value instead.");

	// Create thread for counting the encoder events
	std::thread encoder_counting_thread (encoderCountingThreadMain);
	//boost::thread encoder_counting_thread(encoderCountingThreadMain);

	// Spin the node
	ros::spin();

	// Set the flag to stop counting
	encoder_thread_should_count = false;

	// Join back the encoder counting thread
	encoder_counting_thread.join();

	return 0;
}
