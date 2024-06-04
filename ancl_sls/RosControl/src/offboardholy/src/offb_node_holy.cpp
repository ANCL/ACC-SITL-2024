#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

#include <offboardholy/PTStates.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
// #include <gazebo_msgs/LinkStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

#include <StabController.h>
#include <TracController.h>
#include <rtwtypes.h>
#include <cstddef>
#include <cstdlib>
# include <iostream>

void PT_state_pub(ros::Publisher &sls_state_pub);
void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);
void IntrgralStabController(const double x[10], const double Kv[15],
                           const double param[4], const double setpoint[3],
                           double u[3], double err_int[3], double Ki[3], double t_last);

geometry_msgs::Pose quadpose;
geometry_msgs::Pose loadpose;
geometry_msgs::Pose pendpose;
geometry_msgs::Twist pendtwist;
geometry_msgs::Twist quadtwist;
geometry_msgs::Twist loadtwist;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_local_pos;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_local_pos = *msg;
}

geometry_msgs::TwistStamped current_local_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_local_vel = *msg;
}


mavros_msgs::AttitudeTarget attitude;
void attitude_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    attitude = *msg;
}

offboardholy::PTStates PTState;
void sls_state_cb(const offboardholy::PTStates::ConstPtr& msg){
    PTState = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",10,vel_cb);
    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/offboardholy/target_attitude", 10, attitude_target_cb);
    ros::Subscriber sls_state_sub = nh.subscribe<offboardholy::PTStates>("/offboardholy/sls_state", 10, sls_state_cb);

    ros::Publisher attitude_setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    double err_int[3] = {};
    double t_last = ros::Time::now().toSec();
    double Ki[3] = {0.0000001, 0.0000001, 0.000000001};

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.1; //1.1;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    // mavros_msgs::AttitudeTarget attitude;
    attitude.header.stamp = ros::Time::now();
    attitude.header.frame_id = "map"; 
    attitude.orientation.x = 0;
    attitude.orientation.y = 0;
    attitude.orientation.z = 0;
    attitude.orientation.w = 0;
    attitude.thrust = 0.2;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        // attitude.header.stamp = ros::Time::now();
        // attitude_setpoint_pub.publish(attitude);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

	mavros_msgs::SetMode land_mode;
	land_mode.request.custom_mode = "AUTO.LAND";


    ros::Time last_request = ros::Time::now();

	double distance = 0;

    int stage = 0;

    while(ros::ok()){
        double dv[10] = {};
        double controller_output[3] = {};
        double Kv12[12] = {2.2361,  3.1623,  3.1623,    3.0777,    8.4827,    8.4827,   0,    9.7962,    9.7962,      0,      5.4399,       5.4399};
        double Kv15[15] = {2.2667,  3.2469,  3.2469,    3.0876,    8.5803,    8.5803,   -0.0224 ,   9.8503,      9.8503,    0 , 5.4498,  5.4498,     0,   -0.0316, -0.0316};

        // double Param[4] = {1.4, 0.08, 0.75, 9.8};
        double Param[4] = {1.5, 0.2, 1, 9.8};
        double Setpoint[3] = {0, 0, -0.3};
        //double Setpoint[3] = {0, 0, -10.0};
        for (int i=0;i<10; i++){
          dv[i] = PTState.PT_states[i];
          // ROS_INFO_STREAM( "dv[i]: "<< i << " : " << dv[i] << "\n");
        }
        switch (stage)
        {  
        case 0: // takeoff
            if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);
            distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2) 
            + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
            + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);

            // distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2) 
            // + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
            // + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);

            if(ros::Time::now() - last_request > ros::Duration(10.0) && distance < 0.2){
                stage += 1;
                last_request = ros::Time::now();
                ROS_INFO("Takeoff finished and switch to position setpoint control mode");
            }
            break;

        case 1: // setpoint position control
            attitude.header.stamp = ros::Time::now();
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            // IntrgralStabController(dv, Kv15, Param, Setpoint, controller_output, err_int, Ki, t_last);
            force_attitude_convert(controller_output, attitude);
            attitude_setpoint_pub.publish(attitude);
            
            distance = std::pow((current_local_pos.pose.position.x - Setpoint[0]),2) 
            + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])),2)
            + std::pow((current_local_pos.pose.position.z - (-Setpoint[2]+0.95)),2);

            // ROS_INFO_STREAM("Distance: " << distance);
            if(ros::Time::now() - last_request > ros::Duration(15.0) && distance < 0.2){
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 1");
                last_request = ros::Time::now();
            }
            break;

        case 2: // setpoint position control
            Setpoint[0] = 1;
            Setpoint[1] = 0.5;
            Setpoint[2] = -0.6;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);
            
            distance = std::pow((current_local_pos.pose.position.x - Setpoint[0]),2) 
            + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])),2)
            + std::pow((current_local_pos.pose.position.z - (-Setpoint[2]+0.95)),2);
            // ROS_INFO_STREAM(" X: " << current_local_pos.pose.position.x << " Y: " << current_local_pos.pose.position.y << " Z: " << current_local_pos.pose.position.z);
            ROS_INFO_STREAM("Distance: " << distance);
            if(ros::Time::now() - last_request > ros::Duration(15.0) && distance < 0.2){
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 2");
                last_request = ros::Time::now();
            }
            break;
        
        case 3: // setpoint position control
            Setpoint[0] = -1;
            Setpoint[1] = 0;
            Setpoint[2] = -0.3;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);
            
            distance = std::pow((current_local_pos.pose.position.x - Setpoint[0]),2) 
            + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])),2)
            + std::pow((current_local_pos.pose.position.z - (-Setpoint[2]+0.95)),2);
            ROS_INFO_STREAM("Distance: " << distance);
            if(ros::Time::now() - last_request > ros::Duration(15.0) && distance < 0.2){
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 3");
                last_request = ros::Time::now();
            }
            break;

        case 4: // setpoint position control
            Setpoint[0] = 0;
            Setpoint[1] = 0;
            Setpoint[2] = -0.3;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);
            
            distance = std::pow((current_local_pos.pose.position.x - Setpoint[0]),2) 
            + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])),2)
            + std::pow((current_local_pos.pose.position.z - (-Setpoint[2]+0.95)),2);
            ROS_INFO_STREAM("Distance: " << distance);
            if(ros::Time::now() - last_request > ros::Duration(15.0) && distance < 0.2){
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Trajectory Tracking");
                last_request = ros::Time::now();
            }
            break;
        
        case 5: //Trajectory tracking
            attitude.header.stamp = ros::Time::now();
            TracController(dv, Kv12, Param, ros::Time::now().toSec() - last_request.toSec(), controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude_setpoint_pub.publish(attitude);
            if(ros::Time::now() - last_request > ros::Duration(32.0)){
                stage += 1;
                ROS_INFO("Finish Trajactory Tracking and land");
                last_request = ros::Time::now();
            }
            break;


        case 6: // land
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0.5;
            local_pos_pub.publish(pose);
            distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2) 
            + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
            + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);
            if(ros::Time::now() - last_request > ros::Duration(5.0)){
            // if(ros::Time::now() - last_request > ros::Duration(5.0) && distance < 0.2){

                // pose.header.stamp = ros::Time::now();
                // pose.header.frame_id = "map";
                // pose.pose.position.x = -0.5;
                // pose.pose.position.y = 0;
                // pose.pose.position.z = 0.5;
                // local_pos_pub.publish(pose);
                // distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2) 
                // + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
                // + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);
                // if(ros::Time::now() - last_request > ros::Duration(10.0) && distance < 0.1){
                if( set_mode_client.call(land_mode) && land_mode.response.mode_sent){
                    stage += 1;
                    ROS_INFO("Land finished");
                    // }
                }
            }
            break;
        
        default:
            if( set_mode_client.call(land_mode) && land_mode.response.mode_sent){
                // arm_cmd.request.value = false;
                // arming_client.call(arm_cmd);
                // ROS_INFO("Land enabled"); 
            }
            break;
        }
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }
        // if((ros::Time::now() - last_request < ros::Duration(20.0))&& stage==0){
        //     pose.header.stamp = ros::Time::now();
        //     local_pos_pub.publish(pose);
        //     ROS_INFO("Position Control armed");
        // }else{
        //     stage += 1;
        // }

      
        // attitude.header.stamp = ros::Time::now();
        // attitude_setpoint_pub.publish(attitude);


        
        // ROS_INFO("Attitude Control armed");


		// distance = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2) 
		// + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
		// + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);

		// if(distance < 0.1 && (ros::Time::now() - last_request > ros::Duration(50.0)) ){
		// 	if( set_mode_client.call(land_mode) &&
        //         land_mode.response.mode_sent){
        //         ROS_INFO("Land enabled");
		// 		break;
        //     }
		// }
		
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude){
  attitude.header.stamp = ros::Time::now();
  double roll,pitch,yaw, thrust;
  thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
  yaw = 0;
  roll = std::asin(controller_output[1]/thrust);
  pitch = std::atan2(controller_output[0], -controller_output[2]);
  ROS_DEBUG("F1: %f, F2: %f, F3:, %f", controller_output[0], controller_output[1], controller_output[2]);
  tf2::Quaternion attitude_target_q;
  attitude_target_q.setRPY(roll, pitch, yaw);
  attitude.orientation.x = attitude_target_q.getX();
  attitude.orientation.y = attitude_target_q.getY();
  attitude.orientation.z = attitude_target_q.getZ();
  attitude.orientation.w = attitude_target_q.getW();

  attitude.thrust = (thrust-16.67122)/20 + 0.8168;
  //   attitude.thrust = (thrust-14.504)/8 + 0.78;

//   ROS_INFO_STREAM("Force: " << controller_output[0]<< "   " << controller_output[1]<< "   " << controller_output[2] << " orientation " << roll << "  " << pitch);
}

// void IntrgralStabController(const double x[10], const double Kv[12],
//                            const double param[4], const double setpoint[3],
//                            double u[3], double err_int[3], double Ki[3]){
    
//     err_int[0] += (current_local_pos.pose.position.x - setpoint[0]);
//     err_int[1] += current_local_pos.pose.position.y - (-setpoint[1]);
//     err_int[2] += current_local_pos.pose.position.z - (-setpoint[2]+1.00);

//     StabController(x, Kv, param, setpoint, u);

//     // set a bound for the error integration
//     for(int i=0; i<3; i++){
//         if(err_int[i]>20) err_int[i] = 20;
//         else if (err_int[i]<-20) err_int[i] = -20;
//         ROS_DEBUG("err1: %f, err2: %f, err3: %f", err_int[0], err_int[1], err_int[2]);
//     }

//     Ki[0] = -0.01;
//     Ki[1] = 0.01;
//     Ki[2] = 0.001;

//     u[0] += Ki[0] * err_int[0];
//     u[1] += Ki[1] * err_int[1];
//     u[2] += Ki[2] * err_int[2];
// }

void IntrgralStabController(const double x[10], const double Kv[15],
                           const double param[4], const double setpoint[3],
                           double u[3], double err_int[3], double Ki[3], double t_last){
    
    double dt;
    // dt = ros::Time::now().toSec() - t_last;

    ROS_INFO("dt = %f, t_last = %f", dt, t_last);

    err_int[0] += (setpoint[0] - x[0])*dt;
    err_int[1] += (setpoint[1] - x[1])*dt;
    err_int[2] += ((setpoint[2] - 1) - x[2])*dt;

    // err_int[0] = setpoint[0] - x[0];
    // err_int[1] = setpoint[1] - x[1];
    // err_int[2] = (setpoint[2] - 1) - x[2];

    //set a bound for the error integration
    for(int i=0; i<3; i++){
        if(err_int[i]>20) err_int[i] = 20;
        else if (err_int[i]<-20) err_int[i] = -20;
    }       
    
    ROS_DEBUG("err1: %f, err2: %f, err3: %f", err_int[0], err_int[1], err_int[2]);


    double x_ext[13]={};

    for(int i=0; i<10; i++){
        x_ext[i] = x[i];
    }

    for(int i=10; i<13; i++){
        x_ext[i] = err_int[i-10];
    }

    for(int i=0; i<13; i++){
        // ROS_INFO("x_ext[%d]: %f", i, x_ext[i]);
    }
   
    StabController(x_ext, Kv, param, setpoint, u);

    t_last = ros::Time::now().toSec();

}