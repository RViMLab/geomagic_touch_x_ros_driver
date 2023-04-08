// Std lib
#include <string.h>

// Eigen
#include <Eigen/Dense>

// ROS
#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Wrench.h>

// Haptic device API
#include <HD/hd.h>
#include <HDU/hduError.h>

// Haptic device state
struct HapticDeviceState {

  // current position of the device facing the device base. Right is
  // +x, up is +y, toward user is +z.
  HDdouble position[3];

  // current velocity of the device.
  HDdouble velocity[3];

  // column-major transform of the device end-effector.
  HDdouble transform[16];

  // angular velocity of the device gimbal.
  HDdouble angular_velocity[3];

  // joint angles of the device. These are joint angles used for
  // computing the kinematics of the armature relative to the base
  // frame of the device. For Touch devices: Turet Left +, Thigh Up +,
  // Shin Up +
  HDdouble joint_angles[3];

  // angles of the device gimbal. For Touch devices: From Neutral
  // position Right is +, Up is -, CW is +
  HDdouble gimbal_angles[3];

  // current force, i.e. the force that the user is commanding to the
  // device during the frame in which this is called.
  HDdouble force[3];

  // current torque, i.e. the torque that the user is commanding to
  // the device during the frame in which this is called.
  HDdouble torque[3];

  // current joint torque, i.e. the torque the user is commanding to
  // the first 3 joints of the device during the frame in which this
  // is called
  HDdouble joint_torque[3];

  // current gimbal torque, i.e. the gimbal torque the user is
  // commanding to the device during the frame in which this is
  // called.
  HDdouble gimbal_torque[3];

  // Force to command at end-effector
  HDdouble cmd_force[3];

};

HDCallbackCode HDCALLBACK device_state_callback(void *data) {

  // Setup
  HapticDeviceState *state = (HapticDeviceState *)data;

  // Begin frame
  hdBeginFrame(hdGetCurrentDevice());

  // Get device state
  hdGetDoublev(HD_CURRENT_POSITION, state->position);
  hdGetDoublev(HD_CURRENT_VELOCITY, state->velocity);
  hdGetDoublev(HD_CURRENT_TRANSFORM, state->transform);
  hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, state->angular_velocity);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, state->joint_angles);
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, state->gimbal_angles);
  hdGetDoublev(HD_CURRENT_FORCE, state->force);
  hdGetDoublev(HD_CURRENT_TORQUE, state->torque);
  hdGetDoublev(HD_CURRENT_JOINT_TORQUE, state->joint_torque);
  hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, state->gimbal_torque);

  // Set device force
  hdSetDoublev(HD_CURRENT_FORCE, state->cmd_force);

  // End frame
  hdEndFrame(hdGetCurrentDevice());

  // Check for error
  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback\n");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  return HD_CALLBACK_CONTINUE;

}

class CmdForceListener {

public:

  HapticDeviceState *state;
  ros::Subscriber _cmd_force_sub;

  CmdForceListener(HapticDeviceState *statein, ros::NodeHandle *nh) {
    state = statein;
    _cmd_force_sub = nh->subscribe("cmd_force", 1000, &CmdForceListener::cmd_force_callback, this);
  }

  void cmd_force_callback(const geometry_msgs::Wrench::ConstPtr& msg) {
    state->cmd_force[0] = msg->force.x;
    state->cmd_force[1] = msg->force.y;
    state->cmd_force[2] = msg->force.z;
  }

};

int main(int argc, char **argv) {

  // Initialize ROS node
  ros::init(argc, argv, "geomagic_touch_x_node");
  ros::NodeHandle nh("~");

  // Setup constants
  const double mm_to_m=0.001;
  const int hz = 200;
  const double dt = 1.0/static_cast<double>(hz);

  // Setup tf broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "touch_x_base";
  tf.child_frame_id = "touch_x_ee";

  // Setup twist publisher
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist", 1000);
  geometry_msgs::TwistStamped twist;

  // Setup joint state publisher
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
  bool q_prev_set = false;
  Eigen::VectorXd q(6), q_prev(6), dqdt(6);
  sensor_msgs::JointState joint_state;
  joint_state.name.push_back("joint_angle_1");
  joint_state.name.push_back("joint_angle_2");
  joint_state.name.push_back("joint_angle_3");
  joint_state.name.push_back("gimbal_angle_1");
  joint_state.name.push_back("gimbal_angle_2");
  joint_state.name.push_back("gimbal_angle_3");
  for (int i = 0; i < 6; ++i) {
    joint_state.position.push_back(0.);
    joint_state.velocity.push_back(0.);
    joint_state.effort.push_back(0.);
  }

  // Get device name
  std::string device_name;
  nh.getParam("device_name", device_name);
  if (!(device_name.length() > 0)) {
    ROS_ERROR("No device name given, shutting down geomagic_touch_x_node!");
    return -1;
  }
  ROS_INFO("Haptic device name is \"%s\".", device_name.c_str());

  // Initialize haptic device
  HHD hHD = hdInitDevice(device_name.c_str());

  // Enable force output, i.e. all motors are turned on.
  if (!hdIsEnabled(HD_FORCE_OUTPUT)) {
    hdEnable(HD_FORCE_OUTPUT);
    ROS_INFO("Force output is enabled for the device \"%s\".", device_name.c_str());
  } else {
    ROS_ERROR("Did not enable force output.");
    return -1;
  }

  // Setup device state
  HapticDeviceState state;
  for (int i = 0; i < 3; ++i)
    state.cmd_force[i] = 0.;

  // Start state publisher
  hdScheduleAsynchronous(device_state_callback, &state, HD_DEFAULT_SCHEDULER_PRIORITY);

  // Start the scheduler
  hdStartScheduler();

  // Start cmd_force listener
  CmdForceListener cmd_force_listener = CmdForceListener(&state, &nh);

  // Setup for main state publisher loop
  ros::Rate rate(hz); // runs at 200 Hz

  // Publish state
  while (ros::ok()) {

    // Get time
    ros::Time now = ros::Time::now();

    // Pack transform stamp and translation
    tf.header.stamp = now;
    tf.transform.translation.x = mm_to_m * state.transform[12];
    tf.transform.translation.y = mm_to_m * state.transform[13];
    tf.transform.translation.z = mm_to_m * state.transform[14];

    // Convert state transform to rotation matrix, then quaternion, and pack transform
    Eigen::Matrix3d R(3, 3);
    R(0, 0) = state.transform[0];
    R(0, 1) = state.transform[1];
    R(0, 2) = state.transform[2];
    R(1, 0) = state.transform[4];
    R(1, 1) = state.transform[5];
    R(1, 2) = state.transform[6];
    R(2, 0) = state.transform[8];
    R(2, 1) = state.transform[9];
    R(2, 2) = state.transform[10];

    Eigen::Quaterniond quat(R.transpose()); // require transpose otherwise rotation is inverted
    tf.transform.rotation.x = quat.x();
    tf.transform.rotation.y = quat.y();
    tf.transform.rotation.z = quat.z();
    tf.transform.rotation.w = quat.w();

    // Send transform
    br.sendTransform(tf);

    // Pack joint states
    joint_state.header.stamp = now;

    for (int i = 0; i < 3; ++i) {
      q(i) = state.joint_angles[i];
      q(i+3) = state.gimbal_angles[i];
      joint_state.effort[i] = state.joint_torque[i];
      joint_state.effort[i+3] = state.gimbal_torque[i];
    }

    if (q_prev_set) {
      dqdt = (q - q_prev)/dt;
    } else {
      dqdt = Eigen::VectorXd::Zero(6);
    }

    q_prev = q;
    q_prev_set = true;

    for (int i = 0; i < 6; ++i) {
      joint_state.position[i] = q(i);
      joint_state.velocity[i] = dqdt(i);
    }

    // Publish joint states
    joint_state_pub.publish(joint_state);

    // Pack twist
    twist.header.stamp = now;
    twist.twist.linear.x = mm_to_m * state.velocity[0];
    twist.twist.linear.y = mm_to_m * state.velocity[1];
    twist.twist.linear.z = mm_to_m * state.velocity[2];

    twist.twist.angular.x = state.angular_velocity[0];
    twist.twist.angular.y = state.angular_velocity[1];
    twist.twist.angular.z = state.angular_velocity[2];

    // Publish twist
    twist_pub.publish(twist);

    // Spin and sleep
    ros::spinOnce();
    rate.sleep();

  }

  // Cleanup
  hdStopScheduler();
  hdDisable(HD_FORCE_OUTPUT);
  hdDisableDevice(hHD);

  return 0;

}
