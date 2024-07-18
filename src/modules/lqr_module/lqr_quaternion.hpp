#pragma once
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"


#include <cmath>
// #include <ros/ros.h>
// #include <ct/optcon/optcon.h>
#include <lqr_module/declarations_quaternion.hpp>
// #include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
// #include <mavros/frame_tf.h>
// #include <mav_msgs/eigen_mav_msgs.h>
// #include <mav_trajectory_generation/polynomial_optimization_linear.h>
//#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
// #include <mav_trajectory_generation/trajectory.h>
// #include <mav_trajectory_generation/trajectory_sampling.h>
// #include <mav_trajectory_generation_ros/ros_visualization.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <ros/package.h>
#include <drivers/drv_hrt.h> // Include for hrt_absolute_time
#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <vector>
#include "LQR.hpp"

namespace LQR_Q{

struct State {
    double timestamp;
    Eigen::Vector3d position_W;
    Eigen::Quaterniond orientation_W_B;
    Eigen::Vector3d velocity_W;
    Eigen::Vector3d angular_velocity_B;
    Eigen::Vector3d acceleration_W;
};

class LQR_Quaternion {
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    LQR_Quaternion();

    /*!
     * Destructor.
     */
    virtual ~LQR_Quaternion();

    control_vector_t getTrajectoryControl();
    state_vector_t getError();
    // ct::core::FeedbackMatrix<nStates, nControls> getGain();
    using feedback_matrix_t = ::feedback_matrix_t;
    feedback_matrix_t getGain() const;
    void setOutput(double output, int j);
    void setOutput(control_vector_t output);
    control_vector_t getOutput();
    state_vector_t getRefStates();
    void topicCallback(const vehicle_local_position_s& local_pos, const vehicle_attitude_s& attitude, const vehicle_local_position_setpoint_s& local_pos_sp, const vehicle_attitude_setpoint_s& attitude_sp, const vehicle_rates_setpoint_s& rates_sp);
    void readTrajectoryFromFile(const std::string &file_path);
    bool initiated;
    std::vector<State>& getStates();
    std::vector<State> states_;


   private:

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */

    void setStates(const vehicle_local_position_s& local_pos, const vehicle_attitude_s& attitude, state_vector_t& x);
    void setError(const state_vector_t& xref, const state_vector_t& x,
                  state_vector_t& xerror,
                  const vehicle_local_position_setpoint_s& local_pos_sp,
                  const vehicle_attitude_setpoint_s& attitude_sp);
    bool setTrajectoryReference(state_vector_t& xref, control_vector_t& uref);
    bool setReference(state_vector_t& xref, control_vector_t& uref,
                      const vehicle_local_position_setpoint_s& local_pos_sp,
                      const vehicle_attitude_setpoint_s& attitude_sp,
                      const vehicle_rates_setpoint_s& rates_sp);
    Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond &q);

    // void generateTrajectory(mav_msgs::EigenTrajectoryPoint::Vector& states);

    // //! ROS node handle.
    // ros::NodeHandle& nodeHandle_;

    // //! ROS topic subscriber.
    // ros::Subscriber odom_sub_;

    // //Marker publisher
    // ros::Publisher marker_pub_;

    //! uORB subscriptions.
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
    uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
    uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
    // uORB::Publication<vehicle_rates_setpoint_lqr_s>

    //! State and control matrix dimensions
    const size_t state_dim = nStates;
    const size_t control_dim = nControls;

    //Trajectory
    double sampling_interval = 0.1;
    const double v_max = 2.0;
    const double a_max = 5.0;
    const int dimension = 3;



    // mav_msgs::EigenTrajectoryPoint::Vector states_;
    // visualization_msgs::MarkerArray markers_;

    hrt_abstime init_time_;
    Eigen::Vector3d position_enu_;
    Eigen::Vector3d velocity_enu_;
    Eigen::Quaterniond q_enu_;
    state_matrix_t A_;
    control_gain_matrix_t B_;
    // ct::core::FeedbackMatrix<nStates, nControls> Kold_;
    feedback_matrix_t Kold_;
    // ct::core::FeedbackMatrix<nStates, nControls> Knew_;
    feedback_matrix_t Knew_;
    hrt_abstime callBack_;
    state_vector_t x_;
    control_vector_t u_;
    state_vector_t xref_;
    control_vector_t uref_;
    state_vector_t xerror_;
    control_vector_t output_;
    int traj_index;

    // ct::optcon::TermQuadratic<nStates, nControls> quadraticCost_;
    // ct::optcon::TermQuadratic<nStates, nControls>::state_matrix_t Q_;
    // ct::optcon::TermQuadratic<nStates, nControls>::control_matrix_t R_;
    //ct::optcon::LQR<nStates, nControls> lqrSolver_;

    LQR<nStates, nControls> lqrSolver_;
    LQR<nStates, nControls>::state_matrix_t Q_;
    LQR<nStates, nControls>::control_matrix_t R_;


    //states
    state_matrix_t A_quadrotor(const state_vector_t& x, const control_vector_t& u);
    control_gain_matrix_t B_quadrotor(const state_vector_t& x, const control_vector_t& u);
  };

} /* namespace */

#pragma GCC diagnostic pop
