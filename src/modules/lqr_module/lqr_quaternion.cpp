
#include "lqr_quaternion.hpp"

namespace LQR_Q {

LQR_Quaternion::LQR_Quaternion()
{
    // Manually assign values to Q_ matrix
    // Q_.setZero();
    // Q_(0,0) = 100.0;
    // Q_(1,1) = 100.0;
    // Q_(2,2) = 80.0;
    // Q_(3,3) = 0.5;
    // Q_(4,4) = 0.5;
    // Q_(5,5) = 0.5;
    // Q_(6,6) = 0.5;
    // Q_(7,7) = 5.0;
    // Q_(8,8) = 5.0;
    // Q_(9,9) = 3.0;

    // // Manually assign values to R_ matrix
    // R_.setZero();
    // R_(0,0) = 50.0;  // Increase to penalize roll control input more
    // R_(1,1) = 50.0;  // Increase to penalize pitch control input more
    // R_(2,2) = 10.0;  // Increase to penalize yaw control input more
    // R_(3,3) = 5.0;   // Increase to penalize thrust control input more

    Q_.setZero();
    Q_(0,0) = 100.0;
    Q_(1,1) = 100.0;
    Q_(2,2) = 80.0;
    Q_(3,3) = 0.5;
    Q_(4,4) = 0.5;
    Q_(5,5) = 0.5;
    Q_(6,6) = 0.5;
    Q_(7,7) = 5.0;
    Q_(8,8) = 5.0;
    Q_(9,9) = 3.0;

    // Manually assign values to R_ matrix
    R_.setZero();
    R_(0,0) = 15.0;
    R_(1,1) = 15.0;
    R_(2,2) = 1.0;
    R_(3,3) = 0.4;

    // Initialize other member variables
    initiated = false;
    callBack_ = hrt_absolute_time();
    init_time_ = hrt_absolute_time();
    traj_index = 1;
}

LQR_Quaternion::~LQR_Quaternion()
{
}

void LQR_Quaternion::setError(const state_vector_t& xref, const state_vector_t& x, state_vector_t& xerror)
{
  /*Position error*/
  xerror(0) = (x(0) - xref(0));
  xerror(1) = (x(1) - xref(1));
  xerror(2) = (x(2) - xref(2));

  /*Orientation error*/
  Eigen::Quaterniond q(x(3), x(4), x(5), x(6));
  Eigen::Quaterniond qref(xref(3), xref(4), xref(5), xref(6));
  Eigen::Quaterniond qerror = q.inverse() * qref;
  xerror(3) = 0;
  xerror(4) = qerror.x();
  xerror(5) = qerror.y();
  xerror(6) = qerror.z();

  /*Velocity error*/
  xerror(7) = (x(7) - xref(7));
  xerror(8) = (x(8) - xref(8));
  xerror(9) = (x(9) - xref(9));
}

void LQR_Quaternion::topicCallback(const vehicle_local_position_s& local_pos, const vehicle_attitude_s& attitude)
{

    try
    {
        setStates(local_pos, attitude, x_);
        //PX4_INFO("setStates executed successfully");
    } catch (const std::exception &e) {
        PX4_ERR("Exception in setStates: %s", e.what());
        return;
    }

    try {
        setTrajectoryReference(xref_, uref_);
        //PX4_INFO("setTrajectoryReference executed successfully");
    } catch (const std::exception &e) {
        PX4_ERR("Exception in setTrajectoryReference: %s", e.what());
        return;
    }

    try {
        setError(xref_, x_, xerror_);
        //PX4_INFO("setError executed successfully");
    } catch (const std::exception &e) {
        PX4_ERR("Exception in setError: %s", e.what());
        return;
    }

    // Check for NaN values in state vectors
    if (x_.hasNaN() || xref_.hasNaN() || xerror_.hasNaN()) {
        PX4_ERR("NaN detected in state vectors");
        return;
    }

    if ((hrt_absolute_time() - callBack_) > 100000)
    { // 0.1 seconds in microseconds
            A_ = A_quadrotor(xref_, uref_);
            B_ = B_quadrotor(xref_, uref_);

        // Check for NaN values in A and B matrices
        if (A_.hasNaN() || B_.hasNaN()) {
            PX4_ERR("NaN detected in A or B matrices");
            return;
        }

        if (lqrSolver_.compute(Q_, R_, A_, B_, Knew_)) {
            if (!Knew_.hasNaN()) {
                Kold_ = Knew_;
            } else {
                PX4_ERR("NaN detected in Knew_ matrix");
            }
        } else {
            PX4_ERR("LQR solver failed to compute new gain matrix");
        }

        callBack_ = hrt_absolute_time();
    }
}

void LQR_Quaternion::setStates(const vehicle_local_position_s& local_pos, const vehicle_attitude_s& attitude, state_vector_t& x) {
    // Convert position from NED to ENU
    x(0) = local_pos.y;  // y in NED becomes x in ENU
    x(1) = local_pos.x;  // x in NED becomes y in ENU
    x(2) = -local_pos.z; // z in NED becomes -z in ENU

    // Convert quaternion from NED to ENU
    x(3) = attitude.q[0];  // w remains the same
    x(4) = attitude.q[2];  // z in NED becomes x in ENU
    x(5) = attitude.q[1];  // y in NED becomes y in ENU
    x(6) = -attitude.q[3]; // x in NED becomes -z in ENU

    // Convert velocity from NED to ENU
    x(7) = local_pos.vy;  // vy in NED becomes vx in ENU
    x(8) = local_pos.vx;  // vx in NED becomes vy in ENU
    x(9) = -local_pos.vz; // vz in NED becomes -vz in ENU

    // Update position_enu_ with the converted position
    this->position_enu_ = Eigen::Vector3d(x(0), x(1), x(2));

    // Assuming angular velocity and acceleration are not part of the state vector
}

state_matrix_t LQR_Quaternion::A_quadrotor(const state_vector_t& x, const control_vector_t& u)
{
    double wx = u(0);
    double wy = u(1);
    double wz = u(2);
    double norm_thrust  = u(3);
    Eigen::Quaternion<double> q(x(3),x(4),x(5),x(6));
    Eigen::Matrix<double,4,4> q_partial_correction;
    Eigen::Matrix<double,4,4> dqdot_dq;
    Eigen::Matrix<double,3,4> dvdot_dq;
    Eigen::Matrix<double,4,1> q_vec;

    q_vec(0) = q.w();
    q_vec(1) = q.x();
    q_vec(2) = q.y();
    q_vec(3) = q.z();

    state_matrix_t A;
    A.setZero();

    //Position
    A(0,7) = 1;
    A(1,8)= 1;
    A(2,9)= 1;
    Eigen::Matrix<double,4,4> Identity;

    //Orientation
    q_partial_correction = pow(q.norm(),-1.0)*(Identity.Identity() - pow(q.norm(),-2.0)*(q_vec * q_vec.transpose()));

    dqdot_dq << 0, -wx, -wy, -wz,
                wx, 0, wz, -wy,
                wy, -wz, 0, wx,
                wz, wy, -wx, 0;
    dqdot_dq = 0.5*dqdot_dq*q_partial_correction;

    A(3,3) = dqdot_dq(0,0);
    A(3,4) = dqdot_dq(0,1);
    A(3,5) = dqdot_dq(0,2);
    A(3,6) = dqdot_dq(0,3);

    A(4,3) = dqdot_dq(1,0);
    A(4,4) = dqdot_dq(1,1);
    A(4,5) = dqdot_dq(1,2);
    A(4,6) = dqdot_dq(1,3);

    A(5,3) = dqdot_dq(2,0);
    A(5,4) = dqdot_dq(2,1);
    A(5,5) = dqdot_dq(2,2);
    A(5,6) = dqdot_dq(2,3);

    A(6,3) = dqdot_dq(3,0);
    A(6,4) = dqdot_dq(3,1);
    A(6,5) = dqdot_dq(3,2);
    A(6,6) = dqdot_dq(3,3);


    //Velocity
    dvdot_dq << q.y(),  q.z(),  q.w(), q.x(),
              -q.x(), -q.w(),  q.z(), q.y(),
               q.w(), -q.x(), -q.y(), q.z();

    dvdot_dq = 2*norm_thrust*dvdot_dq*q_partial_correction;

    A(7,3) = dvdot_dq(0,0);
    A(7,4) = dvdot_dq(0,1);
    A(7,5) = dvdot_dq(0,2);
    A(7,6) = dvdot_dq(0,3);

    A(8,3) = dvdot_dq(1,0);
    A(8,4) = dvdot_dq(1,1);
    A(8,5) = dvdot_dq(1,2);
    A(8,6) = dvdot_dq(1,3);

    A(9,3) = dvdot_dq(2,0);
    A(9,4) = dvdot_dq(2,1);
    A(9,5) = dvdot_dq(2,2);
    A(9,6) = dvdot_dq(2,3);

    return A;
}

control_gain_matrix_t LQR_Quaternion::B_quadrotor(const state_vector_t& x, const control_vector_t& u)
{
  //  double wx = u(0);
  //  double wy = u(1);
  //  double wz = u(2);
  //  double norm_thrust  = u(3);
   Eigen::Quaternion<double> q(x(3),x(4),x(5),x(6));
   Eigen::Matrix<double,3,1> dvdot_dc;
   Eigen::Matrix<double,4,3> dqdot_dw;

   control_gain_matrix_t B;
   B.setZero();

   dvdot_dc << 2*(q.w()*q.y() + q.x()*q.z()),
               2*(q.y()*q.z() - q.w()*q.x()),
               pow(q.w(),2) - pow(q.x(),2) - pow(q.y(),2) + pow(q.z(),2);

   B(7,3) = dvdot_dc(0);
   B(8,3) = dvdot_dc(1);
   B(9,3) = dvdot_dc(2);

   dqdot_dw << -q.x(), -q.y(), -q.z(),
                q.w(), -q.z(),  q.y(),
                q.z(),  q.w(), -q.x(),
               -q.y(),  q.x(),  q.w();

   dqdot_dw = 0.5*dqdot_dw;

   B(3,0) = dqdot_dw(0,0);
   B(3,1) = dqdot_dw(0,1);
   B(3,2) = dqdot_dw(0,2);

   B(4,0) = dqdot_dw(1,0);
   B(4,1) = dqdot_dw(1,1);
   B(4,2) = dqdot_dw(1,2);

   B(5,0) = dqdot_dw(2,0);
   B(5,1) = dqdot_dw(2,1);
   B(5,2) = dqdot_dw(2,2);

   B(6,0) = dqdot_dw(3,0);
   B(6,1) = dqdot_dw(3,1);
   B(6,2) = dqdot_dw(3,2);

   return B;
}

void LQR_Quaternion::readTrajectoryFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    bool is_header = true;
    while (std::getline(file, line)) {
        if (is_header) {
            is_header = false;
            continue; // Skip the header line
        }

        if (line.empty()) {
            PX4_WARN("Empty line detected, skipping");
            continue;
        }

        std::stringstream ss(line);
        std::string cell;
        State state;

        try {
            // Skip the timestamp
            std::getline(ss, cell, ',');
            if (cell.empty()) throw std::invalid_argument("Missing timestamp");
            state.timestamp = std::stod(cell);

            // Parse position
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing position x"); state.position_W.x() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing position y"); state.position_W.y() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing position z"); state.position_W.z() = std::stod(cell);

            // Parse velocity
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing velocity x"); state.velocity_W.x() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing velocity y"); state.velocity_W.y() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing velocity z"); state.velocity_W.z() = std::stod(cell);

            // Parse acceleration
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing acceleration x"); state.acceleration_W.x() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing acceleration y"); state.acceleration_W.y() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing acceleration z"); state.acceleration_W.z() = std::stod(cell);

            // Parse orientation
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing orientation w"); state.orientation_W_B.w() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing orientation x"); state.orientation_W_B.x() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing orientation y"); state.orientation_W_B.y() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing orientation z"); state.orientation_W_B.z() = std::stod(cell);

            // Parse angular velocity
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing angular velocity x"); state.angular_velocity_B.x() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing angular velocity y"); state.angular_velocity_B.y() = std::stod(cell);
            std::getline(ss, cell, ','); if (cell.empty()) throw std::invalid_argument("Missing angular velocity z"); state.angular_velocity_B.z() = std::stod(cell);

            this->states_.push_back(state);
        } catch (const std::invalid_argument &e) {
            PX4_ERR("Invalid argument in std::stod: %s", e.what());
            PX4_ERR("Line: %s", line.c_str());
            return;
        } catch (const std::out_of_range &e) {
            PX4_ERR("Out of range error in std::stod: %s", e.what());
            PX4_ERR("Line: %s", line.c_str());
            return;
        }
    }
    file.close();
}

bool LQR_Quaternion::setTrajectoryReference(state_vector_t& xref, control_vector_t& uref)
{

    initiated = true;
    Eigen::Vector3d position_ref_enu;
    Eigen::Vector3d accel;
    Eigen::Vector3d orient_yaw;
    Eigen::Vector3d thrust;
    Eigen::Vector3d thrust_dir;
    Eigen::Quaterniond q_yaw;
    Eigen::Vector3d rpy_ref_enu;
    Eigen::Quaterniond qref_enu;
    Eigen::Vector3d velocity_enu;
    std::vector<double> gamma;
    std::vector<double> dist_traj;


    try {
        gamma.resize(states_.size() - 1);
        dist_traj.resize(states_.size() - 1);
    } catch (const std::exception &e) {
        PX4_ERR("Exception in resizing vectors: %s", e.what());
        return false;
    }

    std::vector<LQR_Q::State>::size_type i = traj_index - 1;
    std::vector<LQR_Q::State>::size_type end_index = std::min(static_cast<std::vector<LQR_Q::State>::size_type>(traj_index + 50), states_.size() - 1);

    do {
        i++;
        Eigen::Vector3d distance_vec(states_[i].position_W - states_[i-1].position_W);
        gamma[i-1] = (distance_vec.dot(position_enu_ - states_[i-1].position_W) / pow(distance_vec.norm(), 2));
        dist_traj[i-1] = ((states_[i].position_W - position_enu_).norm());
        // std::cout << "i: " << i << ", gamma[" << i-1 << "]: " << gamma[i-1] << ", dist_traj[" << i-1] << ": " << dist_traj[i-1] << std::endl;
    } while (i < end_index);

    if (traj_index > static_cast<int>(states_.size() - 1))
    {
    std::cout<< "ending - traj_index:    " << traj_index <<std::endl;
    traj_index = states_.size() - 1;

    position_ref_enu = states_[traj_index].position_W;

    /*Position*/
    xref(0) = position_ref_enu.x();
    xref(1) = position_ref_enu.y();
    xref(2) = position_ref_enu.z();

    /*Orientation*/
    xref(3) = states_[traj_index].orientation_W_B.w();
    xref(4) = states_[traj_index].orientation_W_B.x();
    xref(5) = states_[traj_index].orientation_W_B.y();
    xref(6) = states_[traj_index].orientation_W_B.z();

    /*Velocity*/
    xref(7) = 0;
    xref(8) = 0;
    xref(9) = 0;

    /*Body rates*/
    uref(0) = 0;
    uref(1) = 0;
    uref(2) = 0;
    uref(3) = 9.81;
    return true;

    } else {


    std::vector<double>::iterator result = std::min_element(dist_traj.begin() + traj_index, dist_traj.begin() + end_index);
    this->traj_index = std::distance(dist_traj.begin(), result);
    std::cout << "traj_index:    " << traj_index << std::endl;

    /*Position*/
    position_ref_enu = states_[traj_index-1].position_W + gamma[traj_index]*(states_[traj_index].position_W - states_[traj_index-1].position_W);

    /*Orientation*/
    accel = (states_[traj_index-1].acceleration_W + gamma[traj_index]*(states_[traj_index].acceleration_W - states_[traj_index-1].acceleration_W));
    orient_yaw = (quaternion_to_rpy_wrap(states_[traj_index-1].orientation_W_B) + gamma[traj_index]*(quaternion_to_rpy_wrap(states_[traj_index].orientation_W_B) - quaternion_to_rpy_wrap(states_[traj_index-1].orientation_W_B)));
    thrust = (Eigen::Vector3d(0,0,9.81) + accel);
    thrust_dir = (thrust.normalized());
    q_yaw = (Eigen::AngleAxisd(orient_yaw.z(),Eigen::Vector3d(0,0,1)));

    rpy_ref_enu = (quaternion_to_rpy_wrap(states_.at(traj_index-1).orientation_W_B) + gamma[traj_index]*(quaternion_to_rpy_wrap(states_.at(traj_index).orientation_W_B) - quaternion_to_rpy_wrap(states_.at(traj_index-1).orientation_W_B)));

    qref_enu =  Eigen::AngleAxisd(rpy_ref_enu.z(), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy_ref_enu.y(), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy_ref_enu.x(), Eigen::Vector3d::UnitX());

    /*Velocity*/
    velocity_enu << states_.at(traj_index-1).velocity_W.x() + gamma[traj_index]*(states_.at(traj_index).velocity_W.x() - states_.at(traj_index-1).velocity_W.x()),
                    states_.at(traj_index-1).velocity_W.y() + gamma[traj_index]*(states_.at(traj_index).velocity_W.y() - states_.at(traj_index-1).velocity_W.y()),
                    states_.at(traj_index-1).velocity_W.z() + gamma[traj_index]*(states_.at(traj_index).velocity_W.z() - states_.at(traj_index-1).velocity_W.z());

    /*Body rates*/
    Eigen::Vector3d angular_velocity_des_B;
    angular_velocity_des_B << states_.at(traj_index-1).angular_velocity_B + gamma[traj_index]*(states_.at(traj_index).angular_velocity_B - states_.at(traj_index-1).angular_velocity_B);


    xref(0) = position_ref_enu.x();
    xref(1) = position_ref_enu.y();
    xref(2) = position_ref_enu.z();

    xref(3) = qref_enu.w();
    xref(4) = qref_enu.x();
    xref(5) = qref_enu.y();
    xref(6) = qref_enu.z();

    xref(6) = velocity_enu.x();
    xref(7) = velocity_enu.y();
    xref(8) = velocity_enu.z();

    uref(0) = angular_velocity_des_B[0];
    uref(1) = angular_velocity_des_B[1];
    uref(2) = angular_velocity_des_B[2];
    uref(3) = thrust.norm();
    return false;
    }
}

// void LQR_Quaternion::convertToNED(state_vector_t& xref, control_vector_t& uref)
// {
//     // Convert position
//     std::swap(xref(0), xref(1));
//     xref(2) = -xref(2);

//     // Convert velocity
//     std::swap(xref(7), xref(8));
//     xref(9) = -xref(9);

//     // Convert orientation (quaternion)
//     Eigen::Quaterniond q_enu(xref(3), xref(4), xref(5), xref(6));
//     Eigen::Quaterniond q_ned = enuToNed(q_enu);
//     xref(3) = q_ned.w();
//     xref(4) = q_ned.x();
//     xref(5) = q_ned.y();
//     xref(6) = q_ned.z();

//     // Convert body rates
//     std::swap(uref(0), uref(1));
//     uref(2) = -uref(2);
// }

// Eigen::Quaterniond LQR_Quaternion::enuToNed(const Eigen::Quaterniond& q_enu)
// {
//     // ENU to NED conversion
//     Eigen::Matrix3d R_enu = q_enu.toRotationMatrix();
//     Eigen::Matrix3d R_ned;
//     R_ned << 0, 1, 0,
//              1, 0, 0,
//              0, 0, -1;
//     Eigen::Matrix3d R_ned_converted = R_ned * R_enu * R_ned.transpose();
//     return Eigen::Quaterniond(R_ned_converted);
// }

Eigen::Vector3d LQR_Quaternion::quaternion_to_rpy_wrap(const Eigen::Quaterniond& q)
{
  Eigen::Vector3d rpy;
  double roll = atan2(2*(q.w()*q.x()+q.y()*q.z()),1-2*(pow(q.x(),2)+pow(q.y(),2)));
  double pitch = asin(2*(q.w()*q.y()-q.z()*q.x()));
  double yaw = atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(pow(q.y(),2)+pow(q.z(),2)));

  rpy << roll,
         pitch,
         yaw;

  return rpy;
}

// void LQR_Quaternion::generateTrajectory(mav_msgs::EigenTrajectoryPoint::Vector& states)
// {
//   std::vector<double> segment_times;
//   mav_trajectory_generation::Vertex::Vector vertices;
//   const int deriv_to_opt = mav_trajectory_generation::derivative_order::SNAP;
//   mav_trajectory_generation::Trajectory trajectory;
//   Eigen::Vector3d rpy_enu = (quaternion_to_rpy_wrap(q_enu_));

//   mav_trajectory_generation::Vertex start(dimension), interm1(dimension), interm2(dimension), interm3(dimension), interm4(dimension),end(dimension);

//   start.makeStartOrEnd(Eigen::Vector3d(0,0,0-0.1), deriv_to_opt);
//   vertices.push_back(start);

//   interm1.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0,0,1));
//   vertices.push_back(interm1);

//   interm2.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(4,4,1));
//   interm2.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,1,0));
//   vertices.push_back(interm2);

//   interm3.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0,8,1));
//   interm3.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(-1,0,0));
//   vertices.push_back(interm3);

//   interm4.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-4,4,1));
//   interm4.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0,-1,0));
//   vertices.push_back(interm4);

//   end.makeStartOrEnd(Eigen::Vector3d(-4,0,1), deriv_to_opt);
//   vertices.push_back(end);

//   segment_times = estimateSegmentTimes(vertices, v_max, a_max);

//   //  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
//   //  parameters.max_iterations = 1000;
//   //  parameters.f_rel = 0.05;
//   //  parameters.x_rel = 0.1;
//   //  parameters.time_penalty = 500.0;
//   //  parameters.initial_stepsize_rel = 0.1;
//   //  parameters.inequality_constraint_tolerance = 0.1;

//   //  const int N = 10;
//   //  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
//   //  opt.setupFromVertices(vertices, segment_times, deriv_to_opt);
//   //  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
//   //  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
//   //  opt.optimize();

//   const int N = 10;
//   mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
//   opt.setupFromVertices(vertices, segment_times, deriv_to_opt);
//   opt.solveLinear();
//   mav_trajectory_generation::Segment::Vector segments;
//   opt.getSegments(&segments);

//   opt.getTrajectory(&trajectory);

//   bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

//   double distance = 1.0;
//   std::string frame_id = "local_origin";
//   mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers_);
// }

state_vector_t LQR_Quaternion::getError()
{
  return this->xerror_;
}

LQR_Quaternion::feedback_matrix_t LQR_Quaternion::getGain() const
{
    return this->Kold_;
}

void LQR_Quaternion::setOutput(double output,int j)
{
  this->output_(j) = output;
}

void LQR_Quaternion::setOutput(control_vector_t output)
{
  this->output_ = output;
}

control_vector_t LQR_Quaternion::getOutput()
{
  return this->output_;
}

state_vector_t LQR_Quaternion::getRefStates()
{
  return this->xref_;
}

control_vector_t LQR_Quaternion::getTrajectoryControl()
{
  return this->uref_;
}

std::vector<State>& LQR_Quaternion::getStates()
{
    return this->states_;
}

}
