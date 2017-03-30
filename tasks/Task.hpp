/****************************************************************
 *
 * Copyright (c) 2017
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class for GPS-based heading drift compensation.
 * The orinetation is obtained from IMU without a magnetometer.
 * The heading drift is compensated using high-precision DGPS
 * measurements during rover motion.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Jan 2017
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */



#ifndef GPS_HEADING_TASK_TASK_HPP
#define GPS_HEADING_TASK_TASK_HPP

#include "gps_heading/TaskBase.hpp"

namespace gps_heading {


    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        base::samples::RigidBodyState gps_pose, gps_pose_prev;
        base::samples::RigidBodyState imu_pose;
        double yawCompensated, yawImuPrev, dist_min, calibration_dist_min;
        bool imu_initialized, gps_initialized, gps_new_sample, driving_forward, calibrated;
        base::MotionCommand2D motion_command;
        bool gps_fix;

        double deltaHeading(double yaw, double yaw_prev);
        inline double wrapAngle(double angle);

    public:

        Task(std::string const& name = "gps_heading::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
        ~Task();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
