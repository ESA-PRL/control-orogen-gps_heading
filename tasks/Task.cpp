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
 * Modified
 * - 2017-02-06: Karl Kangur - Added calibration routine.
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */


#include "Task.hpp"

using namespace gps_heading;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if(!TaskBase::configureHook())
    {
        return false;
    }
    imu_initialized = false;
    gps_initialized = false;
    gps_new_sample  = false;
    calibrated = false;
    driving_forward = false;
    dist_min  = _dist_min.value();
    dist_min *=  dist_min; // Squared
    calibration_dist_min = _calibration_dist_min.value();
    calibration_dist_min *= calibration_dist_min; // Squared
    return true;
}
bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    //----- Calibration state swiching routine -----
    if(!calibrated)
    {
        if(state() != CALIBRATING)
        {
            state(CALIBRATING);
        }
    }
    else if(calibrated && state() == CALIBRATING)
    {
        // Once calibrated switch to running state
        state(RUNNING);
    }
    
    //----- Read new inputs -----
    int rtt_return;
    base::samples::RigidBodyState pose;
    rtt_return = _imu_pose_samples.readNewest(pose);
    if(rtt_return == RTT::NewData && rtt_return != RTT::NoData)
    {
        imu_pose = pose;
        if(!imu_initialized)
        {
            yawImuPrev = imu_pose.getYaw();
            yawCompensated = yawImuPrev;
            imu_initialized = true;
        }
    }
    rtt_return = _gps_pose_samples.readNewest(pose);
    if(rtt_return == RTT::NewData && rtt_return != RTT::NoData)
    {
        gps_pose = pose;
        gps_new_sample = true;
        if(!gps_initialized)
        {
            gps_pose_prev = gps_pose;
            gps_initialized = true;
        }
    }

    // If the motion command is connected then only allow heading to be compensated with GPS when the rover is moving forwards (not point turning)
    if(_motion_command.connected())
    {
        // TODO: also require the rover to be moving in an almost straight line for best GPS heading estimation
        if(_motion_command.readNewest(motion_command) == RTT::NewData)
        {
            // Make sure the rover is moving forwards, translation is 0.0 when doing a point turn
            // also only check for positive forwards motion as backwards would make the code unnecessarily complicated
            driving_forward = motion_command.translation > 0.01;
        } 
    }
    else
    {
        // Without motion command input, the GPS compensation is attempted 
        // regardless of the type of the motion
        driving_forward = true;
    }

    //----- Heading estimation -----
    /* Create uncompensated attitude estimate:
    1) Keep roll & pitch angles  from IMU orientation
    2) Get deltaYaw from the IMU orientation
    3) Yaw(k) = Yaw(k-1) + deltaYaw;
    */
    if(imu_initialized && gps_initialized && gps_new_sample)
    {
        double deltaYaw, yawImu;
        yawImu = imu_pose.getYaw();
        deltaYaw = deltaHeading(yawImu, yawImuPrev); 
        yawImuPrev = yawImu;

        // Prediction step, integration of delta Yaw
        yawCompensated += deltaYaw;     

        // Compensation step
        if(driving_forward)
        {
            Eigen::Vector3d deltaPos = gps_pose.position - gps_pose_prev.position;
            // Ignore vertical distance delta
            deltaPos.z() = 0;

            // If enough distance was traveled
            // 0) Calibrate yaw value using GPS only for the first yaw value
            // 1) Estimate the heading from GPS
            // 2) Compensate the heading estimate
            // WARNING: the dist_min is quared here, so comparing it to squaredNorm is OK
            if(
                (calibrated && deltaPos.squaredNorm() > dist_min) ||
                (!calibrated && deltaPos.squaredNorm() > calibration_dist_min)
            )
            {
                double yawGps = atan2(deltaPos.y(), deltaPos.x());
                
                if(!calibrated)
                {
                    // The first yaw will be 100% GPS based to calibrate the initial value
                    yawCompensated = yawGps;
                    calibrated = true;
                }
                else
                {
                    // Complementary filter
                    double alpha = _alpha.value();
                    
                    // Difference between current compensated estimate and new GPS-based heading
                    deltaYaw = deltaHeading(yawGps, yawCompensated);
                    
                    // yawCompensated = alpha*(yawCompensated+deltaYaw) + (1-alpha)*(yawCompensated); 
                    // Complementary filter equation simplified to:
                    yawCompensated = yawCompensated + alpha*deltaYaw;
                }
                
                // Save the new "previous" GPS position
                gps_pose_prev = gps_pose;

                // Heading drift debug output 
                double heading_drift;
                heading_drift = deltaHeading(yawImu, wrapAngle(yawCompensated));
                heading_drift *= 180.0/M_PI;
                _heading_drift.write(heading_drift);
                std::cout << "Estimated heading drift: " << heading_drift << "deg." << std::endl;
            }
        }
        else
        {
            // "Not driving_forward" resets the previous pose in every interation
            // Avoids taking two gps samples with a point turn in between.
            gps_pose_prev = gps_pose;
        }

        // Output pose estimate
        base::samples::RigidBodyState resulting_pose;
        resulting_pose.time = gps_pose.time;
        double roll, pitch;
        roll    = imu_pose.getRoll();
        pitch   = imu_pose.getPitch();

        // TODO: here the position of the antenna could be roll/pitch compensated
        resulting_pose.position = gps_pose.position;

        if(!calibrated)
        {
            yawCompensated = M_PI / 2;
        }
        
        resulting_pose.orientation = Eigen::Quaterniond( 
            Eigen::AngleAxisd(wrapAngle(yawCompensated), Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));    
        _pose_samples_out.write(resulting_pose);
        gps_new_sample = false;
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

// Maintains the unwrapped heading so that it can be 
// low-pass filtered even during +- PI transitions
double Task::deltaHeading(double yaw, double yaw_prev)
{
    double delta;
    delta = yaw - yaw_prev;
    delta = wrapAngle(delta);
    return delta;
}

// Wraps the angle to [-pi; +pi) interval
inline double Task::wrapAngle(double angle)
{
    if(angle > M_PI || angle <= -M_PI)
    {
        double intp;
        const double side = copysign(M_PI,angle);
        angle = -side + 2*M_PI * modf( (angle-side) / (2*M_PI), &intp ); 
    }
    return angle;
}
