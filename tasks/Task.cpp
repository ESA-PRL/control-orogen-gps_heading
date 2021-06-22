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
    gyro_initialized = false;
    gps_new_sample  = false;
    calibrated = false;
    driving_forward = false;
    stopped=false;
    integrate_gyro=true;

    dist_min  = _dist_min.value();
    dist_min *=  dist_min; // Squared
    calibration_dist_min = _calibration_dist_min.value();
    calibration_dist_min *= calibration_dist_min; // Squared

    rtk_fix = false;
    using_gps_heading = false;
    gps_offset = _offset.value();
    alpha = _alpha.value();
    headingOffset = _heading_offset.value() * M_PI / 180.0f;
    stopping_threshold = _stop_integrating_time.value();

    return true;
}
bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }

    if(!_gyro_pose_samples.connected())
        gyro_initialized = true;

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    //----- Read new inputs -----
    if(_gps_raw_data.connected())
    {
        if(_gps_raw_data.readNewest(gps_raw_data) == RTT::NewData)
        //if(_gps_raw_data.read(gps_raw_data) == RTT::NewData)
        {
            rtk_fix = (gps_raw_data.positionType == gnss_trimble::RTK_FIXED);
            //printf("gps_heading: RTK status update: %d\n", rtk_fix);
        }
    }
    else if(!rtk_fix)
    {
        //printf("gps_heading: RTK not connected\n");
        // If the GPS raw data port is not connected force the flag to true
        rtk_fix = true;
    }

    if(_imu_pose_samples.readNewest(imu_pose) == RTT::NewData)
    //if(_imu_pose_samples.read(imu_pose) == RTT::NewData)
    {
        //printf("gps_heading: IMU update\n");
        if(!imu_initialized)
        {
            yawImuPrev = imu_pose.getYaw();
            yawCompensated = yawImuPrev; //ToDo Check with Karl if this should be yawImu=yawImuPrev; and initialize yawCompensated in start/configure hook
            imu_initialized = true;
        }
    }

    if(_gps_pose_samples.readNewest(gps_pose) == RTT::NewData)
    //if(_gps_pose_samples.read(gps_pose) == RTT::NewData)
    {
        //printf("gps_heading: GPS update\n");
        gps_new_sample = true;
        if(!gps_initialized)
        {
            gps_pose_prev = gps_pose;
            gps_initialized = true;
        }
    }
    
    if(_gyro_pose_samples.readNewest(gyro_pose) == RTT::NewData)
    //if(_gyro_pose_samples.read(gyro_pose) == RTT::NewData)
    {
        //printf("gps_heading: Gyro update\n");
        if(!gyro_initialized)
        {
            yawGyroPrev = gyro_pose.getYaw();
            //yawGyro = yawGyroPrev;
            gyro_initialized = true;
        }
    }

    if(_ground_truth_pose_samples.readNewest(ground_truth_pose) == RTT::NewData)
    //if(_ground_truth_pose_samples.read(ground_truth_pose) == RTT::NewData)
    {
        //printf("gps_heading: Ground Truth update\n");
        yawCompensated=ground_truth_pose.getYaw();
    }

    // If the motion command is connected then only allow heading to be compensated with GPS when the rover is moving forwards (not point turning)
    if(_motion_command.connected())
    {
        // TODO: also require the rover to be moving in an almost straight line for best GPS heading estimation
        //    printf("gps_heading: Motion command update, driving forward status: %d\n", driving_forward);
        //if(_motion_command.readNewest(motion_command) == RTT::NewData)
        _motion_command.read(motion_command);
        {
            // Make sure the rover is moving forwards, translation is 0.0 when doing a point turn
            // also only check for positive forwards motion as backwards would make the code unnecessarily complicated
            driving_forward = motion_command.translation > 0.0f;
        //    printf("gps_heading: Motion command update, driving forward status: %d\n", driving_forward);
            stopped = ((motion_command.translation == 0.0) && (motion_command.rotation == 0.0)); 
            if (stopped)
            {
                base::Time time_stopped = base::Time::now() - last_time_moving;
                //std::cout << " time stopped in seconds: " << time_stopped.toSeconds() << std::endl;
                if (time_stopped.toSeconds()>stopping_threshold)
                {
                    integrate_gyro=false;
                    //std::cout << " Stopped for long time. Do not integrate gyro." << std::endl;
                }
            }
            else
            {
                last_time_moving = base::Time::now();
                integrate_gyro=true;
                //std::cout << " Moving. Integrating gyro." << std::endl;
            }
        }
    }
    else
    {
        //printf("gps_heading: Motion command not commented\n");
        // Without motion command input, the GPS compensation is attempted
        // regardless of the type of the motion
        driving_forward = true;
        stopped=false;
        integrate_gyro=true;
    }

    //----- Heading estimation -----
    /* Create uncompensated attitude estimate:
    1) Keep roll & pitch angles from IMU orientation
    2) Get deltaYaw from the IMU orientation
    3) Yaw(k) = Yaw(k-1) + deltaYaw;
    */
    if(imu_initialized && gps_initialized && gyro_initialized && gps_new_sample)
    {
        double deltaYaw; // yawImu; defined it in the hpp
        yawImu = imu_pose.getYaw();
        deltaYaw = deltaHeading(yawImu, yawImuPrev);
        yawImuPrev = yawImu;

        yawGyro = gyro_pose.getYaw();
        deltaYaw = deltaHeading(yawGyro, yawGyroPrev);
        yawGyroPrev = yawGyro;

        if (integrate_gyro)
        {
            // Prediction step, integration of delta Yaw
            yawCompensated += deltaYaw;
        }
        
        //printf("gps_heading: Processing data new data...\n");

        // Compensation step using GPS readings, only used when driving forward and GPS had an RTK fix
        // During calibration (heading setting) one can stop the rover, but (should) not turn it
        // This is a special case for the one stops with the rover during initialisation, otherwise it will reset the starting position
        //if((driving_forward && rtk_fix) || (!driving_forward && rtk_fix && !calibrated))
        if (rtk_fix && !calibrated)
        {
            //printf("gps_heading: Driving forward and RTK fix or calibrating state\n");
            Eigen::Vector3d deltaPos = gps_pose.position - gps_pose_prev.position;
            // Ignore vertical distance delta
            deltaPos.z() = 0;

            // If enough distance was traveled
            // 0) Calibrate yaw value using GPS only for the first yaw value
            // 1) Estimate the heading from GPS
            // 2) Compensate the heading estimate
            // WARNING: the dist_min is quared here, so comparing it to squaredNorm is OK
            //if((calibrated && deltaPos.squaredNorm() > dist_min) || (!calibrated && deltaPos.squaredNorm() > calibration_dist_min))  
            if (deltaPos.squaredNorm() > calibration_dist_min)
            {
                // Get the yaw value from between 2 gps poses
                double yawGps = atan2(deltaPos.y(), deltaPos.x());

                //if(!calibrated)
                //{
                    // The first yaw output will be 100% GPS based to calibrate the initial value
                    yawCompensated = yawGps;
                    calibrated = true;
                    _ready.value() = true;
                //}
                /*else
                {
                    //printf("gps_heading: Applying GPS heading compensation\n");
                    
                    // Complementary filter
                    // Difference between current compensated estimate and new GPS-based heading
                    deltaYaw = deltaHeading(yawGps, yawCompensated);

                    // yawCompensated = alpha*(yawCompensated+deltaYaw) + (1-alpha)*(yawCompensated);
                    // Complementary filter equation simplified to:
                    yawCompensated = yawCompensated + alpha*deltaYaw;
                }

                // Save the new "previous" GPS position
                gps_pose_prev = gps_pose;
                */
                // Heading drift debug output
                double heading_drift;
                heading_drift = deltaHeading(yawImu, wrapAngle(yawCompensated));
                heading_drift *= 180.0/M_PI;
                _heading_drift.write(heading_drift);
                //std::cout << "Estimated heading drift: " << heading_drift << "deg." << std::endl;
                //using_gps_heading = true;
            }
        }
        else
        {
            //printf("gps_heading: Not using GPS compensation\n");
            // "Not driving_forward" resets the previous pose in every interation
            // Avoids taking two gps samples with a point turn in between.
            gps_pose_prev = gps_pose;
            using_gps_heading = false;
        }

        // Output pose estimate
        base::samples::RigidBodyState resulting_pose;
        resulting_pose.time = gps_pose.time;
        // Set the position as the GPS position
        resulting_pose.position = gps_pose.position;

        if(!calibrated)
        {
            // Add a 90 degree offset to point towards north when not calibrated
            yawCompensated = M_PI / 2;
        }
        else
        {
            // Add the GPS position offset configuration
            resulting_pose.position.x() += gps_offset(0) * cos(yawCompensated) - gps_offset(1) * sin(yawCompensated);
            resulting_pose.position.y() += gps_offset(0) * sin(yawCompensated) + gps_offset(1) * cos(yawCompensated);
            resulting_pose.position.z() += gps_offset(2);
        }

        // Add static heading offset
        yawCompensated += headingOffset;

        double roll, pitch;
        roll = -imu_pose.getRoll();
        pitch = -imu_pose.getPitch();
        resulting_pose.orientation = Eigen::Quaterniond(
            Eigen::AngleAxisd(wrapAngle(yawCompensated), Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));

      	resulting_pose.sourceFrame = _gnss_source_frame.get();
    	resulting_pose.targetFrame = _gnss_target_frame.get();
            
        //printf("gps_heading: Writing out gps_heading data\n");
        _pose_samples_out.write(resulting_pose);
        gps_new_sample = false;
    }

    // Conponent state changes
    if(!rtk_fix)
    {
        state(NO_RTK_FIX);
    }
    else if(!calibrated && rtk_fix)
    {
        state(CALIBRATING);
    }
    else if(calibrated && using_gps_heading)
    {
        state(GPS_HEADING);
    }
    else if(calibrated && !using_gps_heading)
    {
        state(GYRO_HEADING);
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
