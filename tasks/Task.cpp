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
    if (!TaskBase::configureHook())
    {
        return false;
    }
    firstDataPoint = true;
    return true;
}

bool Task::startHook()
{
    if(! TaskBase::startHook())
    {
        return false;
    }
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();    
    
    if(_gps.read(gps) == RTT::NewData)
    {
        // This is used to initialise the last published data point
        if(firstDataPoint)
        {
            last_published_position = gps;
            firstDataPoint = false;
        }
        
        // Evaluate the distance between the new GPS data point and the point used during last publishing
        double distance = (gps.position - last_published_position.position).norm();
        
        // See if the new GPS position is at least n meters away from the last point where the heading was published
        if(distance >= _minimumDistanceHeading.get())
        {
            // Evaluate the new heading
            heading_angle = atan2(gps.position[1] - last_published_position.position[1], gps.position[0] - last_published_position.position[0]);
            
            // Create a quatermion
            orientation = Eigen::Quaterniond::Identity();
            
            // Rotate the yaw angle by the heading evaluated before
            orientation = Eigen::AngleAxisd(heading_angle, Eigen::Vector3d::UnitZ()) * orientation;
            
            // Overwrite the GPS orientation with only the heading info
            gps.orientation = orientation;
            
            // Publish the heading
            _heading.write(gps);
            
            // Save the last published GPS position
            last_published_position = gps;
        }
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

