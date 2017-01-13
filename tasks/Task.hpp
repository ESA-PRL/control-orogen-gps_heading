#ifndef GPS_HEADING_TASK_TASK_HPP
#define GPS_HEADING_TASK_TASK_HPP

#include "gps_heading/TaskBase.hpp"

namespace gps_heading {
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        base::samples::RigidBodyState gps;
        base::samples::RigidBodyState last_published_position;
        
        double heading_angle;
        base::Orientation orientation;
        bool firstDataPoint;

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

