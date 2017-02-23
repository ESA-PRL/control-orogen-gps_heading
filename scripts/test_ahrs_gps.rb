#! /usr/bin/env ruby
require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'rock/bundle'
include Orocos


#Initializes the CORBA communication layer
Orocos::CORBA::max_message_size = 100000000
Bundles.initialize
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'


Bundles.run 'gps_heading::Task' => 'ahrs_gps',
            'simple_pose::Task' => 'simple_pose',
            "valgrind" => false,
            'output' => nil, 
            "wait" => 60                    do

    # get the task
    puts "Setting up components..."
    ahrs_gps    = TaskContext::get 'ahrs_gps'
    simple_pose = TaskContext::get 'simple_pose'
    
    ahrs_gps.apply_conf_file("config/ahrs_gps::Task.yml",   ["default"])
    ahrs_gps.configure
    simple_pose.configure

    puts ".. done!"
    
    puts "Using preset path to logs:"
    log_replay  = Orocos::Log::Replay.open( 'gps.log',
                                            'imu.log')
    
    #log_replay.use_sample_time = true
    
    # Maps the logs into the input ports
    log_replay.gps.pose_samples.connect_to(ahrs_gps.gps_pose_samples)
    log_replay.gps.pose_samples.connect_to(simple_pose.gps_pose)
    log_replay.imu_stim300.orientation_samples_out.connect_to(ahrs_gps.imu_pose_samples)
    log_replay.imu_stim300.orientation_samples_out.connect_to(simple_pose.imu_pose)
    
    # Run the task
    ahrs_gps.start
    simple_pose.start
    

    # open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 10

    Vizkit.display ahrs_gps.pose_samples_out, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display ahrs_gps.pose_samples_out, :widget => Vizkit.default_loader.TrajectoryVisualization
    Vizkit.display simple_pose.pose, :widget => Vizkit.default_loader.RigidBodyStateVisualization

    Vizkit.display ahrs_gps.pose_samples_out, :widget => Vizkit.default_loader.OrientationView
    Vizkit.display simple_pose.pose, :widget => Vizkit.default_loader.OrientationView


    Vizkit.display ahrs_gps.heading_drift, :widget => Vizkit.default_loader.Plot2d
    #Vizkit.display simple_pose.pose, :widget => Vizkit.default_loader.TrajectoryVisualization
    
    #Vizkit.display ahrs_gps.pose_samples_out, :widget => Vizkit.default_loader.TrajectoryVisualization
    #Vizkit.display simple_pose.pose, :widget => Vizkit.default_loader.RigidBodyStateVisualization

    Vizkit.exec

end
