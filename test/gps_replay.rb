#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'

include Orocos

if ARGV.size < 1 then
    puts "usage: rock_replay.rb <data_log_directory>"
    exit
end

Bundles.initialize

Orocos::Process.run 'gps_heading::Task' => 'gps_heading' do

    gps_heading = TaskContext.get 'gps_heading'
    Orocos.conf.apply(gps_heading, ['default'], :override => true)
    gps_heading.configure

    # Connect the tasks to the logs
    log_replay = Orocos::Log::Replay.open(ARGV[0])

    log_replay.gps.pose_samples.connect_to gps_heading.gps

    # Log the component output
    gps_heading.log_all_ports
    
    gps_heading.start

    # Open the log replay widget
    control = Vizkit.control log_replay
    control.speed = 1000

    Vizkit.exec

end
