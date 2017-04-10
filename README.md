# GPS heading

## Overview

This component takes in GPS readings, gyroscope and rover motion commands and outputs a fused (via a complementary filter) heading value between two GPS readings and the gyroscope. Other orientation angles are canceled (pitch, roll).

The component needs an RTK fix to start calibration (otherwise it goes into `NO_RTK_FIX` mode), then it will go into `CALIBRATING` state, until the platform moves `calibration_dist_min` in a straight line. After calibration is done the component can be in either `GPS_HEADING` state or `GYRO_HEADING` state. As soon as the platform stops or does a point turn the component goes into `GYRO_HEADING` state where it only considers the gyroscope heading. Once the platform moves at least a distance of `dist_min` meters it goes into `GPS_HEADING` where it fuses the GPS heading and the gyroscope.

![GPS heading evaluating heading from current and previous GPS readings](gps_heading.png "GPS heading")

A python script under `/test` has been made to test the output of this component. Use `gps_replay.rb` to get the `gps_heading` data log file, then `pocolog` to delog the generated data and finally export the data and process it to fit for the python script.

**Authors: [Karl Kangur](mailto:karl.kangur@gmail.com "Contact the author"), [Jan Filip](mailto:jan.filip2@gmail.com "Contact the author")  
Contact: [Martin Azkarate](mailto:Martin.Azkarate@esa.int "Contact the maintainer")  
Affiliation: Automation and Robotics Laboratories, ESTEC, ESA**

## Installation

### Dependencies

This package depends on the following packages:

* [drivers/orogen/gnss_trimble](https://github.com/rock-drivers/drivers-orogen-gnss_trimble)

### Building

In order to install, clone the latest version from this repository into your workspace under `control/orogen/gps_heading`, add the following line to `autoproj/manifest` under `layout:`

    - control/orogen/gps_heading

Execute the following to build the package:

    $ autoproj build


## Basic Usage

### gps_heading

#### Inputs

* **`gps_pose_samples`** (/base/samples/RigidBodyState) - GPS readings.
* **`imu_pose_samples`** (/base/samples/RigidBodyState) - Uncompensated input pose of the IMU.
* **`motion_command`** (/base/MotionCommand2D) - Rover motion control commands.

#### Outputs

* **`pose_samples_out`** (/base/samples/RigidBodyState) - GPS positions with filtered orientation.
* **`heading_drift`** (/double) - Estimated drift of the heading.

#### Parameters

* **`alpha`** (/double) - Complementary filter weight, a value of 0 means only the gyroscope is taken in account, a value of 1 means only the GPS heading is considered. A value 0.1 is a good compromise.
* **`dist_min`** (/double) - Minimum distance to travel between yaw updates (in meters).
* **`calibration_dist_min`** (/double) - Minimum distance to travel for the first heading estimation (calibration), should be higher than `dist_min`.
* **`ready`** (/bool) - Flag indicating if the component is ready, by default set to `false`, after calibration automatically set to `true`.
* **`offset`** (/base/Vector3d) - GPS position offset (x, y, z) with regards to the platform it is mounted on. `Vector3d` format must be set in the configuration file as follows:


    offset:
      data:
        - 0.6
        - 0
        - -0.2
