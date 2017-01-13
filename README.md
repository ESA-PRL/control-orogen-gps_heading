# GPS heading

## Overview

This component takes in GPS readings and outputs the same GPS readings with a heading orientation (yaw) evaluated from two GPS readings, spaced by a user defined distance. Other orientation angles are canceled.

Is useful to get a rough estimate of the heading in absence of other absolute sensors. The component will however publish GPS data less often than the GPS component itself because of the minimum travel distance constraint.

A python script under `/test` has been made to test the output of this component. Use `gps_replay.rb` to get the `gps_heading` data log file, then `pocolog` to delog the generated data and finally export the data and process it to fit for the python script.

**Authors: Karl Kangur  
Contact: Martin Azkarate  
Affiliation: Automation and Robotics Laboratories, ESTEC, ESA**

## Installation

### Dependencies

This package does not have any dependencies.

### Building

In order to install, clone the latest version from this repository into your workspace under `control/orogen/gps_heading`, add the following line to `autoproj/manifest` under `layout:`

    - control/orogen/gps_heading

Execute the following to build the package:

    $ autoproj build


## Basic Usage

### gps_heading

#### Inputs

* **`gps`** (/base/samples/RigidBodyState)

GPS readings.

#### Outputs

* **`heading`** (/base/samples/RigidBodyState)

GPS positions with filtered orientation based on two GPS readings, spaced by `minimumDistanceHeading`.

#### Parameters

* **`minimumDistanceHeading`** (/double)

Minimum distance to travel between publishings (in meters).

