#!/bin/bash

rosparam delete /robot_description
rosparam load ./robot_description_wo_inertial /robot_description
