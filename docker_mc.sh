#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /ws/devel/setup.bash


#=============================================================
#=============================================================
#=============================================================


# dataset locations
bagnames=(
    "V1_01_easy"
    "V1_02_medium"
    "V1_03_difficult"
    "V2_01_easy"
    "V2_02_medium"
    "V2_03_difficult"
#    "dataset-room1_512_16"
#    "dataset-room2_512_16"
#    "dataset-room3_512_16"
#    "dataset-room4_512_16"
#    "dataset-room5_512_16"
#    "dataset-room6_512_16"
)

# location to save log files into
save_path="/datasets/mars/results"
bag_path="/datasets/eth"
#bag_path="/datasets/tum"


#=============================================================
#=============================================================
#=============================================================


# Loop through all datasets
for i in "${!bagnames[@]}"; do

# Monte Carlo runs for this dataset
for j in {00..09}; do

# start timing
start_time="$(date -u +%s)"
filename="$save_path/mono_mars/${bagnames[i]}/${start_time}_estimate.txt"

# run our ROS launch file (note we send console output to terminator)
roslaunch mars_vins euroc.launch bag:="$bag_path/${bagnames[i]}.bag" output:="$filename" do_save:="true" &> /dev/null
#roslaunch mars_vins tumvi.launch bag:="$bag_path/${bagnames[i]}.bag" output:="$filename" do_save:="true" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${bagnames[i]} - run $j took $elapsed seconds";


done

done


