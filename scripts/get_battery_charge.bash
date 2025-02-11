#!/bin/bash

# Get one message from the /power_state topic.
message=$(rostopic echo -n1 /power_state)

# Extract the lines that contain the desired fields.
voltage_line=$(echo "$message" | grep "voltage:")
capacity_line=$(echo "$message" | grep "relative_remaining_capacity:")

# Output the two lines to the console.
echo "$voltage_line"
echo "$capacity_line"
