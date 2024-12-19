#!/bin/bash

# change in ros workspace
cd /home/robotrainer/workspace/ros_ws_melodic_robotrainer/src

# Iterate through all items in the current directory
for dir in */; do
    # Check if the item is a directory
    if [ -d "$dir" ]; then
        # Enter the directory
        cd "$dir" || continue

        # Check if it is a Git repository
        if [ -d ".git" ]; then
            # Check if the specific remote exists
            if git remote | grep -q "robotrainer_github"; then
                # Fetch from the remote
                git fetch robotrainer_github

                # Output status with colors in a single line
                echo -e "\e[1mStatus for $dir:"
                git status -sb
            else
                echo "Remote 'robotrainer_github' not found in $dir. Skipping."
            fi
        else
            echo "$dir is not a Git repository. Skipping."
        fi

        # Return to the parent directory
        cd ..
    fi

done
