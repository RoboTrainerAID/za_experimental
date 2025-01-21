#!/bin/bash

# Define paths to check
paths=(
    "/home/robotrainer/workspace/ros_ws_melodic_robotrainer/src"
    "/home/robotrainer/workspace/docker"
)

# List the paths being checked
echo -e "\e[1mChecking repositories in the following paths:\e[0m"
for path in "${paths[@]}"; do
    echo "$path"
done

# Iterate through defined paths
for base_path in "${paths[@]}"; do
    echo -e "\e[1mChecking repositories in: $base_path\e[0m"

    # Change to the base directory
    cd "$base_path" || {
        echo "Could not access $base_path. Skipping."
        continue
    }

    # Iterate through all items in the current directory
    for dir in */; do
        # Check if the item is a directory
        if [ -d "$dir" ]; then
            # Enter the directory
            cd "$dir" || continue

            # Check if it is a Git repository
            if [ -d ".git" ]; then
                # Get list of remotes
                remotes=($(git remote))

                # Handle remote selection logic
                if echo "${remotes[@]}" | grep -q "robotrainer_github"; then
                    # Use the specific remote if available
                    git fetch robotrainer_github
                    # Output status with colors in a single line
                    echo -e "\e[1mStatus for $dir:"
                    git status -sb
                elif [ ${#remotes[@]} -eq 1 ]; then
                    # Use the only available remote
                    git fetch "${remotes[0]}"
                    # Output status with colors in a single line
                    echo -e "\e[1mStatus for $dir:"
                    git status -sb
                else
                    # Warn and skip if multiple remotes exist and none is the target
                    echo -e "\e[31mWarning: Multiple remotes found in $dir, but 'robotrainer_github' is not among them. Skipping.\e[0m"
                fi
            else
                echo "$dir is not a Git repository. Skipping."
            fi

            # Return to the parent directory
            cd ..
        fi
    done

done

echo "Done!"
