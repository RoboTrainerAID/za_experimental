eval "$(ssh-agent -s)"
ssh-add ~/.ssh/za_robotrainer
cd /home/robotrainer/workspace/ros_ws_melodic_robotrainer/src/za_experimental
git push