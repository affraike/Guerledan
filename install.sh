echo "salut"

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

source /opt/ros/kinetic/setup.bash

echo "\n__Création du Workspace"
rm -rf /home/ue32/workspace_DDBoat
mkdir /home/ue32/workspace_DDBoat
mkdir /home/ue32/workspace_DDBoat/src
cd /home/ue32/workspace_DDBoat
catkin_make

echo "source /home/ue32/workspace_DDBoat/devel/setup.bash" >> ~/.bashrc

echo "\n____Création du package test_kalman_command"
echo "____Création du package gpsd_client"
cd /home/ue32/workspace_DDBoat/src
catkin_create_pkg test_kalman_command --rosdistro=kinetic
catkin_create_pkg gpsd_client --rosdistro=kinetic
catkin_create_pkg start --rosdistro=kinetic
catkin_create_pkg cap_boat --rosdistro=kinetic

unset http_proxy 
unset https_proxy

git clone https://github.com/affraike/Guerledan.git
cp -r /home/ue32/workspace_DDBoat/src/Guerledan/workspaceDDBoat/src/* .

rm -rf /home/ue32/workspace_DDBoat/src/Guerledan

echo "\n__compilation de catkin"
source /home/ue32/workspace_DDBoat/devel/setup.bash
cd /home/ue32/workspace_DDBoat
catkin_make -j2

echo "__Pour lancer, il faut tapper :"
echo "roslaunch start general.launch"

#bash