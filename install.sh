echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

source /opt/ros/kinetic/setup.bash

echo "__Création du Workspace"
rm -rf /home/ue32/workspaceDDBoat
mkdir /home/ue32/workspaceDDBoat
mkdir /home/ue32/workspaceDDBoat/src
cd /home/ue32/workspaceDDBoat
catkin_make

echo "source /home/ue32/workspaceDDBoat/devel/setup.bash" >> ~/.bashrc

cd /home/ue32/workspaceDDBoat/src
catkin_create_pkg test_kalman_command --rosdistro=kinetic
catkin_create_pkg gpsd_client --rosdistro=kinetic
catkin_create_pkg start --rosdistro=kinetic
catkin_create_pkg cap_boat --rosdistro=kinetic
catkin_create_pkg encoders_boat --rosdistro=kinetic

#récupération des codes 
unset http_proxy 
unset https_proxy

git clone https://github.com/affraike/Guerledan.git
cp -r /home/ue32/workspaceDDBoat/src/Guerledan/workspaceDDBoat/src/* .
rm -rf /home/ue32/workspaceDDBoat/src/Guerledan

echo " "
echo "__compilation de catkin"
source /home/ue32/workspaceDDBoat/devel/setup.bash
cd /home/ue32/workspaceDDBoat

catkin_make --pkg gpsd_client encoders_boat
catkin_make --pkg start cap_boat
catkin_make --pkg test_kalman_command
#catkin_make --cmake-args -Wno-dev 

echo "____________________________________________________________________________________"
echo "Pour lancer, il faut tapper :"
echo "roslaunch start general.launch"

bash
