Salut, c'est le readme

Consigne d'installation:

#installation et configuration des drivers gps:
connexion sur le DDBoat avec les droits pi:
ssh pi@172.20.25.2XX
password : pi

unset http_proxy
unset https_proxy

(facultatif, pour tester):
sudo apt install gpsd-clients
(obligatoire): 
sudo apt install libgps-dev 

modifier un fichier : (toujours en mode pi)
sudo nano /etc/default/gpsd

____________________
# Default settings for the gpsd init script and the hotplug wrapper.

# Start the gpsd daemon automatically at boot time
START_DAEMON="true"

# Use USB hotplugging to add new USB devices automatically to the daemon
USBAUTO="false"

# Devices gpsd should collect to at boot time.
# They need to be read/writeable, either by user gpsd or the group dialout.
DEVICES="/dev/ttys0"

# Other options you want to pass to gpsd
GPSD_OPTIONS=""
_________________________

#Pour lancer l'installation, il faut telecharger le install.sh

ensuite, tappez :
scp -r ./install.sh ue32@172.20.25.2XX:~/ 
pour envoyer le fichier sur le DDBoat

ensuite, connectez vous à celui-ci :
ssh ue32@172.20.25.2XX
Password : ue32

sur le DDBoat:

Donnez les droits d'éxecution au fichier
chmod +x install.sh

./install.sh

#Calibration pour le cap

cf Robin et Agathe:

Copier obligatoirement le dossier Driver_IMU dans ~/cpp sur le DDBoat
Sur le bateau : 
mkdir cpp

Sur le PC : 
scp -r Driver_IMU ue32@172.20.25.2XX:~/cpp
scp cal.sh ue32@172.20.25.2XX:~/cpp
scp -r ROS ue32@172.20.25.2XX:~/cpp  (optionnel)

Pour calibrer : 
cd cpp
chmod +x cal.sh
./cal.sh

Faire des rotations lentes du bateau sur les trois axes et dans les deux sens jusqu'à ce que les valeurs pour l'axe n'evoluent plus.

Presser ENTREE
Presser Y puis ENTREE

rosrun cap_boat cap_talker
rostopic echo cap



et Voilà



