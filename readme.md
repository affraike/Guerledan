# Salut, c'est le readme

Consigne d'installation:

## installation et configuration des drivers gps:
connexion sur le DDBoat avec les droits pi:
```
ssh pi@172.20.25.2XX
password : pi
```
### installation des bibliothèques pour le GPS:

A lancer depuis le DDBoat :
```
unset http_proxy
unset https_proxy
```

(facultatif, pour tester):
```
sudo apt install gpsd-clients
```
(obligatoire): 
```
sudo apt install libgps-dev 
```
### modification d'un fichier pour lancer le GPS au démarrage du DDBoat

```
sudo nano /etc/default/gpsd
```

Modifier comme suit :
____________________
#Default settings for the gpsd init script and the hotplug wrapper.

#Start the gpsd daemon automatically at boot time

START_DAEMON="true"

#Use USB hotplugging to add new USB devices automatically to the daemon

USBAUTO="false"

#Devices gpsd should collect to at boot time.
#They need to be read/writeable, either by user gpsd or the group dialout.

DEVICES="/dev/ttyS0"

#Other options you want to pass to gpsd

GPSD_OPTIONS=""
_________________________

## Pour lancer l'installation :

il faut telecharger le install.sh
ensuite, envoyez le fichier sur le DDBoat
* depuis l'ordinateur :
```
scp -r ./install.sh ue32@172.20.25.2XX:~/ 
```
ensuite, connectez vous au DDBoat :
```
ssh ue32@172.20.25.2XX
```
Password : ue32

* depuis le DDBoat:

Donnez les droits d'éxecution au fichier et le lancer :
```
chmod +x install.sh

./install.sh
```

## Calibration pour le cap

cf Robin et Agathe:

Copier obligatoirement le dossier Driver_IMU dans ~/cpp sur le DDBoat

* Sur le bateau : 
```
mkdir cpp
```

* Sur le PC : 
```
scp -r Driver_IMU ue32@172.20.25.2XX:~/cpp
scp cal.sh ue32@172.20.25.2XX:~/cpp
scp -r ROS ue32@172.20.25.2XX:~/cpp  (optionnel)
```

Pour calibrer : 

```
cd cpp
chmod +x cal.sh
./cal.sh
```

Faire des rotations lentes du bateau sur les trois axes et dans les deux sens jusqu'à ce que les valeurs pour l'axe n'evoluent plus.

Presser ENTREE
Presser Y puis ENTREE
```
rosrun cap_boat cap_talker
rostopic echo cap
```


<h1>et Voilà</h1>

## Troubleshooting

* Probleme lors du launch :

```
ERROR: cannot launch node of type [test_kalman_command/arduino_driver_py3_Ros]: can't locate node 
```

Fix :

Donner les droits d'éxecution aux fichiers .py

```
cd /home/ue32/workspaceDDBoat/src/test_kalman_command/
chmod +x *.py
cd src
chmod +x *.py
```
* Freeze pendant l'installation lors du catkin_make

Fix :

Quitter et relancer manuellement le catkin_make

```
cd /home/ue32/workspaceDDBoat/
catkin_make
```

* Pas de GPS

Fix :

Bien vérifier que toutes les installations et le fichier ont été bien réalisées/modifiées.
Vérifier que le bateau a une vue sur le ciel.

Sinon, se connecter en mode pi :

```
sudo reboot -f
```
Et attendre le redémarrage du DDBoat, parfois ça aide



