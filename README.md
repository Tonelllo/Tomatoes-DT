# Run
Per fare andare il simulatore:
```bash
roslaunch agri_challenge tiago.launch use_moveit_camera:=true
```
Per fare andare la parte di visione
```bash
roslaunch tomato_detection vision_manager.launch
```
Per la gui invece scaricare dall'ultimo release gui.zip che si trova sotto gli
asset.
Questo per farvi evitare di scaricare tutto qt.
Poi nella cartella sotto /bin trovate l'eseguibile
```bash
./tomato_guiApp
```
# Topic dei pomodori
```bash
rostopic echo "/tomato_vision_manager/tomato_position"
```
## Elementi:
In **Pose.position** c'e' la posizione del pomodoro rispetto al 
frame: **"base_footprint"** <br/>

In **Pose.orientation** ci sono altre informazioni:
* x: La classe del pomodoro
* y: L'id del pomodoro
* z: Il diametro in metri <br/>
Con classe del pomodoro si intende lo stadio di maturazione:
- 0 -> maturo
- 1 -> mezzo maturo
- 2 -> verde

# GUI
Realizzata con QT/QML <br/>
Per muoversi tra le pagine basta fare uno swipe con il mouse.
## Pagina centrale
Segmentazione basata sul colore. 
* **Save** salva i parametri correnti sotto: `VisionConfig/config.toml`
* **Restore** recupera i valori dallo stesso file<br/>
**NOTA** Valori buoni per la simulazione ci sono gia' nel file quindi basta
premere restore per vedere la maschera<br/>
In alto a destra ci sono tab che mostrano diverse visualizzazioni:
* Camera senza filtri
* Effetto della maschera sull'immagine
* Visualizzazione dei cerchi che indicano i Pomodori
## Pagina di sinistra
Pulsante "HOME" che fa andare il robot in configurazione base. **NOTA** Ci mette
un po' a partire, lasciatelo fare. Quando avra' finito il movimento sotto il
pulsante ci viene scritto l'output del movimento. Non sono riuscito a farlo
interattivo.
## Pagina di destra
Pagina dedicata a YOLO. 
- A sinistra il risultato della detection del modello
- A destra una visualizzazione 3D dei pomodori. <br/>
La visualizzazione puo' essere ruotata, e zoommata

# Tomato Gui installazione
Possibile che manchi questa libreria
```bash
sudo apt install libxcb-cursor-dev
```
# MoveIt
Prima di poter usare MoveIt con la collision detection ci sono da fare delle
modifiche ai file di Tiago:<br/>
Nel file:
```bash
<TiagoTutorialsInstallDir>/src/tiago_moveit_config/config/sensors_rgbd.yaml
```
Ci sono degli esempi di configurazione di octomap che si basano su diversi tipi di sensore.
A noi interessa quella che si basa su PointCloud2. Consiglio anzi che modificare questo file
di crearne un altro dove mettere la nostra configurazione.<br/>
Creare quindi sempre nella directory `<TiagoTutorialsInstallDir>/src/tiago_moveit_config/config/`
il file `sensors_pointcloud.yaml` e metterci dentro:
```yaml
sensors:
 - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
   point_cloud_topic: /xtion/depth_registered/points
   max_range: 2.5
   octomap_resolution: 0.02
   padding_offset: 0.03
   padding_scale: 1.0
   point_subsample: 1
   filtered_cloud_topic: output_cloud
```
Nel file:
```bash
<TiagoTutorialsInstallDir>/src/tiago_moveit_config/launch/tiago_moveit_sensor_manager.launch.xml
```
Settare la risoluzione dell'octomap a 0.02m:
```xml
  <param name="octomap_resolution" type="double" value="0.02" />
```
**NOTA** SE si e' creato il nuovo file `sensors_pointcloud.yaml` c'e' da modificare
la riga del rosparam con:
```xml
  <rosparam command="load" file="$(find tiago_moveit_config)/config/sensors_pointcloud.yaml" />
```
Alla fine il file `tiago_moveit_sensor_manager.launch.xml` deve apparire come:
```xml
<launch>

  <rosparam command="load" file="$(find tiago_moveit_config)/config/sensors_pointcloud.yaml" />

  <param name="octomap_frame" type="string" value="odom" />
  <param name="octomap_resolution" type="double" value="0.02" />
  <param name="max_range" type="double" value="5.0" />

</launch>
```
Per avere la collision dectection di MoveIt il launch file di agri_challenge va 
fatto partire settando un parametro aggiuntivo quindi:
```bash
roslaunch agri_challenge tiago.launch use_moveit_camera:=true
```
# Rviz
Ho fatto una configurazione di rviz dove ci sono tutte le cose che ci servono.
Per avere rviz gia settata inserire a riga 31 nel launchfile `tiago.launch` di 
agri_challenge:
```xml
  <include file="$(find tomato_detection)/launch/rviz.launch"/>
```

# Robot Vero
Aggiungere in fondo al ~/.bashrc queste due righe:

``` bash
export ROS_MASTER_URI=http://10.68.0.1:11311
export ROS_HOSTNAME=10.68.0.130
```
**ATTENZIONE** Le simulazioni non vanno con queste due righe. In caso commentale
e chiudi e riapri terminator. Altrimenti fatti il source del bashrc in tutti i
terminali<br/>

**ATTENZIONE** ROS_HOSTNAME non necessariamente e' quello scritto li'. Dopo
esserti connesso alla rete wifi di Tiago controllare che corrisponda con:

``` bash
ip addr
```
Sotto wlp... dovresti vedere l'ip corretto.
## Cose da sapere
+ Quando si spegne bisogna essere in due. Uno che gli regge le braccia e uno che effettivamente lo spenge
+ Attenzione che le self-collision fanno pena
+ **ATTENZIONE** La detection delle collisioni non e' abilitata di default. Per abilitarla andare nell'interfaccia web e abilitarla in startup extra
+ Per disattivare il movimento idle della testa andare nell'interfaccia web e disabilitare head_manager
## Interfaccia web
$ROS_MASTER_URI con porta 8080: `http://10.68.0.1:8080`
## Password rete Tiago
`TheEngineRoom`
