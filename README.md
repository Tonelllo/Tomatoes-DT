# Run
Per fare andare il simulatore:
```bash
roslaunch agri_challenge tiago.launch
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
**NOTA** al momento le posizioni sono riferite al frame della camera 
perche' manca una trasformazione, quindi sono sbagliate. 
In **Pose.orientation** ci sono altre informazioni:
* x: La classe del pomodoro
* y: L'id del pomodoro
* z: Il raggio in metri
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
* **Restore** recupera i valori dallo stesso file
**NOTA** Valori buoni per la simulazione ci sono gia' nel file quindi basta
premere restore per vedere 
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
- A destra una visualizzazione 3D dei pomodori.
**NOTA** Al momento sono ruotati perche' il frame e' sbagliato. <br/>
La visualizzazione puo' essere ruotata, e zoommata

# Tomato Gui installazione
Possibile che manchi questa libreria
```bash
sudo apt install libxcb-cursor-dev
```
# Octomap in simulazione
In caso in cui non si metta `use_moveit_camera:=true` al momento di far partire
il nodo o con il robot vero non configurato per generare octomap c'e' il nodo
tomato_octo e in particolari il launch file ```tomato_octo.launch```. Lanciando
questo parte l'octomap_server e il nodo responsabile per dare alla PlanningScene
la octomap.

In caso si faccia andare in simulazione e' necessario anche far andare il
seguente comando:

``` bash
rosrun topic_tools throttle messages /xtion/depth_registered/points 3  /throttle_filtering_points/filtered_points
```
Nel robot vero viene gia' pubblicato
