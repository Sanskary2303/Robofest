# Robofest

## Running the Simulation

### Run the MAVROS
Run the following command in the 3 terminals 

```bash
sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I1 --out=tcpin:0.0.0.0:8000
```

```bash
sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I2 --out=tcpin:0.0.0.0:8100
```
```bash
sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I3 --out=tcpin:0.0.0.0:8200
```

### Run the Gazebo

```bash
roslaunch iq_sim multi_drone.launch
```
### Run the launch file

```bash
roslaunch iq_sim multi-apm_1.launch 
```
```bash
roslaunch iq_sim multi-apm_2.launch 
```
```bash
roslaunch iq_sim multi-apm_3.launch 
```

### Run the Polygon Node

```bash
cd Polygon
python polygon.py
```
You can check if the node is publishing on /polygon topic by the following command

```bash
rostopic echo /polygon 
```

### Run the Simulation

```bash 
roslaunch iq_gnc multi_square_sol.launch 
```