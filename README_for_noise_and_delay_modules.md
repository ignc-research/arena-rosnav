## 1.Goals
Adding noise and latency to laser scan sensors


## 2.Principle
We renamed the original "scan" topic to "scan_original" topic and added a new node "scan_process " 
to receive the information from "scan_original", and process the sensor information in the new node 
to increase the noise and delay. Finally,the processed sensor information is broadcast with the "scan" topic.  
So no additional files need to be changed to accommodate the noise and delay modules.



##3. Mode and start-up commands for noise and delay modules

###3.1 Mode for noise and delay modules

#### Mode for noise module
- The first noise mode is the gaussian noise.It is a noise that fits a Gaussian normal distribution.
- The second noise mode is the offset noise.It simulate by Random Walk errors.
- The third noise mode is the angle noise. It simulate angular errors, which are made up of mechanical horizontal and vertical errors.
- The fourth noise mode is bias noise.It simulate Physical offsets.

#### Mode for delay module
- Adding fixed time delays to sensor information.

###3.2 Parameters for noise module

```bash
    "max_value_of_data": Maximum value of sensor information
    "gauss_mean": Mean value of Gaussian noise
    "gauss_sigma": Standard deviation range of Gaussian noise
    "gauss_size": Size of gauss error
    "bias_noise": Size of bias error
    "offset_noise": Mean value of random error
    "angle_noise": Angular error in radians.(mrad)
```
The parameters can be modified in the noise_parameter.json file

###3.3 Start-up commands for noise and delay modules
- In one terminal, start simulation. You can specify the following parameters: 

   * noise_mode:=<0,1,2,3,4>(default 0,means without noise) #Different noises can be selected individually or together.eg.noise_mode:=123 It means that noise mode123 is added at the same time.
   * delay:=<int> (default 0) # Delay time

```bash
roslaunch arena_bringup start_arena_flatland.launch train_mode:=false use_viz:=true local_planner:=mpc map_file:=map1 obs_vel:=0.3 noise_mode := 1 delay:= 10
```

## 4.Location of function and how to merge.
- The functions are implemented in the realistic_modeling/scan_process folder.  
- Changed parameters and started newly added sublaunch in start_arena_flatland.launch.  
- Started new nodes and topics in scan_process.launch.
- Rename the original topic name in flatland_simulator.launch.

The noise and delay modules can be added to any version of the program by simply making changes to the above sections.
