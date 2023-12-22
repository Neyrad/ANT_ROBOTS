# ROSS - Warehouse Simulation

This model simulates robots delivering packages in a warehouse. Every robot in the simulation has a rechargable battery on board, and its capacity is decreasing over time, with each recharge cycle by a constant `CAPACITY_CHUNK`. Correspondance between boxes and containers is specified in a `log.csv` file, which can be randomly generated using `gen_log.c`. The initial placement of the robots is specified in the `robots.csv` file; the geometry of the warehouse is speciefied in the `field.csv`. Both files can be found in the `copy_to_build` directory and should be copied to the `build` directory of the project. Every step of the simulation is saved to a seperate file in a `Simulation_History` directory in the `.csv` format. `field.csv` should also be copied to the `Simulation_History` directory if modified. There is also a `_N_STEPS.txt` file in the `Simulation_History` directory, which specifies the number of steps in the simulation. The paths to the `Simulation_History` directory and the `.csv` files can be specified in `main.c`.

# Usage
Make sure that `robots.csv`, `field.csv` and `gen_log.c` are copied to the `build` directory of the project.
Also make sure that the following pathes specified in `main.c` are correct:
``` shell
const char* path_to_log_folder  = "/mnt/c/Dev/Base/pygame/Simulation_History";
const char* path_to_room_file   = "field.csv";
const char* path_to_robots_file = "robots.csv";
const char* path_to_pairs       = "log.csv";
```

To build and start the simulation do the following:

``` shell
gcc -o gen_log gen_log.c
./gen_log
make
./start
```
Afterwards execute main.py to see animations.

To change duration of the simulation, you can use the `--end=` flag followed by a number (default is 100 000)
Example:
``` shell
./start --end=1000000
```
Parameters of the simulation are defined in `model.h`
For instance, you can change battery capacity from 500
``` shell
#define BATTERY_CAPACITY		  500
```
to 1000:
``` shell
#define BATTERY_CAPACITY		 1000
```

# Installation

This model can be built by ROSS by sym-linking it into the ROSS/models directory and building with `-DROSS_BUILD_MODELS=ON`

``` shell
git clone https://github.com/ROSS-org/ROSS
git clone https://gitlab.car.cos.ru/sim/ross-study-model/
cd ROSS/models
ln -s ../../storage-model ./
cd ../
mkdir build
cd build
cmake ../ -DROSS_BUILD_MODELS=ON
make
```
