# RoboND-HSR-Project


### 1. Clone the project & submodule

```
~ $ git clone --recurse-submodules git@github.com:JasonYCHuang/RoboND-HSR-Project.git
```


### 2. catkin_make & source

```
~ $ cd RoboND-HSR-Project
~/RoboND-HSR-Project $ catkin_make
~/RoboND-HSR-Project $ source devel/setup.bash
```


### 3. Execute scripts

```
~/RoboND-HSR-Project $ chmod +x ./src/ShellScripts/home_service.sh

~/RoboND-HSR-Project $ ./src/ShellScripts/home_service.sh
```

### 4. There are several scripts in `./src/ShellScripts` folder.

```
~/RoboND-HSR-Project $ ./src/ShellScripts/add_markers.sh
~/RoboND-HSR-Project $ ./src/ShellScripts/pick_objects.sh
~/RoboND-HSR-Project $ ./src/ShellScripts/test_slam.sh
~/RoboND-HSR-Project $ ./src/ShellScripts/test_navigation.sh
~/RoboND-HSR-Project $ ./src/ShellScripts/wall_follower.sh
```
