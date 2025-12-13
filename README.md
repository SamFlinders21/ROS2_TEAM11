You need 3 terminals.
In all three, run 
```bash
source install/setup.bash
```

Then, in any of the three, run
```bash
colcon build --package-select robot_controller
```

Terminal 1: (keep this one running b/c its what runs RViz with the correct settings)
```bash
ros2 launch robot_controller visualize.launch.py
```

Terminal 2: (This one just publishes the joint states for RViz and for the main loop)
```bash
ros2 run robot_controller joint
```
Terminal 3: (This one has the main loop.) The robots movement will start/stop when you run/exit the code
```bash
ros2 run robot_controller run_controller
```

To change the pick/place locations, go into controller.py and edit the angles defined with pick/place (i made a large line of ### to seperate this part from the rest of the code)

Save the changes, and dont forget to run
```bash
colcon build
```
before you run 
```bash
ros2 run robot_controller run_controller
```
