# ROS 2 Raspberry Pi GPIO Package
ROS 2 package to control Raspberry Pi GPIO pins

This packages allows you set and read your raspbeery Pi's GPIO pins via ROS 2 action server calls.

Tested with Raspberry Pi3, ROS2 Eloquent, and Ubuntu 18.04 Server

<img src="pi_gpio.png" alt="" width="600">

Link to the project video: 

# Demo:

  <img src="demo.gif" alt="" width="600">

# 1.Build Instructions
 ```
 source /opt/ros/<ros2_distro>/setup.bash
 mkdir <your_ros2_ws_name>
 cd your_ros2_ws_name
 mkdir src
 cd src
 git clone https://github.com/mlherd/ros2_pi_gpio.git
 cd ..
 colcon build --sysmlink-install
 source install/setup.bash
 ```
 
# 2.Usage
- ### 2.1 Setup Raspberry Pi
- ### 2.3 Edit gpio_pins.txt
  - <pin_id>,<input_output>
  - example:
    - 17,out
    - 18,in
- ### 2.4 Run pi_gpio_server
  - ```ros2 run pi_gpio pi_gpio_server```
- ### 2.5 Send goals to the action server
  - example via command line interface:
    - ros2 action send_goal <action_name> <action_message_type>
  - ```ros2 action send_goal pi_gpio_server pi_gpio_interface/action/GPIO {'gpio: "17,high"'}```
  - ```ros2 action send_goal pi_gpio_server pi_gpio_interface/action/GPIO {'gpio: "18,low"'}```
  - ```ros2 action send_goal pi_gpio_server pi_gpio_interface/action/GPIO {'gpio: "18,read"'}```

# 3. Nodes

- ### 3.1 /pi_gpio_server
  - ROS 2 action server node
  - ### 3.1.1 Actions
    - Action name: pi_gpio
    - Action type: pi_gpio_interface/action/GPIO
    - pi_gpio ([pi_gpio_interface/action/GPIO](https://github.com/mlherd/ros2_pi_gpio/blob/master/pi_gpio_interface/action/GPIO.action))
      - Goal
        - string gpio
      - Result
        - int32 value
      - Feedback
        - int32 feedback

