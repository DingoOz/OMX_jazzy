Installation
============

This guide walks you through setting up the Open Manipulator X workspace on Ubuntu 24.04 with ROS2 Jazzy.

Prerequisites
-------------

* Ubuntu 24.04 LTS
* ROS2 Jazzy Jalisco
* Git

Hardware (optional):

* ROBOTIS Open Manipulator X
* U2D2 USB-to-Dynamixel adapter
* Logitech C920 webcam

Step 1: Install ROS2 Jazzy
--------------------------

Follow the `official ROS2 Jazzy installation guide <https://docs.ros.org/en/jazzy/Installation.html>`_.

Quick install:

.. code-block:: bash

   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-jazzy-desktop

Step 2: Install Dependencies
----------------------------

Install all required ROS2 packages:

.. code-block:: bash

   sudo apt update
   sudo apt install -y \
     ros-jazzy-moveit \
     ros-jazzy-moveit-configs-utils \
     ros-jazzy-ros2-control \
     ros-jazzy-ros2-controllers \
     ros-jazzy-joint-state-publisher-gui \
     ros-jazzy-xacro \
     ros-jazzy-warehouse-ros-sqlite \
     ros-jazzy-gz-ros2-control \
     ros-jazzy-ros-gz \
     ros-jazzy-usb-cam \
     ros-jazzy-image-transport \
     ros-jazzy-compressed-image-transport \
     v4l-utils

Step 3: Build the Workspace
---------------------------

.. code-block:: bash

   cd ~/Programming/OMX_jazzy
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --allow-overriding dynamixel_sdk
   source install/setup.bash

Step 4: Hardware Permissions
----------------------------

U2D2 (Robot Arm)
^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Add user to dialout group
   sudo usermod -aG dialout $USER

   # Install udev rules
   sudo cp 99-u2d2.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger

   # Log out and back in for changes to take effect

C920 Camera (Optional)
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo cp 99-c920.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   sudo udevadm trigger

Step 5: Shell Configuration (Optional)
--------------------------------------

Add workspace sourcing to your shell:

.. code-block:: bash

   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   echo "source ~/Programming/OMX_jazzy/install/setup.bash" >> ~/.bashrc

Verifying Installation
----------------------

Check that packages are installed:

.. code-block:: bash

   source install/setup.bash
   ros2 pkg list | grep omx_control

You should see ``omx_control`` in the output.

Test with simulation (no hardware required):

.. code-block:: bash

   ros2 launch omx_control simulation.launch.py

This should launch Gazebo with the robot model and RViz for visualization.
