<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS Navigation Stack

<details>
<summary>Table of Contents</summary>
<ol>
<li>
<a href="#introduction">Introduction</a>
</li>
<li>
<a href="#getting-started">Getting Started</a>
<ul>
<li><a href="#prerequisites">Prerequisites</a></li>
<li><a href="#installation">Installation</a></li>
</ul>
</li>
<li><a href="#launch-and-usage">Launch and Usage</a></li>
<ul>
<li><a href="#map-generation">Map Generation</a></li>
<li><a href="#registering-locations">Registering Locations</a></li>
<li><a href="#running-navigation">Running Navigation</a></li>
</ul>
</li>
<li><a href="#milestones">Milestones</a></li>
<li><a href="#references">References</a></li>
</ol>
</details>



## Introduction

This is an autonomous navigation package for SOBIT PRO, SOBIT EDU, SOBIT MINI, SOBIT LIGHT, and HSR (Real&Simulation).
For an overview of open-source navigation, check [here](https://docs.nav2.org/).
For the mechanism of autonomous navigation, refer to the [ROS open-source project](https://github.com/ros-navigation/navigation2).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

| System | Version |
| :------------ | :------------ |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS | Humble Hawksbill |
| Python | 3.0~ |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

1.  Navigate to your ROS2 `src` folder.
    ```bash
    cd　~/colcon_ws/src/
    ```
2.  Clone this repository.
    ```bash
    git clone -b humble-devel https://github.com/TeamSOBITS/sobits_nav_stack.git
    ```
3.  Move into the repository directory.
    ```bash
    cd sobits_navigation_stack
    ```
4.  Install dependent packages.
    ```bash
    bash install.sh
    ```
5.  Compile the package.
    ```bash
    cd ~/colcon_ws/
    ```
    ```bash
    colcon build --symlink-install
    ```
    ```bash
    source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Launch and Usage

Basic workflow for using Navigation:

1.  **Map Generation**
      * The robot needs to know the map in advance to generate paths that avoid obstacles to the destination.
      * The robot estimates its current location from obstacle data in the map and current data acquired by the robot.
2.  **Registering Locations**
      * Register key points on the generated map to define the start and end points for path generation.
3.  **Calling via Action Communication**
      * Generates a path from the robot's current location to a registered location within a safe, obstacle-free area on the map.
      * Since reaching the destination can take time, Action communication is used to send intermediate progress as well as the final result.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Map Generation

1.  **Start the robot**
    Start the robot body and the 2D-LiDAR. For details, check the GitHub repositories for each robot ([SOBIT PRO](https://github.com/TeamSOBITS/sobit_pro.git), [SOBIT EDU](https://github.com/TeamSOBITS/sobit_edu.git), [SOBIT MINI](https://github.com/TeamSOBITS/sobit_mini.git), [SOBIT LIGHT](https://github.com/TeamSOBITS/sobit_light.git)). For HSR (Real&simulator), start sigverse and the HSR's built-in sensor data.\
    However, since SOBIT LIGHT requires removing maps from the Kachaka API, please refer to the [SOBIT LIGHT README](https://github.com/TeamSOBITS/sobit_light.git).
2.  **Generate a map**
      * **To generate a map manually:**
        1.  Switch the `robot_name` in [slam.launch.py](/sobits_slam/launch/slam.launch.py) to the robot you are using, then execute the following command. You will be asked if you want to save the map after execution, but ignore it for now.
            ```sh
            ros2 launch sobits_slam slam.launch.py
            ```
        2.  Next, switch the `velocity_topic_name` in [teleop.launch.py](/sobits_slam/launch/teleop.launch.py) to the topic name of the robot you are using, then execute the following command. Refer to the instructions in the launched xterm terminal (blue terminal) to operate the robot while viewing the map in Rviz.
            ```sh
            ros2 launch sobits_slam teleop.launch.py
            ```
      * **To use autonomous map generation:**
        Switch the `robot_name` in [active_slam.launch.py](/sobits_slam/launch/active_slam.launch.py) to the robot you are using, then execute the following command.

        - To utilize the camera on the robot's head for autonomous map generation while it's moving its neck, change ``use_flex_nav`` to ``True``.

        ```sh
        ros2 launch sobits_slam active_slam.launch.py
        ```
3.  **Save the generated map.**
    **Once map generation is complete, save the map.**
4.  If you have created a new map file, run `colcon build`. If you replaced an existing map file, you do not need to run `colcon build`.
    ```sh
    cd　~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```
    ```bash
    source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Registering Locations

1.  **Specify the path to the generated map.**
    Modify the `map` parameter in [create_location_file.launch.py](/sobits_slam/launch/create_location_file.launch.py). The `map` should point to the map you generated. For example, if your map is named `map_example.pgm`, specify it as follows:
    ```sh
    DeclareLaunchArgument(
            # Map file path
            'map', default_value=os.path.join(get_package_share_directory("sobits_slam"), 'map', 'map_example.yaml')
        ),
    ```
    *Note: The extension should be `.yaml`. Do not directly specify the image file; specify the YAML data file of the map.*
2.  **Set whether to register locations with the actual robot.**
      * **If you are NOT registering locations with the actual robot:**
        Set `use_robot` to `false` in [create_location_file.launch.py](/sobits_slam/launch/create_location_file.launch.py).
        ```sh
        'use_robot', default_value='false'
        ```
      * **If you ARE registering locations with the actual robot:**
        First, set `use_robot` to `true` and change `robot_name` to the robot you are using in [create_location_file.launch.py](/sobits_slam/launch/create_location_file.launch.py).
        ```sh
        'use_robot', default_value='true'
        ```
3.  **If registering locations with the actual robot, start the robot.**
    Start the robot body and the 2D-LiDAR. For details, check the GitHub repositories for each robot ([SOBIT PRO](https://github.com/TeamSOBITS/sobit_pro.git), [SOBIT EDU](https://github.com/TeamSOBITS/sobit_edu.git), [SOBIT MINI](https://github.com/TeamSOBITS/sobit_mini.git), [SOBIT LIGHT](https://github.com/TeamSOBITS/sobit_light.git)). For HSR (Real&simulator), start sigverse and the HSR's built-in sensor data.\
    However, since SOBIT LIGHT requires removing maps from the Kachaka API, please refer to the [SOBIT LIGHT README](https://github.com/TeamSOBITS/sobit_light.git).
4.  **Start location registration.**
    Launch with the following command:
    ```sh
    ros2 launch sobits_slam create_location_file.launch.py
    ```
    After launching, **save the location registration file before starting to register locations.**
5.  **Register locations.**
      * **If you are NOT registering locations with the actual robot:**
        In RVIZ, select **2D Goal Pose** and click on the map at the desired position and orientation to register.
      * **If you ARE registering locations with the actual robot:**
        Move the robot to the desired location for registration. There are two ways to move the robot:
          * **Using Navigation features:**
            In RVIZ, select **2D Goal Pose** and click on the map.
          * **Manually operating the robot as when generating the map:**
            Execute the following command:
            ```sh
            ros2 launch sobits_slam teleop.launch.py
            ```
      * **ADD LOCATION:** Enter a location name and register it.
      * **Delete:** Delete a registered location.
      * **Rename:** Change the name of a registered location.
6.  Once all location registrations are complete, terminate all launched processes. If you have created a new location registration file, run `colcon build`.
    ```sh
    cd　~/colcon_ws/
    ```
    ```sh
    colcon build --symlink-install
    ```
    ```bash
    source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Setting Up Keep-out Zones
You can set up areas on the map where you do not want the robot to enter (keep-out zones).

1.  **Creating the Keep-out Zone Map**
    1.  Open the map image (`.pgm` file) created during map generation with an image editor (e.g., GIMP).
    2.  Fill the areas you want to designate as keep-out zones with black (color code: `#000000`). Leave other areas as white (`#FFFFFF`) or gray (`#CDCDCD`).
    3.  Save the edited image with a new name (e.g., `map_example_keepout_mask.pgm`).
    4.  Copy the original map's `.yaml` file and rename it (e.g., `map_example_keepout_mask.yaml`).
    5.  Open the copied `.yaml` file and change the `image` value to the new image file name (`map_example_keepout_mask.pgm`).

2.  **Launching Navigation**
    When launching `nav2.launch.py`, set the `use_keepout_filter` argument to `True`.
    ```sh
    ros2 launch sobits_nav nav2.launch.py use_keepout_filter:=True
    ```
    This will load `sobits_slam/map/map_example_keepout_mask.yaml` as the keep-out zone map.
    > [!NOTE]
    > If you want to use a different keep-out zone map, please edit the `keepout_mask_yaml_file` path in nav2.launch.py.

Now, when navigation is running, the black-filled areas will be recognized as high-cost obstacles, and paths will be generated to avoid these zones.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Running Navigation

1.  **Change the map to the one you generated.**
    Register the map with Navigation. Modify the `map` parameter in [nav2.launch.py](/sobits_nav/launch/nav2.launch.py) to the filename of the map you created.

    Example: If the created map file is `map_example.yaml`

    ```sh
    default_value=os.path.join(get_package_share_directory('sobits_slam'), 'map', 'map_example.yaml'),
    ```

    *Here, you are specifying the map data. Do not confuse it with the location registration file.*

2.  **Register the location information.**
    Change the `location_file_path` in [nav2.launch.py](/sobits_nav/launch/nav2.launch.py) to the location registration file you created.

    Example: If the created location registration file is `location_example.yaml`

    ```sh
    default_value=os.path.join(
            get_package_share_directory('sobits_slam'), 'location', 'location_example.yaml'),
    ```

3.  Change the `robot_name` in [nav2.launch.py](/sobits_nav/launch/nav2.launch.py) to the robot you are using.

    - To utilize the camera on the robot's head to navigate while it's moving its neck, change ``use_flex_nav`` to ``True``.

4.  **Start the robot.**
    Start the robot body and the 2D-LiDAR. For details, check the GitHub repositories for each robot ([SOBIT PRO](https://github.com/TeamSOBITS/sobit_pro.git), [SOBIT EDU](https://github.com/TeamSOBITS/sobit_edu.git), [SOBIT MINI](https://github.com/TeamSOBITS/sobit_mini.git), [SOBIT LIGHT](https://github.com/TeamSOBITS/sobit_light.git)). For HSR (Real&simulator), start sigverse and the HSR's built-in sensor data.\
    However, since SOBIT LIGHT requires removing maps from the Kachaka API, please refer to the [SOBIT LIGHT README](https://github.com/TeamSOBITS/sobit_light.git).

5.  **Start Navigation.**
    Launch Navigation with the following command:

    ```sh
    ros2 launch sobits_nav nav2.launch.py
    ```

    This should display the map and the TFs of the registered locations on top of it.

6.  **Start the action client.**
    This is typically launched from a program. It is possible to move to any registered location by name.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Milestones

  - [ ] Map generation using cameras
  - [ ] Customization of obstacle layers
      - [ ] Bumper layer
      - [ ] Noise color layer
      - [ ] Objects layer

Please check the [Issues page](issues-url) for current bugs and feature requests.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## References

  * [ROS Navigation Stack Software Design Specification](https://robo-marc.github.io/navigation_documents/)
  * [explore_lite](http://wiki.ros.org/explore_lite)
  * [Flex Nav](https://github.com/TeamSOBITS/flex_nav)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_navigation_stack/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_navigation_stack/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_navigation_stack/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_navigation_stack/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_navigation_stack.svg?style=for-the-badge
[license-url]: LICENSE