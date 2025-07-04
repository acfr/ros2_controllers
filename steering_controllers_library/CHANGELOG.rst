^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package steering_controllers_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.41.0 (2025-01-13)
-------------------
* Update generate_parameter_library dependency in steering_controllers_library (backport `#1465 <https://github.com/ros-controls/ros2_controllers/issues/1465>`_) (`#1468 <https://github.com/ros-controls/ros2_controllers/issues/1468>`_)
* Fix typos in steering_controllers_lib (backport `#1464 <https://github.com/ros-controls/ros2_controllers/issues/1464>`_) (`#1466 <https://github.com/ros-controls/ros2_controllers/issues/1466>`_)
* Contributors: mergify[bot]

2.40.0 (2025-01-01)
-------------------
* fix(timeout): do not reset steer wheels to 0. on timeout (backport `#1289 <https://github.com/ros-controls/ros2_controllers/issues/1289>`_) (`#1452 <https://github.com/ros-controls/ros2_controllers/issues/1452>`_)
* steering_controllers_library: Add `reduce_wheel_speed_until_steering_reached` parameter (backport `#1314 <https://github.com/ros-controls/ros2_controllers/issues/1314>`_) (`#1429 <https://github.com/ros-controls/ros2_controllers/issues/1429>`_)
* Use the .hpp headers from `realtime_tools` package (backport `#1406 <https://github.com/ros-controls/ros2_controllers/issues/1406>`_) (`#1427 <https://github.com/ros-controls/ros2_controllers/issues/1427>`_)
* [CI] Add clang job and setup concurrency (backport `#1407 <https://github.com/ros-controls/ros2_controllers/issues/1407>`_) (`#1418 <https://github.com/ros-controls/ros2_controllers/issues/1418>`_)
* Contributors: mergify[bot]

2.39.0 (2024-12-03)
-------------------
* Update maintainers and add url tags (`#1363 <https://github.com/ros-controls/ros2_controllers/issues/1363>`_) (`#1364 <https://github.com/ros-controls/ros2_controllers/issues/1364>`_)
* Contributors: mergify[bot]

2.38.0 (2024-11-09)
-------------------
* Added -Wconversion flag and fix warnings (`#667 <https://github.com/ros-controls/ros2_controllers/issues/667>`_) (`#1321 <https://github.com/ros-controls/ros2_controllers/issues/1321>`_)
* fix(steering-odometry): convert twist to steering angle (`#1288 <https://github.com/ros-controls/ros2_controllers/issues/1288>`_) (`#1295 <https://github.com/ros-controls/ros2_controllers/issues/1295>`_)
* fix(steering-odometry): handle infinite turning radius properly (`#1285 <https://github.com/ros-controls/ros2_controllers/issues/1285>`_) (`#1286 <https://github.com/ros-controls/ros2_controllers/issues/1286>`_)
* Contributors: mergify[bot]

2.37.3 (2024-09-11)
-------------------

2.37.2 (2024-08-22)
-------------------

2.37.1 (2024-08-14)
-------------------

2.37.0 (2024-07-24)
-------------------
* Fix WaitSet issue in tests  (backport `#1206 <https://github.com/ros-controls/ros2_controllers/issues/1206>`_) (`#1211 <https://github.com/ros-controls/ros2_controllers/issues/1211>`_)
* Fix steering controllers library kinematics (`#1150 <https://github.com/ros-controls/ros2_controllers/issues/1150>`_) (`#1194 <https://github.com/ros-controls/ros2_controllers/issues/1194>`_)
* Contributors: mergify[bot]

2.36.0 (2024-07-09)
-------------------
* [Steering controllers library] Reference interfaces are body twist (`#1168 <https://github.com/ros-controls/ros2_controllers/issues/1168>`_) (`#1173 <https://github.com/ros-controls/ros2_controllers/issues/1173>`_)
* [STEERING] Add missing `tan` call for ackermann (`#1117 <https://github.com/ros-controls/ros2_controllers/issues/1117>`_) (`#1176 <https://github.com/ros-controls/ros2_controllers/issues/1176>`_)
* Fix steering controllers library code documentation and naming (`#1149 <https://github.com/ros-controls/ros2_controllers/issues/1149>`_) (`#1164 <https://github.com/ros-controls/ros2_controllers/issues/1164>`_)
* Add mobile robot kinematics 101 and improve steering library docs (`#954 <https://github.com/ros-controls/ros2_controllers/issues/954>`_) (`#1160 <https://github.com/ros-controls/ros2_controllers/issues/1160>`_)
* Fix correct usage of angular velocity in update_odometry() function (`#1118 <https://github.com/ros-controls/ros2_controllers/issues/1118>`_) (`#1153 <https://github.com/ros-controls/ros2_controllers/issues/1153>`_)
* Contributors: mergify[bot]

2.35.0 (2024-05-22)
-------------------

2.34.0 (2024-04-01)
-------------------

2.33.0 (2024-02-12)
-------------------
* Fix usage of M_PI on Windows (`#1036 <https://github.com/ros-controls/ros2_controllers/issues/1036>`_) (`#1037 <https://github.com/ros-controls/ros2_controllers/issues/1037>`_)
* Add tests for `interface_configuration_type` consistently (`#899 <https://github.com/ros-controls/ros2_controllers/issues/899>`_) (`#1011 <https://github.com/ros-controls/ros2_controllers/issues/1011>`_)
* Contributors: mergify[bot]

2.32.0 (2024-01-20)
-------------------
* Update ci-ros-lint.yml and copyright format (backport `#720 <https://github.com/ros-controls/ros2_controllers/issues/720>`_) (`#918 <https://github.com/ros-controls/ros2_controllers/issues/918>`_)
* Contributors: mergify[bot]

2.31.0 (2024-01-11)
-------------------
* Fix ackermann steering odometry (`#921 <https://github.com/ros-controls/ros2_controllers/issues/921>`_) (`#955 <https://github.com/ros-controls/ros2_controllers/issues/955>`_)
* Contributors: mergify[bot]

2.30.0 (2023-12-20)
-------------------
* Changing default int values to double in steering controller's yaml file (`#927 <https://github.com/ros-controls/ros2_controllers/issues/927>`_) (`#928 <https://github.com/ros-controls/ros2_controllers/issues/928>`_)
* Contributors: mergify[bot]

2.29.0 (2023-12-05)
-------------------

2.28.0 (2023-11-30)
-------------------

2.27.0 (2023-11-14)
-------------------
* Steering controllers library: fix open loop mode (`#793 <https://github.com/ros-controls/ros2_controllers/issues/793>`_) (`#800 <https://github.com/ros-controls/ros2_controllers/issues/800>`_)
* Improve docs (`#785 <https://github.com/ros-controls/ros2_controllers/issues/785>`_) (`#786 <https://github.com/ros-controls/ros2_controllers/issues/786>`_)
* Contributors: mergify[bot]

2.26.0 (2023-10-03)
-------------------

2.25.0 (2023-09-15)
-------------------

2.24.0 (2023-08-07)
-------------------

2.23.0 (2023-06-23)
-------------------

2.22.0 (2023-06-14)
-------------------
* Bump versions for release
* Let sphinx add parameter description to documentation (backport `#651 <https://github.com/ros-controls/ros2_controllers/issues/651>`_) (`#663 <https://github.com/ros-controls/ros2_controllers/issues/663>`_)
* Second round of dependencies fix (`#655 <https://github.com/ros-controls/ros2_controllers/issues/655>`_) (`#656 <https://github.com/ros-controls/ros2_controllers/issues/656>`_)
* Fix sphinx for steering odometry library/controllers (`#626 <https://github.com/ros-controls/ros2_controllers/issues/626>`_) (`#661 <https://github.com/ros-controls/ros2_controllers/issues/661>`_)
* Remove unnecessary include (backport `#645 <https://github.com/ros-controls/ros2_controllers/issues/645>`_) (`#646 <https://github.com/ros-controls/ros2_controllers/issues/646>`_)
* Steering odometry library and controllers (backport `#484 <https://github.com/ros-controls/ros2_controllers/issues/484>`_) (`#624 <https://github.com/ros-controls/ros2_controllers/issues/624>`_)
* Contributors: Bence Magyar, Christoph Fröhlich, Tomislav Petković, Reza Kermani, Denis Štogl

2.21.0 (2023-05-28)
-------------------

2.20.0 (2023-05-14)
-------------------

2.19.0 (2023-05-02)
-------------------

2.18.0 (2023-04-29)
-------------------

2.17.3 (2023-04-14)
-------------------

2.17.2 (2023-03-07)
-------------------

2.17.1 (2023-02-20)
-------------------

2.17.0 (2023-02-13)
-------------------

2.16.1 (2023-01-31)
-------------------

2.16.0 (2023-01-19)
-------------------

2.15.0 (2022-12-06)
-------------------

2.14.0 (2022-11-18)
-------------------

2.13.0 (2022-10-05)
-------------------

2.12.0 (2022-09-01)
-------------------

2.11.0 (2022-08-04)
-------------------

2.10.0 (2022-08-01)
-------------------

2.9.0 (2022-07-14)
------------------

2.8.0 (2022-07-09)
------------------

2.7.0 (2022-07-03)
------------------

2.6.0 (2022-06-18)
------------------

2.5.0 (2022-05-13)
------------------

2.4.0 (2022-04-29)
------------------

2.3.0 (2022-04-21)
------------------

2.2.0 (2022-03-25)
------------------

2.1.0 (2022-02-23)
------------------

2.0.1 (2022-02-01)
------------------

2.0.0 (2022-01-28)
------------------

1.3.0 (2022-01-11)
------------------

1.2.0 (2021-12-29)
------------------

1.1.0 (2021-10-25)
------------------

1.0.0 (2021-09-29)
------------------

0.5.0 (2021-08-30)
------------------

0.4.1 (2021-07-08)
------------------

0.4.0 (2021-06-28)
------------------

0.3.1 (2021-05-23)
------------------

0.3.0 (2021-05-21)
------------------

0.2.1 (2021-05-03)
------------------

0.2.0 (2021-02-06)
------------------

0.1.2 (2021-01-07)
------------------

0.1.1 (2021-01-06)
------------------

0.1.0 (2020-12-23)
------------------
