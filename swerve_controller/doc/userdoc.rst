:github_url: https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/swerve_controller/doc/userdoc.rst

.. _swerve_controller_userdoc:

swerve_controller
=================

Controller for four-wheel independent steering (swerve) drive robots.
Each wheel module has a dedicated drive motor (velocity-controlled) and a steer motor (position-controlled), giving the robot full holonomic motion capability — it can translate in any direction and rotate simultaneously without any kinematic constraints.

The controller implements ``ChainableControllerInterface``, so it can be used standalone or as a downstream controller in a controller chain.

For an introduction to mobile robot kinematics and the nomenclature used here, see :ref:`mobile_robot_kinematics`.

Other features
--------------

   + Realtime-safe implementation.
   + Holonomic (omnidirectional) motion — simultaneous translation and rotation
   + Odometry publishing
   + Instantaneous Centre of Rotation (ICR) visualisation
   + Task-space velocity, acceleration and jerk limits
   + Automatic stop after command time-out
   + Chainable Controller
   + Forward/reverse wheel-flip optimisation to minimise steering travel


Robot Geometry
--------------

::

             wheelbase
        |<------------->|

     ___                 ___
    |FL |               |FR |
    |___|               |___|
      |    base_link      |    ---
      |       o           |     ^  wheel_track
      |                   |     v
     ___                 ___   ---
    |RL |               |RR |
    |___|               |___|

``wheelbase``
  Longitudinal distance between front and rear axle centres (m).

``wheel_track``
  Lateral distance between left and right wheel centres (m).

``drive_to_steer_offset``
  Lateral offset from the steer joint axis to the drive wheel contact point (m).
  When non-zero the effective steering track is ``wheel_track - 2 * drive_to_steer_offset``.

``wheel_radius``
  Radius of each drive wheel (m).


Inverse Kinematics
------------------

Given a desired body-frame velocity command :math:`(\dot{x},\ \dot{y},\ \dot{\theta})`, the controller computes an independent steering angle and drive speed for each wheel.

Let :math:`L` = ``wheelbase`` and :math:`T_s` = ``wheel_track - 2 * drive_to_steer_offset`` (effective steering track).

Per-wheel velocity vectors
,,,,,,,,,,,,,,,,,,,,,,,,,,

Each wheel's velocity is the superposition of the translational body velocity and the rotational contribution from :math:`\dot{\theta}`:

.. math::

   \begin{aligned}
   v_{FL,x} &= \dot{x} - \dot{\theta}\frac{T_s}{2}, \quad &v_{FL,y} &= \dot{y} + \dot{\theta}\frac{L}{2} \\
   v_{FR,x} &= \dot{x} + \dot{\theta}\frac{T_s}{2}, \quad &v_{FR,y} &= \dot{y} + \dot{\theta}\frac{L}{2} \\
   v_{RL,x} &= \dot{x} - \dot{\theta}\frac{T_s}{2}, \quad &v_{RL,y} &= \dot{y} - \dot{\theta}\frac{L}{2} \\
   v_{RR,x} &= \dot{x} + \dot{\theta}\frac{T_s}{2}, \quad &v_{RR,y} &= \dot{y} - \dot{\theta}\frac{L}{2}
   \end{aligned}

Drive speed and steering angle
,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

.. math::

   s_i = \sqrt{v_{i,x}^2 + v_{i,y}^2}, \qquad \theta_i = \operatorname{atan2}(v_{i,x},\ v_{i,y})

The controller also evaluates the reverse solution :math:`\theta_i + \pi` with negated speed and selects whichever requires less steering travel from the current joint position, minimising rotation to reach the target angle.

Speed normalisation
,,,,,,,,,,,,,,,,,,,

All four wheel speeds are scaled by a common factor so that the fastest wheel operates at exactly 1.0 m/s.
This preserves the velocity ratios between wheels while keeping commands within hardware limits.

.. math::

   k = \min\!\left(1,\ \frac{1}{\max_i(s_i)}\right), \qquad s_i^* = k\, s_i

Drive motor command
,,,,,,,,,,,,,,,,,,,

The normalised linear speed is converted to an angular velocity command for the drive motor:

.. math::

   \omega_i = \frac{s_i^*}{r}

where :math:`r` = ``wheel_radius``.

Steer assist
,,,,,,,,,,,,

When ``drive_to_steer_offset`` is non-zero, rotating the steer motor imparts a small longitudinal component to the wheel's contact velocity.
The controller compensates by blending the commanded and measured drive velocities:

.. math::

   \omega_i^{\text{corrected}} = \omega_i \pm \frac{\omega_i + \omega_i^{\text{measured}}}{2} \cdot d \cdot r

where :math:`d` = ``drive_to_steer_offset``.
The sign is negative for left-side wheels (FL, RL) and positive for right-side wheels (FR, RR).


Forward Kinematics (Odometry)
------------------------------

Given wheel velocities :math:`\Omega_i` (rad/s) and steering angles :math:`\theta_i` (rad), the controller estimates the body-frame velocity.

Each wheel's velocity is projected into body-frame X and Y:

.. math::

   v_{i,x} = \sin(\theta_i)\,\Omega_i\,r, \qquad v_{i,y} = \cos(\theta_i)\,\Omega_i\,r

Define the intermediate averages:

.. math::

   a = \frac{v_{RL,x} + v_{RR,x}}{2}, \quad
   b = \frac{v_{FR,x} + v_{FL,x}}{2}, \quad
   c = \frac{v_{FR,y} + v_{RL,y}}{2}, \quad
   d = \frac{v_{FL,y} + v_{RR,y}}{2}

Body angular and linear velocities are then estimated by averaging over the four wheels:

.. math::

   \hat{\dot{\theta}} = \frac{1}{2}\left(\frac{b - a}{L} + \frac{c - d}{T_s}\right)

.. math::

   \hat{\dot{x}} = \frac{1}{2}\left(\hat{\dot{\theta}}\frac{L}{2} + c\right)
                 + \frac{1}{2}\left(-\hat{\dot{\theta}}\frac{L}{2} + d\right)

.. math::

   \hat{\dot{y}} = \frac{1}{2}\left(\hat{\dot{\theta}}\frac{T_s}{2} + a\right)
                 + \frac{1}{2}\left(-\hat{\dot{\theta}}\frac{T_s}{2} + b\right)

Position is integrated each cycle:

.. math::

   x_{t+1} = x_t + \hat{\dot{x}}\,\Delta t, \quad
   y_{t+1} = y_t + \hat{\dot{y}}\,\Delta t, \quad
   \psi_{t+1} = \psi_t + \hat{\dot{\theta}}\,\Delta t

Three odometry modes are available via parameters:

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - ``open_loop``
     - ``position_feedback``
     - Description
   * - ``false``
     - ``false``
     - Closed-loop odometry from drive wheel **velocity** states
   * - ``false``
     - ``true``
     - Closed-loop odometry from drive wheel **position** states (differentiated)
   * - ``true``
     - —
     - Open-loop: integrates velocity commands directly, no hardware feedback


Instantaneous Centre of Rotation (ICR)
---------------------------------------

After computing the steer commands the controller calculates the Instantaneous Centre of Rotation for each pair of wheel axes using projective geometry (line intersection in homogeneous coordinates).
The ICR positions are published as a ``visualization_msgs/Marker`` on ``~/icrs`` for diagnostic visualisation in RViz.


Description of controller's interfaces
----------------------------------------

References
,,,,,,,,,,

When the controller is in chained mode it exposes the following references which can be commanded by the preceding controller:

- ``<controller_name>/linear_x/velocity``   double, in m/s
- ``<controller_name>/linear_y/velocity``   double, in m/s
- ``<controller_name>/angular/position``    double, in rad/s

Together these represent the body twist that in unchained mode is obtained from ``~/reference``.

Feedback
,,,,,,,,

As feedback interface type the drive joints' velocity (``hardware_interface::HW_IF_VELOCITY``) is used by default.
If the parameter ``position_feedback=true`` is set, drive joint position (``hardware_interface::HW_IF_POSITION``) is used instead (and differentiated to obtain velocity).
Steer joints always use position feedback (``hardware_interface::HW_IF_POSITION``).
If ``open_loop=true`` no external state interfaces are used; commanded velocity is integrated directly for odometry.

Output
,,,,,,

- Drive joints: ``hardware_interface::HW_IF_VELOCITY`` (rad/s)
- Steer joints: ``hardware_interface::HW_IF_POSITION`` (rad)


ROS 2 Interfaces
----------------

Subscribers
,,,,,,,,,,,

``~/reference`` [geometry_msgs/msg/TwistStamped]
  Velocity command. Used when ``use_stamped_vel=true`` (recommended).

``~/reference_unstamped`` [geometry_msgs/msg/Twist]
  Velocity command without timestamp. Used when ``use_stamped_vel=false``. Deprecated; prefer the stamped topic.

Publishers
,,,,,,,,,,

``~/odometry`` [nav_msgs/msg/Odometry]
  Estimated pose and velocity of the robot in the odometry frame.

``~/tf_odometry`` [tf2_msgs/msg/TFMessage]
  Odometry TF transform. Published only if ``enable_odom_tf=true``.

``~/icrs`` [visualization_msgs/msg/Marker]
  Instantaneous Centre of Rotation markers for RViz diagnostics.

``~/controller_state`` [ros2_controllers_interfaces/msg/SwerveControllerStatus]
  Per-wheel commanded and measured drive velocities and steer angles.

``~/cmd_vel_limitted`` [geometry_msgs/msg/TwistStamped]
  Rate-limited velocity command after applying speed/acceleration limits.
  Published only if ``publish_limited_velocity=true``.


Parameters
,,,,,,,,,,

This controller uses the `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ to handle its parameters.
The parameter `definition file located in the src folder <https://github.com/ros-controls/ros2_controllers/blob/{REPOS_FILE_BRANCH}/swerve_controller/src/swerve_controller_parameter.yaml>`_ contains descriptions for all the parameters used by the controller.

.. generate_parameter_library_details:: ../src/swerve_controller_parameter.yaml


Example Configuration
---------------------

.. code-block:: yaml

   controller_manager:
     ros__parameters:
       swerve_controller:
         type: swerve_controller/SwerveController

   swerve_controller:
     ros__parameters:
       wheel_radius: 0.1
       wheelbase: 0.5
       wheel_track: 0.4
       drive_to_steer_offset: 0.0

       drive_joints_names:
         - fl_drive_joint
         - fr_drive_joint
         - rl_drive_joint
         - rr_drive_joint

       steer_joints_names:
         - fl_steer_joint
         - fr_steer_joint
         - rl_steer_joint
         - rr_steer_joint

       drive_joints_state_names:
         - fl_drive_joint
         - fr_drive_joint
         - rl_drive_joint
         - rr_drive_joint

       steer_joints_state_names:
         - fl_steer_joint
         - fr_steer_joint
         - rl_steer_joint
         - rr_steer_joint

       use_stamped_vel: true
       reference_timeout: 1.0
       enable_odom_tf: true

       linear:
         x:
           has_velocity_limits: true
           max_velocity: 1.0
           has_acceleration_limits: true
           max_acceleration: 2.0
         y:
           has_velocity_limits: true
           max_velocity: 1.0
           has_acceleration_limits: true
           max_acceleration: 2.0
       angular:
         z:
           has_velocity_limits: true
           max_velocity: 1.5
           has_acceleration_limits: true
           max_acceleration: 3.0
