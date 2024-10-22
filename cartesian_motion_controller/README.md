# Cartesian Motion Controller

This controller is a flexible and versatile default for Cartesian motion without contact.
The primary use case is approaching and tracking moving targets in the robots workspace.
One of the advantages is that the controller interpolates for you, i.e. you do
not need to densely sample intermediate poses towards your targets.   

As an example, the `CartesianMotionController` can be given sparsely sampled target poses
with a relatively low frequency.
Depending on their use case, users can obtain anything in between
smooth, directed motion, or fast jumps to these target poses.
If the desired Cartesian trajectories are sampled more high-frequently, the
controller can achieve good tracking for more precise tasks.

The interpolation behavior can be tweaked by the following parameters:
* The `p` and `d` gains determine the responsiveness in each individual Cartesian axis. The higher these
  values, the faster does the robot *drive* to the given target pose.
  Proportional gains (`p`) are normally sufficient and you'll probably not need
  a `d` gain for most use cases.
* The `error_scale` achieves this behavior uniformly for all axes. It
  post-scales the computed error from the controller gains (by multiplication) and is a little
  easier to adjust with a single parameter. The idea is
  to first weigh the different Cartesian axes with the controller gains and adjust the
  *overall* responsiveness with this parameter.
* The number of internal `iterations` per robot control cycle. The higher this
  value, the more does the controller become an inverse kinematics solver for accurate tracking.

Algorithm:    
for each control update:    
$~~~~~$ for i in num_iter:    
$~~~~~~~~~~ X_{error} = X_{target} - X_{current} ~~~~~$  => This includes rot_vector error        
$~~~~~~~~~~ X_{pd} = kp X_{error} + kd * (X_{error} - X_{error\_prev})/(internal\_period)$     
$~~~~~~~~~~ X_{input} = error\_scale * X_{pd}$ => this is like spring force $F_{spring} = k * \Delta X$     
$~~~~~~~~~~ joint\_motion = IK(X_{input})~~~~~$ => the IK method here is not really inverse kinematics (i dont know why they call it that), it is **ForwardDynamicsSolver** which means 
they say the error in Cartesian is like force that is dragging the end-effector. We are not measuing any force 
here at all. $ joint\_motion $ is a JointTrajectoryPoint type holding positions, velocities and accelerations of each joint

**ForwardDynamicsSolver**     
$ \ddot{q} = H^{-1} ( J^T F_{ee}) $       
They are saying $F_{ee}$ is equal to $X_{input}$ multiplied by a scale
So basically, their cartesian motion control, to me, is like a admittance control but without measuring force, we 
calculate the force at ee by the Cartesian motion error.    

If we would use the $J^T F_{ee}$ as is in a control law like the following, then we can call it impedance control; 
straight to joint torque:   
$ \tau = gravity(q) + J^T F_{ee} $

## Getting Started
We assume that you have the `cartesian_controller_simulation` package installed.
1) Start the simulation environment as described [here](./../cartesian_controller_simulation/README.md).

2) Now we activate the `cartesian_motion_controller` and a
   [graphical handle](../cartesian_controller_handles/README.md) for drag-drop interaction:
   ```bash
   ros2 control switch_controllers --start cartesian_motion_controller motion_control_handle
   ```

   This activates an interactive marker for moving the robot's end-effector.
   Drag the marker by its handles. The robot should smoothly follow your lead.

3) Next, open `rqt` in a new, sourced terminal to change some of the controller's parameters:
   ```bash
   rqt
   ```
   Open the *Dynamic Reconfigure* plugin under *Plugins/Configuration* and
   select the *cartesian_motion_controller*. Play a little with the parameters
   (e.g. `solver/error_scale` and `solver/iterations`) and observe the changes
   in RViz  using the interactive marker as handle.

This concludes our small tutorial of using the `cartesian_motion_controller`.
The approach of steering the robot with the graphical handle is a good first step when working with real robot hardware.
It lets you check if everything runs as expected and allows you to adapt the parameters as desired.

When controlling your robot in special use cases, you would normally write
custom scripts that continuously publish the controller's target pose by
sampling from desired Cartesian trajectory profiles.


## Example Configuration
Below is an example `controller_manager.yaml` for a controller specific configuration. Also see [the simulation config](../cartesian_controller_simulation/config/controller_manager.yaml) for further information.
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    cartesian_motion_controller:
      type: cartesian_motion_controller/CartesianMotionController

    # More controller instances here
    # ...


cartesian_motion_controller:
  ros__parameters:
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6

    # Choose: position or velocity.
    command_interfaces:
      - position
        #- velocity

    solver:
        error_scale: 1.0
        iterations: 10

    pd_gains:
        trans_x: {p: 1.0}
        trans_y: {p: 1.0}
        trans_z: {p: 1.0}
        rot_x: {p: 0.5}
        rot_y: {p: 0.5}
        rot_z: {p: 0.5}


# More controller specifications here
# ...

```
