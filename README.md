# CompRobo Warmup Project

This repository contains my [warmup project][warmup-project] for Olin's Computational Introduction to Robotics class, taken as an independent study in Spring 2022.

This project introduces ROS and the associated toolset (Gazebo, RViz, TF, etc.), all of which were new to me. Our robot was [a Neato robotic vacuum cleaner][neato]. For practical reasons (the physical Neatos needed some maintenance after having not been used for the two years of the pandemic), almost all testing was done using [a simulated Neato in Gazebo][neato-sim].

The project requirements included five distinct Neato behaviors, each described more in-depth below: TeleOp, Driving in a Square, Wall Following, Person Following, and Obstacle Avoidance. I integrated a sixth, using a state machine to combine multiple behaviors, into the square behavior. All behaviors are documented here, and correspond to a distinct file in the [`scripts/`](scripts/) directory. Screen recordings of each behavior (excluding TeleOp) are included in this document, and `rosbag` recordings can be be found in the [`bags/`](bags/) directory.

My thanks to Paul Ruvolo for advising this independent study, and for his help throughout this project.

## TeleOp

The TeleOp behavior enables a human to control the Neato in real time using a computer keyboard. The script supports driving forwards/backwards, turning left/right, and stopping.

The code is fairly straightforward, repeatedly checking for input from stdin (using a slightly-modified version of a non-blocking input mechanism provided by the teaching team) and acting accordingly.

A significant area of possible future improvement for this behavior would be to support various forms of analog control. Currently, it doesn't support turning and driving straight at the same time, and can only drive at a fixed speed. Additionally, the input mechanism used doesn't support listening for key up events, so the script can't stop the robot when the user stops pressing (ex.) the forwards key. Solving these issues would be mostly an exercise in Python I/O techniques rather than robotics, hence it being out of scope for this project.

_There is no screen capture or rosbag recording of the TeleOp behavior, because neither would make much sense without being able to see key presses._

## Drive in a Square

![Video of neato driving in a square][square]

As the name implies, this behavior drives the Neato in a 1x1 meter square, repeatedly. It uses the Neato's odometry to drive precisely 1 meter and turn precisely 90 degrees.

This behavior is implemented as a finite state machine, with two states: moving forward and turning left. Each state is encapsulated as a subclass of `NodeState`, and managed by the `StateMachineNode` class (which also owns all rospy objects, such as subscribers or publishers).

I wrote this behavior before fully wrapping my head around TF, so the way it handles it handles odometry (both when driving a certain distance and when turning to a specific angle) is sub-optimal and non-idiomatic. This made implementing the behavior both difficult and frustrating--not to mention time consuming--since I got myself into an [XY problem][xy-problem]: I knew that ROS used quaternions to represent rotation, so I wanted to learn how to use them properly instead of simply converting to Euler angles. However, I didn't realize that idiomatic ROS code generally doesn't use quaternions directly, instead handling them through TF. As a result, there's very little documentation or tooling for using quaternions directly. I think that properly understanding how to use reference frames and TF (which was something of a lightbulb moment) was the single most important thing I learned from this project.

This behavior is currently implemented as follows:

1. When the robot switches states:
	1. Extract its position and heading in the `odom` (ie. global) reference from its `Pose`, as a point and quaternion, respectively.
	2. Set the target to the heading + 90 degrees if in the rotating state, or to position + 1 meter _in the direction that the robot is facing_ if in the moving forward state. Calculate that value using low-level quaternion multiplication.
2. Whenever new odometry data arrives, compare the robot's position and heading to the target. Set the angular (when in the rotating state) or linear (when in the moving forward state) velocity to a fixed value if the robot isn't within a specified tolerance of the target yet.
3. If the robot is within a specified tolerance of its target heading/position, transition to the next state.

Now that I understand (at least somewhat) TF and reference frames, I would instead implement this behavior as:

1. When the robot switches states, use TF to establish a `target` reference frame as a 90 degree/1 meter (as appropriate) transformation of the robot's current reference frame (`base_link`).
2. Whenever new odometry data arrives, use TF to determine the new difference between the `target` and updated `base_link` reference frames (ie. how far the robot is from its target, either in terms of rotation or translation).
3. Act depending on that difference: if the robot is sufficiently near its target, transition to the next state; otherwise, set its speed proportionally to the difference.

This alternate solution would be much better and more idiomatic ROS code, in addition to being easier, simpler, and less error prone.

## Wall Follower

![Video of Neato following a wall][wall-follower]

Under this behavior, the Neato uses its LIDAR sensor to detect a wall, then drive parallel to it. Wall detection is done using a simplified version of [Random Sample Consensus (RANSAC)][ransac]. Essentially, RANSAC uses a guess-and-check algorithm to find the line that represents the wall in the noisy LIDAR data, while ignoring noise/outliers.

Once the wall has been identified, the script proportionally controls the Neato's angular velocity to align it with the wall, while continually moving forward at a constant speed. This behavior was also written before I properly understood TF, although its use of quaternions is very minimal (it simply takes the ijk magnitude of the difference between the robot's heading and the wall's heading). Ideally, it should be refactored to use TF. Additionally, increasing the sophistication and robustness of the RANSAC algorithm would likely increase performance in real-world environments, which are have more noise than the simulator.

## Person Follower

![Video of Neato following a "person", simulated by a cylinder][person-follower]

In this behavior, the Neato follows around a "person" (simulated by a cylinder). More specifically, it follows the geometric mean of anything seen by the LIDAR sensor in the 2x2 meter square immediately in front of it. It moves towards that center using proportional control for both the angular and linear velocity of the robot.

This behavior was written with the benefit of an understanding of TF, and uses reference frames to its advantage. Specifically, it transforms the point cloud produced by the LIDAR scanner to be relative to the robot (the `base_link` frame). This makes it easy to filter out points not in the 2x2 meter square in front of the robot, and means that, when treated as a vector, the geometric mean of the remaining laser points is equal to the direction and distance that the robot needs to move.

This is actually the second version of this behavior. The first didn't use TF, and was subject to much of the same complexity as the previous two behaviors. It's still [available in Git history][old-person-follower], but is inferior to this version in every regard (including functionality and reliability).

## Obstacle Avoider

![Video of Neato avoiding obstacles][obstacle-avoider]

In this behavior, the Neato tries to move to a pre-programmed target location while avoiding obstacles. Obstacles are detected using the LIDAR scanner.

The Neato avoids obstacles using a potential-field based algorithm, inspired by [this tutorial][potential-fields] (and [the QEA 2 version of this same problem][qea2]). Each point detected by the LIDAR scanner counts as an obstacle, and emits a "repelling" force against the robot, which decreases with distance from the robot. The target is pre-programmed to have a large "attractive" force, which is also inversely-proportional to distance. By summing these forces, we get a net direction of travel for the robot.

This behavior makes heavy use of reference frames and TF to calculate the goal in the global reference frame (`odom`), then transform it into the current frame of the robot (`base_link`). LIDAR data is also translated into the robot's reference frame.

The equations and coefficients used to produce the attractive and repellant forces are imperfect heuristics, calculated through trial-and-error. Generating those forces in a more thoughtful way would likely result in a more efficient and reliable obstacle avoidance system. Additionally, having a more sophisticated algorithm for translating the net force vector to Neato movement--given that the Neato can only move in one direction, or rotate--would be valuable. Currently, it rotates until it's facing approximately the right direction, then drives forward while continuing to correct its heading.

![Neato's obstacle avoidance target][obstacle-target]

## Conclusion

This project was a fun, engaging way to gain familiarity with ROS, TF, and associated tools. In particular, coming to understand how to best use reference frames, transformations, and TF helped me to see robotics programming in a different light, and made this project drastically easier. Overall, I'm proud with how the project turned out, and look forward to the remainder of this independent study.

[warmup-project]: https://comprobo20.github.io/assignments/warmup_project
[neato]: https://comprobo20.github.io/How%20to/shopping_list
[neato-sim]: https://comprobo20.github.io/How%20to/run_the_neato_simulator

[xy-problem]: https://xyproblem.info
[old-person-follower]: https://github.com/ariporad/warmup_project/blob/a3eb92a11b5aaf8c93bcdf7432f6a37ea4f4d3f7/scripts/person_follower.py
[ransac]: https://en.wikipedia.org/wiki/Random_sample_consensus
[potential-fields]: https://phoenix.goucher.edu/~jillz/cs325_robotics/goodrich_potential_fields.pdf
[qea2]: https://github.com/ariporad/QEA/blob/main/Robo/gauntlet/report.pdf

[square]: https://user-images.githubusercontent.com/1817508/160192241-77a464f0-b10d-4c46-ae81-622f4b1eaae0.mov
[person-follower]: https://user-images.githubusercontent.com/1817508/160192347-a02f9a9e-7793-4760-ad47-fd27924d5c34.mov
[wall-follower]: https://user-images.githubusercontent.com/1817508/160192468-5fdffefd-ad3f-4eb7-8a0d-cd1665aa869f.mov
[obstacle-avoider]: https://user-images.githubusercontent.com/1817508/160192595-590895bc-a807-4a51-b52f-4d74bc17aeb3.mov
[obstacle-target]: recordings/obstacle_avoidance_target.jpg
