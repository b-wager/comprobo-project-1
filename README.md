# RoboBehaviors and Finite State Machines
### CompRobo Project 1
Brooke Wager and Kelsey McClung

This project explores the implementation of different robot behaviors within a finite state machine using Python, Ros2, and the Olin College Neato fleet. We have implemented four behaviors within a finite state machine: Teleoperation, Draw a Figure-Eight, Dizzy Collision Avoidance, and Wall Following.

## The Behaviors

### Teleoperation:
Teleoperations, or teleops, allow us to control the Neato’s movement from a computer keyboard. We have implemented two sets of teleops controls, which both run at the same time. The space bar stops the program when the current state is teleops. Our two sets of controls are:

**wasd:** This teleops system uses the keys w,a,s,d and is based on the Neato heading at the start of the Teleops state, where the Neato’s heading at the start is North. Each key represents a cardinal direction of the Neato’s initial position. When a direction is indicated by a key press, the Neato turns in place to the associated heading, then proceeds in a straight line at a constant velocity. 

**ijkl:** This teleops system uses the keys i,j,k,l and acts more typically. There are keys for forwards and backwards movement in a straight line, and keys for rotating left and right.

Our teleops system uses the following controls:

	w - turn north, then continue forward
	a - turn west, then continue forward
	s - turn east, then continue forward
	d - turn south, then continue forward
	i - move forward at current heading
	j - rotate left
	k - move backwards at current heading
	l - rotate right
	space - stop moving
	ctrl-c - stop moving and exit program

#### Design Decisions:
We initially implemented the wasd controls to have a unique set of controls that were slightly different from most wasd control systems. Our wasd method is unique but is also inexact and frustrating to use, as you can only drive in a straight line at 4 angles. We added more controls with ijkl to allow the user to turn the Neato to any heading and then continue in a straight line from there.

#### Code Structure
Using Victoria’s starting code that allows python to interpret keyboard input, we set up the teleops using if, elif, statements to determine different velocity and angular velocity values based on the user’s input. This new information went into a run_loop function that told the Neato how to move.


### Draw a Figure-Eight:
The Neato moves in a figure-eight shape, ending at its start location and heading. Programming a robot to move in a specific path based on its initial heading can be useful in cases when the robot’s physical space is known and controlled, and the robot must complete a task involving an exact movement. Ending in the same pose allows the robot to continue repeating the task. We choose a figure-eight shape for challenge and cuteness. 
![](https://github.com/b-wager/comprobo-project-1/blob/main/images/figure_eight_diagram.png)


#### Design Decisions:
For the loop ends of the figure-eight, the Neato drives in a semi-circle with a constant angular velocity of 0.3 or -0.3 depending on which side of the figure-eight it’s currently drawing. When the semi circle is completed, the neato rotates to the correct heading and proceeds in a slight curve to the starting point of the next semi circle.

#### Code Structure
The majority of the figure-eight code is run within the run_loop function, using while loops and time.sleep() to determine when it is time to switch from a semicircle to a slight curve and vice versa.


### Dizzy Collision Avoidance:
To avoid the Neato continuously driving into a wall or other obstacles, we implemented collision avoidance. When the Neato’s bump sensors are activated, it will back up in a loose squiggly line, then turn around to face the opposite direction of its collision. 

#### Design Decisions:
The Neato’s bump sensors are used to detect the collision. Next, the Neato reverses with an angular velocity switching between positive and negative values to create the squiggly line backwards. We made this choice to add a charming, dizzy appearance to the robot. 

#### Code Structure
The DizzyReaction node is very similar to the FigureEight node in that most of the logic is in the run_loop function. It also uses while loops and time.sleep() to make the desired shape.


### Wall Following:

Our Neato can approach a wall, turn itself to drive parallel to the wall, and follow the wall to its end using lidar scan data. This behavior is useful to robots that are autonomously navigating an unknown space, for example, a Neato robot in normal use navigating a home’s floorplan for the first time.

A video of our Neato’s wall following behavior can be found here: 
https://youtu.be/6rgcGucc17I?si=xKCYfT8hX44F69ze

![](https://github.com/b-wager/comprobo-project-1/blob/main/images/wall_following_diagram.png)


#### Design Decisions
The Neato approaches the wall using code adapted from the in class “wall_approach.py” sample code. If the object directly in front of the Neato is within target distance, the Neato will turn to come alongside the wall. Whether it turns to the left or right is based on which angle has the shortest distance to the wall between 45 degrees and 315 degrees. So, if the distance at 45 degrees is closer to the wall, the Neato will turn left since it was already angled towards a left turn. This means that the wall following requires the Neato to be initially pointing towards a wall, though it doesn’t have to be at a 90 degree angle to the wall. 

![](https://github.com/b-wager/comprobo-project-1/blob/main/images/wall_following_diagram_2.png)

Once the Neato has turned to come alongside the wall, it uses lidar data from its wall side to determine if it is closer or farther to the wall than the target distance range. The Neato will adjust based on this calculation, turning closer to the wall when it is too far, and turning away from the wall when it is too close. The Neato also uses proportional control, slowing down as it gets closer to the wall.

Our code uses the minimum distance of an array of lidar data (the distances from the wall at angles 0 through 60 on the right and 300 through 360 on the left) to calculate whether the Neato’s distance is too far or too close to the wall. Using the minimum distance out of many distances at different angles allows us to ensure that the Neato’s understanding of distance to the wall isn’t too skewed by its repeated turns towards and away from the wall as it corrects itself. For example, If we only used the distance to the wall at a 90 degree angle, this distance would be much smaller when the Neato is exactly parallel to the wall than when it is turning away from the wall, this would cause the Neato to turn back towards the wall too soon and possibly crash. 

##### Code Structure
The Neato uses the run_loop to approach the wall, using proportional control to slow down as it gets closer. When the Neato is within target range of the wall, the turn_at_wall function is used to compare the distance to the wall at angles 45 and 315 to determine which direction the Neato should turn in, this sets the self.turned var to *l* or *r*, indicating the turn direction.

Once the neato has made its turn to come alongside the wall, it uses the minimum distance of 61 lidar scans to determine whether it is too close or too far from the wall within the run_loop. The variable self.turned informs which side of the Neato, or which angles, should be used to scan distance to the wall. Then, the angular velocity is adjusted for the Neato based on its distance from the wall.

## The Finite State Machine

Our finite state machine moves the Neato through four states, with a series of transitions that ensure the Neato’s next state is always determined (unless the user keeps the Neato in the teleoperations state forever). The four states are our robot behaviors:
- Teleops
- Draw a Figure-Eight
- Dizzy Collision Avoidance
- Wall Follower

All states and transitions are shown in the diagram below: 
![](https://github.com/b-wager/comprobo-project-1/blob/main/images/finite_state_machine.png)

#### Behavior Summary
The Finite State machine begins in the Teleops state, and behaves based on the user's keyboard input, following the controls outlined in the Teleoperations behavior section. Transition out of the Teleops state to the Draw a Figure-Eight state is triggered by the press of the 8 key. If the user drives the Neato into a collision that activates the bump sensor, the Neato will transition to the Dizzy Collision Avoidance state. 

The Draw a Figure-Eight state draws the figure eight shape to completion, unless interrupted by a collision. When the figure eight is completed, the Neato transitions back to the Teleops state. If there is a collision, the activation of the bump sensor triggers the transition to the Dizzy Collision Avoidance state. 

The Dizzy Collision Avoidance state causes the Neato to make its squiggly line motion, then when the spin is complete, the state will transition to the Wall Following state. 

If the Neato is already in a Wall Following state and enters the Dizzy Collision Avoidance state, it will transition to Teleops. The idea is that when a Neato hits a wall, it will attempt to follow that wall, but if it gets confused and crashes again, the user can take over and maneuver the Neato away from the wall. 

#### Design Decisions
We choose to transition from Teleops to Draw a Figure-Eight by the 8 key press because in the Teleops state, the user is in full control of the Neato, and it would make sense to allow the user to choose to transition to the Draw Figure-Eight state. The Dizzy Collision Avoidance state interrupts the other states without user approval because we assume a collision is not intentional, and if it is, the user should be inconvenienced for abusing our poor Neato.

Wall Following only ever takes place after a Dizzy Collision Avoidance so that it will be certain that there’s an object (potentially a wall) in front of the Neato for it to follow. However, since we knew this object might not be a wall, and our Wall Following code is not perfect, we made it so that Wall Following is exited by another crash, which Leads to the Dizzy Collision Avoidance state, and then to Teleops, to allow the user to drive the Neato away from the wall if desired.

To combine all our behaviors into a finite state machine, we rewrote all their functionality into a node called finite_state_controller.

#### Code Structure
Instead of directly pulling from our previously written nodes, we rewrote the behaviors to make transitioning within our finite state machine easier. We utilized threads to call and switch between behaviors, and they are called in the run_loop whenever the state changes. The reason why we decided to do this is so that we can run the current behavior while still calling the run_loop to collect sensor data. The first thread that runs is the Teleop thread, which functions the same as in the Teleop node, with the added functionality of the 8 key activating the figure-eight behavior.

The Teleop thread ends when either ctrl-c is pressed or the bump sensor is activated. When ctrl-c is pressed, the program stops, but when the bump sensor is activated, the dizzy_reaction thread is subsequently run. The dizzy_reaction thread is also very similar to the dizzy_reaction node. The entire program then waits for dizzy_reaction to finish with thread.join().

The next thread to start is the wall_following behavior. Due to the nature of threads, this function is able to run while the lidar data is being collected in __init__. This allows us to receive updated data while also running the logic for following a wall. If the wall_following fails somehow and the bump sensor is activated, the thread is terminated and the dizzy_reaction is activated again. Finally, we end up back at Teleop so the user is able to control the robot again.

### Challenges:
#### Wall Following:
The Wall Following behavior was the most challenging to implement successfully for a few reasons. First, in simulation it took us a while to find out how long the Neato should turn for the come alongside the wall, since we wanted it to be able to approach from multiple angles. Our initial attempts had the Neato start its turn to come alongside the wall, then, too soon, start turning back towards the wall and crash. We were able to fix this by changing the angles we were using lidar data from.

Once the initial turn to come alongside the wall was completed, we then struggled to fine-tune the Neato's distance correction strategy. It was repeatedly turning very far from the wall, then crashing back into it. At this time, we had two angles that we were getting lidar data at. We experimented with different angles and different amounts of angles, and found that lidar readings from angles 0 to 61 on the right side and 300 to 361 on the other side solved our problem! 

The final challenge of wall following was switching from simulation to the real Neato. We made many adjustments and tried multiple Neatos before we were successful.

#### Finite State Machine:
Combining the different behaviors into a finite state machine was another large challenge. While some of these behaviors were very simple on their own, putting them all into one file took a lot of refactoring and retesting - more time than we had expected.

#### Threading:
Threads can be very complicated and annoying to implement due to their difficulty to debug. We spent a lot of time trying to figure out why the teleop thread wasn’t finishing, before we realized that the get_key() function was blocking it from looping.

### Future Improvements:
To improve this project in the future we would like to alter the Collision Avoidance behavior to have the Neato avoid collisions completely, instead of backing away dizzily once a collision has happened. Though the dizzy behavior is cute, avoiding a collision in the first place is more useful. It would be extra nice to have the Neato circumvent obstacles during its figure-eight path, and then continue the figure-eight shape afterwards, though, this would get difficult in a finite state machine, as the Draw a Figure-Eight state would need to remember where the Neato was on the path before it switched to a Obstacle Avoidance state.

### Take Aways:
One of the biggest take-aways for us was learning how to use the lidar scan data to affect the robots movement. This was a very satisfying way to have the neato interact with its environment.

We also got great practice debugging, both in simulation and with the physical Neatos. It was frustrating at times, but we both feel we have learned a lot. 

### Team Contributions:
Both team members contributed to the code and the write up. Brooke led the code and Kelsey led the write up. Kelsey provided most of the wall-following code and Brooke combined the parts into the finite-state machine.
