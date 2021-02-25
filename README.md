### EXPRL_Assignment_2-Behavioural Architecture

This is the assgnment of the lab 2 of Experimental robotics lab simulating a pet robot interacting with a human in ros_noetic

## Software architecture

![Screenshot from 2021-02-25 21-07-06](https://user-images.githubusercontent.com/62798224/109227459-a7593680-77c0-11eb-9251-7bf5a30bcf5e.png)
![Screenshot from 2021-02-25 21-05-03](https://user-images.githubusercontent.com/62798224/109227470-aaecbd80-77c0-11eb-8247-4896180f2cbe.png)
![Screenshot from 2021-02-25 21-04-24](https://user-images.githubusercontent.com/62798224/109227474-ade7ae00-77c0-11eb-802c-baf1b46228d6.png)


*'person_command'*
This node publishes a topic command. The user gives a command either **sleep*** or **play**.If its play a random coordinate is generated and assigned so that the dog will go there.

*'drive_bot'*
This component is responsible for making the pet robot navigate in the modelled world based on the pos(target position) it receives.

*'go_to_point_ball'*
This component is responsible for making the green ball navigate in the modelled world based on the pos(target position) it receives.

*'state_machine'*
The state_machine shifts the three states namely : 
**sleep** : In sleep state the robot dog goes to its home an sleeps after certain time it goes to the normal state
**normal** : In normal state the robot just roams arround untill it receives a command to play or it goes to sleep again
**play** : In play state the robot receive a command from a preson and goes there

## State diagram
![SD](https://user-images.githubusercontent.com/62798224/99132134-7c821280-2615-11eb-97a4-5b45a627cf05.png)


**sleep** : In sleep state the robot dog goes to its home an sleeps after certain time it goes to the normal state
**normal** : In normal state the robot just roams arround untill it receives a command to play or it goes to sleep again
**play** : In play state the robot receive a command from a preson and goes there

## Package and files list




Along with those files a zip file named doc is present.which contains the documentation from Doxygen.General folder could not be uploaded since it had too many files.

## Installation and running the code
*'Clone the repository'*
```
git clone https://github.com/raghuveer-sid/exp_assignment2
```
*'run roscore'*
```
roscore
```
*'open new terminal'*
*'goto specific directory'*
```
cd exp_assignment2
```
*'Give permissions (not neede unless an error comes up)'*
```
chmod +x state_machine.py
chmod +x person_command.py
```
*'open new terminal'*
*' goto workspace'*
*'source'*
```
source devel/setup.bash
```
*'make'*
```
catkin_make
```
*'launch file'*
```
roslaunch exp_assignment gazebo_world.launch
```
## Working

It is as simple as it sounds.Initially the dog wakes up and goes to the normal state where it moves randomly untill it receives a command from a person if it receives the command then it goes to the command point but if it does not receive a command after some time then the dog will go to sleep
The layout is set to **8x8**..The person can move from one place to another randomly.

## Limitations and Possible improvements

* The main goal was to make the progtram work completely on its ow but I could not do it.The part where the ball goes to different places is not implimented correctly so the followig topic is published in adifferent terminal to make it u for it. *'/reaching_goal/goal'*.

* One more proble is the robot description the code works well with basic design but when neck connection is added the robot only looks at human.This is something that should be fixed.

## Author and contact

**Raghuveer Siddaraboina**
**raghuveersiddaraboina@gmail.com**





