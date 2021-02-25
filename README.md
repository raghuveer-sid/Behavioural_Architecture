### EXPRL_Assignment_2-Behavioural Architecture

This is the assgnment of the lab 2 of Experimental robotics lab simulating a pet robot interacting with a human in ros_noetic

## Software architecture



*'person_command'*
This node publishes a topic command. The user gives a command either **sleep*** or **play**.If its play a random coordinate is generated and assigned so that the dog will go there.

*'getpoint'*
This note publishes the commad_point and gives the location to which the person is trying to point

*'gotopoint'*
This is where the robot dog recieves the location from the person and goes there also while publishing it

*'state_machine'*
The state_machine shifts the three states namely : 
**sleep** : In sleep state the robot dog goes to its home an sleeps after certain time it goes to the normal state
**normal** : In normal state the robot just roams arround untill it receives a command to play or it goes to sleep again
**play** : In play state the robot receive a command from a preson and goes there

## State diagram


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
cd my_dog/src
```
*'Give permissions'*
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
roslaunch my_dog main.launch
```
## Working

It is as simple as it sounds.Initially the dog wakes up and goes to the normal state where it moves randomly untill it receives a command from a person if it receives the command then it goes to the command point but if it does not receive a command after some time then the dog will go to sleep
The layout is set to **50x50**..The person can move from one place to another randomly.

## Limitations and Possible improvements

* The time difference to travel the command point is calculated so just 5 seconds is given.
* The dog could have gotten tired in normal or play so it could be sent to home.
* The robot dog could have receive command externally.
* The verbal interaction between person and robot dog is limited.
* Could be implemented in gazebo for a good vizuvalization.

## Author and contact

**Raghuveer Siddaraboina**
**raghuveersiddaraboina@gmail.com**





