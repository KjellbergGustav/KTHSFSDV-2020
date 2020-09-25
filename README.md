# KTHSFSDV-2020
Tasks for KTHFS 2020 application

# ROS exercice

# Building the project

## Installation
Install ROS melodic following [this guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Running the program
```
1. Add a workspace
$ mkdir -p ~/[your_workspace]/src # create the dir
2. Build the workspace
$ catkin build
3. Add the folder exc1 into the folder /src
4. Add the workspace to the env
$ . ~/[your_workspace]/devel/setup.bash
5. Add the source
$ source /opt/ros/melodic/setup.bash
$ source devel/setup.bash
6. Start rosecore from your workspace root
$ roscore
7. Open a new terminal window and start the publisher node
$ rosrun name_frequency_broadcast name_frequency_publisher_node.py
8. Open a new terminal window and start the receiver node
$ rosrun name_frequency_receiver name_frequency_receiver_node.py
9. You can echo the topics used to the data sent
$ rostopic echo kjellberg
$ rostopic echo /kthfs/result
```

# Building the project from scratch with explanations

## Installation
Install ROS melodic following [this guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

## 1. Create a catkin workspace
In order to have a workspace we need to create one. This is done by using the following commands

```
$ mkdir -p ~/kthfsdv/src # create the dir
$ cd ~/kthfsdv/ #navigate to your workspace
$ catkin build #Builds the workspace, will need to do this multiple times as new deps are added.
```

## 2. Create the packages
First we need to create the workspace structure that we are asked to have

```
$ roscd ~/kthfsdv/src
$ mkdir exc1
```

Second, let's create our first package, i.e. the publisher.

```
$ catkin_create_pkg name_frequency_broadcast stf_msg rospy # This line will
create a package with the name name_frequency_broadcast, that depends on std messages 
as we'll use Ints and rospy as we'll use python.
```

Then we need to build our workspace again, using

```
$ catkin build
```
Also, add the workspace to the environment with the generated `setup.bash`
```
$ . ~/kthfsdv/devel/setup.bash #Adds workspace to env
```

Now the first package should be setup and ready to be configured.
First let's look at the direct deps by using the command
```
$ rospack depends1 name_frequency_broadcast
```

Which returns
```
rospy
std_msgs
```
which seems correct as that is what we said previously.

### Configure the package info

 To make the package more clear, let's define author and licens. I went with BSD.

In order to change, just navigate to your `package.xml` and enter `vim package.xml` to edit.

### Create the receiver
`Repeat the above steps.`


### Build the packages

First, add the source
```
$ source /opt/ros/melodic/setup.bash
```

Now build again to verify that the workspace is valid, which it should be.

## Creating a node
Next up is to create a node for the sender. We do this by adding a `scripts` folder in the broadcaster package.

```
$ roscd namefreq_broadcast
$ mkdir scripts
$ touch name_freq_pub_node.py
$ chmod +x name_freq_pub.py # make it executable
```
Double check that the file is executable by `ls` and verify that the file is green. (depending on terminal settings).


### Write the program
Now write the code that your node should execute. Also, keep in mind that you must reference the node in the CMmakelist.txt file in the pkg.

Add the following reference

```
catkin_install_python(PROGRAMS scripts/name_frequency_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Now try to build the one node. It should work but got an error due to copying the above into CMakeList thus including `\\`. Changed that and it worked.

# Run the publisher

Now, let's run the publisher and see that it works. Navigate to the root folder and start the core. Secondly, ensure that the package exists by.
```
$ roscore
$ rospack list
```
Then run the node:
```
$ rosrun name_frequency_broadcast name_frequency_publisher_node.py
```
Since we're logging we see the number increasing. We can remove the log if that's annoying. Let's ensure that our node and topic exist by:

```
$ rosnode list
$ rosnode topic
```

## Create the receiver
`Repeat the steps for the publisher node but program as a receiver.`

Build with `$ catkin build` to ensure that the new node also works. Don't forget to add the source.

### Run both nodes
1. `roscore`
2. `start publisher`
3. `start receiver`
4. `verify communication`

I'm contantly struggeling with one terminal not finding the packages??? Answer: I had forgetten to add `$ source devel/setup.bash` - didn't unserstand it was required for each new terminal, though the session was global.

---

# General coding excercise

## Installation
Install python on your system, either by itself or by using [anaconda or miniconda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/). `Disclaimer: conda and ROS do not play well together, either use a VM for one of them or install a python without using conda if you want to be able to compile both project.` In the file `graphics.py` you'll need to enter the path to the `/plots` directory to correctly save the graph.

### Packages
```
Python w/o conda:
$ pip install numpy
$ pip install matplotlip

Both:
$ pip install tikzplotlib
```
Depending on how your system is set up you might need to specify that it should be installed on your user and not the entire machine, use `$ pip install [pkg] --user`

## Running the program
After installing the above it is time to run the program.

### Navigate to folder
```
$ cd ~/KTHFSDV-2020/exc2/src
```
### Run program
```
$ python main.py
```
The program will now run, first asking you what you want `t_0` to be, keep in mind that this is defined as an integer. Second, it will prombt you to enter `t_n`, i.e the for span of `t` you want see the function.

You will see a graph showing up, similair to the one below.

![One period of the function](https://media.giphy.com/media/uvUIXY8bcCFQ7aGrOl/giphy.gif)

The graph will show the function value over time `t` and dynamically adapt its axes. As the function is periodic (mentioned in the exercise instruction) we do not want to show redundant inforamtion. Therefore, the animation will stop after the first period. You will then be prombted with the option to continue to animate the graph. If you choose yes, the program will run until `t_n`. If not you will have the option to save the graph. You will also get this option as you reach `t_n`.
