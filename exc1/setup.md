# ROS exercice

## 1. Create a catkin workspace
In order to have a workspace we need to create one. This is done by using the following commands

```
mkdir -p ~/kthfsdv/src # create the dir
cd ~/kthfsdv/ #navigate to your workspace
catkin build #Builds the workspace, will need to do this multiple times as ne   w deps are added.
```

## 2. Create the packages
First we need to create the workspace structure that we are asked to have

```
roscd ~/kthfsdv/src
mkdir exc1
```

Second, let's create our first package, i.e. the publisher.

```
catkin\_create\_pkg name\_frequency\_broadcast stf\_msg rospy # This line will create a package with the name name\_frequency\_broadcast, that depends on std messages as we'll use Ints and rospy as we'll use python.
```

Then we need to build our workspace again, using

```
catkin build
```
Also, add the workspace to the environment wiuth the generated `setup.bash`
```
. ~/kthfsdv/devel/setup.bash #Adds workspace to env
```

Now the first package should be setup and ready to be configured. First let's look at the direct deps by using the command
```
rospack depends1 name\_frequency\_broadcast
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

First, att the source
```
source /opt/ros/melodic/setup.bash
```

Now build again to verify that the workspace is valid, which it should be.

## Creating a node
Next up is to create a node for the sender. We do this by adding a `scripts` folder in the broadcaster package.

```
roscd name\freq\_broadcast
mkdir scripts
touch name\_freq\_pub\_node.py
chmod +x name\_freq\_pub.py # make it executable
```
Double check that the file is executable by `ls` and verify that the file is green. (depending on termrinal settings).


### Write the program
Now write the code that your node sohuld execute. Also, bear in mind that you must reference the node in the CMmakelist.txt file in the pkg.

Add the following reference

```
catkin_install_python(PROGRAMS scripts/name\_frequency\_publisher\_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Now try to build the one node. It should work but got an error due to copying the above into CMakeList thus including `\\`. Changed that and it worked.

# Run the publisher

Now, let's run the publisher and see that it works. Navigate to the root folder and start the core. Secondly, ensure that the package exists by.
```
roscore
rospack list
```
Then run the node:
```
rosrun name_frequency_broadcast name_frequency_publisher_node.py
```
Since we're logging we see the number increasing. We can remove the log if that's annoying. Let's ensure that we our node and topic exist by:

```
rosnode list
rosnode topic
```

## Create the receiver
`Repeat the steps for the publisher node but program as a receiver.`

Build with `catkin build` to ensure that the new node also works.

### Run both nodes
1. `roscore`
2. `start publisher`
3. `start receiver`
4. `verify communation`

I'm contantly struggeling with one terminal not finding the packages??? Answer: I had forgetten to add `source devel/setup.bash` - didn't unserstand it was required for each new terminal, though the session was global.
