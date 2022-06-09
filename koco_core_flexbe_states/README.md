# Koco Core Flexbe States
This repository holds various core FlexBE states to operate with the KoCo system. It should not contain robot or use-case specific states.

## Check digital input
Checks digital input and outcomes its state.

## Compare KocoPose with DB
Compare a KocoPose that is passed to the state in userdata with the same pose that is stored in MongoDB (`child_frame_id` defines the DB entry name). This state allows you to set the tolerances in which the errors should be. If no tolerance is set (i.e. empty array `[]`), that DOF is ignored. Possible outcomes are `within_tolerance`, `outside_tolerance` and `error`.

## Insert to db
Inserts (or updates) a desired ROS message object to the database.

## Move joints from db
Moves the robot to the joints read from the database using movej action server.

## Move sequence action state
Calls the action server that triggers a sequence of motions, which were previously sent appended with the seq movel and seq movej action servers.

## Movej action state
Allows moving the robot with movej action server.

## Movel action state
Allows moving the robot with movel action server.

## Seq move joints from db
Appends a joint move to the sequence action server where joints are read from the database.

## Seq movej action state
Appends a joint move to the sequence action server.

## Seq movel action state
Appends a linear move to the sequence action server.

## Service SetOutputs
Calls a desired service with SetOutputs message type.

## Service TriggerString
Calls a desired service with TriggerString message type.

## Wait for digital input
Waits for a digital input to hold a predefined value and outcomes true if yes or false if not.

# Adding new states

At a certain point you might want to add a state that you created to this repository for other people to use. Great! We all love that. **However** (there is always a catch), we would like you to take some time to write a test case for your state.

Feel free to have a look on how the other tests are written and if in doubt find some extra resources for writing [Flexbe test](http://wiki.ros.org/flexbe/Tutorials/Writing%20State%20Tests%20Using%20flexbe_testing) online.