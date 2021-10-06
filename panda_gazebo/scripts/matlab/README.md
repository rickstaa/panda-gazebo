# MATLAB/Simulink scripts

This folder contains several useful MATLAB and Simulink scripts.

## Use custom messages

To use the custom messages found in the catkin workspace in MATLAB or Simulink you have to run the
`rosgenmsg` command while passing it the path of the folder that contains the messages. You then
have to add the output folder of this command to the MATLAB path. This is done using
the `addpath` and `savepath` commands.
