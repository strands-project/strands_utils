# Change environment varialbe HOSTNAME to a ros friendly format

This script modifies the hostname of the machine (removes -, spaces, changes upper case letters to lower case letters) and sets the result in the STRANDS_HOSTNAME environment variable in the bashrc.

The STRANDS_HOSTNAME variable is used to run nodes on different machines using the same roscore. 

Run this script once on each machine. 
