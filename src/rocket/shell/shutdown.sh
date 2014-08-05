#!/bin/bash

echo "Using shell script to shutdown rocket"
rostopic pub /cmd std_msgs/String -1 shutdown
