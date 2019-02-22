#!/bin/bash

scp ./Release/propulsion_rustyDuck_2019.hex pi@digduck:~/teensyBinaries/
echo $(pwd)
ssh pi@digduck 'tools/teensy_loader_cli/teensy_loader_cli -mmcu=mk64fx512 -s -v -w teensyBinaries/propulsion_rustyDuck_2019.hex'

