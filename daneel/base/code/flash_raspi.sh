#!/bin/bash
ssh pi@digduck 'rm teensyBinaries/*.hex'
scp ./Release/*.hex pi@digduck:~/teensyBinaries/
echo $(pwd)
ssh pi@digduck 'tools/teensy_loader_cli/teensy_loader_cli -mmcu=mk64fx512 -s -v -w teensyBinaries/*.hex'

