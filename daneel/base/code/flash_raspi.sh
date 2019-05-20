#!/bin/bash
ssh -4 pi@digduck 'rm teensyBinaries/*.hex'
scp -4 ./Release/*.hex pi@digduck:~/teensyBinaries/
echo $(pwd)
ssh -4 pi@digduck 'tools/teensy_loader_cli/teensy_loader_cli -mmcu=mk64fx512 -s -v -w teensyBinaries/*.hex'

