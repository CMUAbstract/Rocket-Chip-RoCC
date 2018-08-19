This directory contains all source code for the Rocket-Chip processor and the RoCC co-processor. Custom definitions and 
implmentations of RoCCs can be found in the tile/LazyRoCC.scala file. The accumulator RoCC has been modified to be a dot product RoCC.
This dot product implementation can only fetch a single word from memory at a time and uses the custom0 instruction found in the 
accum.c file under the test directory at the top. To modify or add new RoCC co-processors, go to subsystem/configs.scala and add 
or modify the instantiations found in the WithRoccExample class on line 188.
