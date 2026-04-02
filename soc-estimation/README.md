# the files
## src
this contains all of the cpp code
- voltage_lookup.cpp/.h are the functions that calculate the SOC from the lookup table, uses both voltage and current to get a more accurate estimation
- batteryLUT.h is the lookup table that votlage_lookup.cpp references
- testRunDataFuncRunner.cpp is a wrapper for calling SOC estimations based on command line inputs, outputs back out to the command line, used by python scripts

## tools
this contains the data sheets for the batteries and helper python code
- batteryLUTGenerator.py generates the lookup tables for each of the batteries, run whenever we use a new battery with a different discharge graph
- testRunData.py parses the parquet run data from test days and calls the cpp function to test with old data, uses the testRunFuncDataRunner.exe
- old-soc-voltage-lookup.py has some old code that i was messing with, should probably delete

# for voltage based soc estimation
1) generate a LUT for your battery using batterLUTGenerator.py
2) put this header file into src
3) call functions from voltage_lookup.cpp/.h 
4) everything else can be ignored

# to test against run data
1) compile and build testRunDataFuncRunner.cpp
2) change the file path for the desired run data to test against in testRunData.py
3) add the required list of libraries in requirements.txt to your python environment
4) run testRunData.py script which uses the cpp function