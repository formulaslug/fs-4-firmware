import subprocess
import os
import polars as pl 
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot

def labelPlots(title, axis):
  plt.title(title)
  plt.ylabel(axis)

# Pull test data from parquet file
cwd = os.getcwd()
print(cwd )
RUN_NAME = "11222025_21.parquet"
TIME_DATA_PROVIDED = True
RUN_DIR = "../runData" if "tools" in cwd else "/runData"
runDataDF = pl.read_parquet(cwd+"/"+RUN_DIR+"/"+RUN_NAME)

# Pull voltage, current, and time from data
voltageArray = runDataDF["ACC_POWER_PACK_VOLTAGE"].to_numpy() 
currentArray = runDataDF["SME_TEMP_BusCurrent"].to_numpy() 
timeArray = runDataDF["Time_ms"].to_numpy() 

# Make mask to get rid of nan values
voltageTempArray = np.nan_to_num(voltageArray)
voltageTempArray[voltageTempArray<0] = 0
voltageNanMask = np.ma.make_mask(voltageTempArray)
currentTempArray = np.nan_to_num(currentArray)
currentTempArray[currentTempArray<0] = 0
currentNanMask = np.ma.make_mask(currentTempArray)
nanMask = np.logical_and(voltageNanMask,currentNanMask)

# Clean data
voltageArray = voltageArray[nanMask]
currentArray = currentArray[nanMask]
timeArray = timeArray[nanMask]

# Convert to millivolts and milliamps
CELLS_IN_SERIES = 30
CELLS_IN_PARALLEL = 30
MILLI = 1000
voltageConverted = (voltageArray/CELLS_IN_SERIES)*MILLI
currentConverted = (currentArray/CELLS_IN_PARALLEL)*MILLI

# Make tuple to feed into C++ function
voltageCurrentPairs = tuple(map(tuple,np.vstack((voltageConverted,currentConverted)).T))

# Call function on every pair for estimated capacity drained
C_TEST_FUNC = ".\\..\\src\\testRunDataFuncRunner.exe"
estimatedDischargeCapacity = np.zeros(len(voltageCurrentPairs))
for i in range(len(voltageCurrentPairs)):
  milliVolts = int(voltageCurrentPairs[i][0])
  milliAmps = int(voltageCurrentPairs[i][1])

  cmd = C_TEST_FUNC+" "+str(milliVolts)+" "+str(milliAmps)
  cmdOutput = subprocess.run(cmd, capture_output=True, text=True)

  if int(cmdOutput.returncode) != 0:
    print(cmdOutput.stderr)
  else:
    print("("+str(milliVolts)+", "+str(milliAmps)+", "+cmdOutput.stdout+")")
    result = float(cmdOutput.stdout)
    estimatedDischargeCapacity[i] = result
emptyDischargeMask = np.ma.make_mask(np.nan_to_num(estimatedDischargeCapacity))

# Clean discharge and convert to soc
BATTERY_CAPACITY = 2600 # mAh
estimatedSOC = 1-(estimatedDischargeCapacity[emptyDischargeMask]/BATTERY_CAPACITY)

# Plot data
if TIME_DATA_PROVIDED:
  indexArray = timeArray 
else:
  indexArray = np.linspace(0,len(voltageCurrentPairs),len(voltageCurrentPairs))
plt.figure(1,figsize=[20,12])

voltagePlot = host_subplot(211)
voltagePlotParasite = voltagePlot.twinx()
voltagePlot.plot(indexArray, voltageConverted)
voltagePlotParasite.plot(indexArray[emptyDischargeMask], estimatedSOC,color="k")
voltagePlot.set_ylabel("Voltage (mV)")
voltagePlotParasite.set_ylabel("SOC %")
plt.legend()

currentPlot = host_subplot(212)
currentPlotParasite = currentPlot.twinx()
currentPlot.plot(indexArray, currentConverted)
currentPlotParasite.plot(indexArray[emptyDischargeMask], estimatedSOC,color="k")
currentPlot.set_ylabel("Current (mA)")
currentPlotParasite.set_ylabel("SOC %")
plt.legend()