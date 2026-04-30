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
#C_TEST_FUNC = ".\\..\\src\\testRunDataFuncRunner.exe"
# for mac
C_TEST_FUNC = "../src/testRunDataFuncRunner"
estimatedDischargeCapacity = np.zeros(len(voltageCurrentPairs))
for i in range(len(voltageCurrentPairs)):
  milliVolts = int(voltageCurrentPairs[i][0])
  milliAmps = int(voltageCurrentPairs[i][1])

  cmd = [C_TEST_FUNC, str(milliVolts), str(milliAmps)]
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

# -------------------- COULOMB COUNTING ADDED --------------------
C_COULOMB_FUNC = "../src/testRunCoulombCountingRunner.exe"

COULOMB_INITIAL_SOC = 100.0
COULOMB_CAPACITY_AH = 2.6
COULOMB_ETA_CHARGE = 1.0
COULOMB_ETA_DISCHARGE = 1.0

coulombSOC = np.zeros(len(currentConverted))
if len(currentConverted) > 0:
  coulombSOC[0] = COULOMB_INITIAL_SOC / 100.0

prev_soc_pct = COULOMB_INITIAL_SOC

for i in range(1, len(currentConverted)):
  prev_current_A = currentConverted[i-1] / 1000.0
  current_A = currentConverted[i] / 1000.0
  dt_s = (timeArray[i] - timeArray[i-1]) / 1000.0

  cmd = [
    C_COULOMB_FUNC,
    str(prev_soc_pct),
    str(COULOMB_CAPACITY_AH),
    str(COULOMB_ETA_CHARGE),
    str(COULOMB_ETA_DISCHARGE),
    str(dt_s),
    str(prev_current_A),
    str(current_A)
  ]

  cmdOutput = subprocess.run(cmd, capture_output=True, text=True)

  if int(cmdOutput.returncode) != 0:
    print(cmdOutput.stderr)
    coulombSOC[i] = coulombSOC[i-1]
  else:
    prev_soc_pct = float(cmdOutput.stdout)
    coulombSOC[i] = prev_soc_pct / 100.0
# ------------------ SOH COUNTING -------------------------
SOH_FUNC = "../src/testRunSOH.exe"

INIT_SOH = 1.00

stateOfHealth = np.zeros(len(currentConverted))
stateOfHealth[0] = INIT_SOH

for i in range(1, len(currentConverted)):
  prev_SOH = stateOfHealth[i-1] * 100
  currentDraw = currentConverted[i] / MILLI
  milliSinceCall = timeArray[i] - timeArray[i-1]
  
  cmd = [
    SOH_FUNC,
    str(prev_SOH),
    str(currentDraw),
    str(milliSinceCall)
  ]

  cmdOutput = subprocess.run(cmd, capture_output=True, text=True)

  if int(cmdOutput.returncode) != 0:
    print(cmdOutput.stderr)
  else:
    print(cmdOutput.stdout)
    stateOfHealth[i] = float(cmdOutput.stdout) / 100.0
'''
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
'''
# -------------------- KALMAN FILTER ----------------------
 
C_KALMAN_FUNC = "../src/testRunKalmanFilter.exe"
 
KALMAN_INITIAL_SOC        = 100.0   
KALMAN_CAPACITY_AH        = 2.6
KALMAN_ETA_CHARGE         = 1.0
KALMAN_ETA_DISCHARGE      = 1.0
KALMAN_P0                 = 0.02    
KALMAN_Q                  = 0.001   
KALMAN_R                  = 0.01   
KALMAN_VOLTAGE_INTERVAL_S = 5.0     
 
kalmanSOC         = np.zeros(len(currentConverted))
kalmanUncertainty = np.zeros(len(currentConverted))
 
if len(currentConverted) > 0:
  kalmanSOC[0]         = KALMAN_INITIAL_SOC / 100.0
  kalmanUncertainty[0] = KALMAN_P0
 
prev_kalman_soc_pct = KALMAN_INITIAL_SOC
prev_kalman_P       = KALMAN_P0   
 
for i in range(1, len(currentConverted)):
  prev_current_A = currentConverted[i-1] / 1000.0  
  current_A      = currentConverted[i]   / 1000.0
  voltage_V      = voltageConverted[i]   / 1000.0   
  dt_s           = (timeArray[i] - timeArray[i-1]) / 1000.0  

  cmd = [
    C_KALMAN_FUNC,
    str(prev_kalman_soc_pct),
    str(KALMAN_CAPACITY_AH),
    str(KALMAN_ETA_CHARGE),
    str(KALMAN_ETA_DISCHARGE),
    str(dt_s),
    str(prev_current_A),
    str(current_A),
    str(voltage_V),
    str(prev_kalman_P),       # pass current P in so covariance is continuous
    str(KALMAN_Q),
    str(KALMAN_R),
    str(KALMAN_VOLTAGE_INTERVAL_S)
  ]
 
  cmdOutput = subprocess.run(cmd, capture_output=True, text=True)
 
  if int(cmdOutput.returncode) != 0:
    print(cmdOutput.stderr)
    kalmanSOC[i]         = kalmanSOC[i-1]
    kalmanUncertainty[i] = kalmanUncertainty[i-1]
  else:
    parts = cmdOutput.stdout.strip().split(",")
    prev_kalman_soc_pct  = float(parts[0])
    prev_kalman_P        = float(parts[1]) if len(parts) > 1 else prev_kalman_P
    kalmanSOC[i]         = prev_kalman_soc_pct / 100.0
    kalmanUncertainty[i] = prev_kalman_P
 
# Plot data
if TIME_DATA_PROVIDED:
  indexArray = timeArray 
else:
  indexArray = np.linspace(0,len(voltageCurrentPairs),len(voltageCurrentPairs))
plt.figure(1,figsize=[20,12])

voltagePlot = host_subplot(211)
voltagePlotParasite = voltagePlot.twinx()
voltagePlot.plot(indexArray, voltageConverted, label="Voltage")
voltagePlotParasite.plot(indexArray[emptyDischargeMask], estimatedSOC, color="k", label="Lookup SOC")
voltagePlotParasite.plot(indexArray[emptyDischargeMask], stateOfHealth, color="r", label="SOH")
voltagePlotParasite.plot(indexArray, coulombSOC, color="g", label="Coulomb SOC")
voltagePlotParasite.plot(indexArray, kalmanSOC,                          color="b", label="Kalman SOC")
voltagePlot.set_ylabel("Voltage (mV)")
voltagePlotParasite.set_ylabel("SOC %")
voltagePlot.legend(loc="upper left")
voltagePlotParasite.legend(loc="upper right")

currentPlot = host_subplot(212)
currentPlotParasite = currentPlot.twinx()
currentPlot.plot(indexArray, currentConverted, label="Current")
currentPlotParasite.plot(indexArray[emptyDischargeMask], estimatedSOC, color="k", label="Lookup SOC")
currentPlotParasite.plot(indexArray[emptyDischargeMask], stateOfHealth, color="r", label="SOH")
currentPlotParasite.plot(indexArray, coulombSOC, color="g", label="Coulomb SOC")
currentPlotParasite.plot(indexArray, kalmanSOC,color="b", label="Kalman SOC")
currentPlot.set_ylabel("Current (mA)")
currentPlotParasite.set_ylabel("SOC %")
currentPlot.legend(loc="upper left")
currentPlotParasite.legend(loc="upper right")

uncertaintyPlot = host_subplot(313)
uncertaintyPlot.plot(indexArray, kalmanUncertainty, color="b", label="Kalman P (uncertainty)")
uncertaintyPlot.set_ylabel("Covariance P")
uncertaintyPlot.set_xlabel("Time (ms)")
uncertaintyPlot.legend(loc="upper right")
plt.tight_layout()

plt.show()