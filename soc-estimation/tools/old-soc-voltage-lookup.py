import os
import polars as pl 
import numpy as np
from scipy.interpolate import griddata
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot

"""
voltagePts = each load col, stacked after one another  
currentPts = 30 x 100 pts, 20 x 100 pts, ...
capacityValues = 6x soc col
"""

SHOW_PLOTS = True
GENERATE_BATTERY_LUT =False

# Reading Battery Discharge Dataframe
dischargeParquet = "data_sources/US18650VTC5A-discharge-data.parquet"
dischargeData = pl.read_parquet(os.path.join(os.getcwd(), dischargeParquet))

# three variables: voltage, current, capacity
# pulls out data for griddata
voltagePts = np.empty(0)
#currentLabels = ["30A", "20A", "10A", "1C", "0.5C", "0.2C"]
# currentLabels = ["10A", "1C", "0.5C", "0.2C"]
#currentLabels = ["35A", "30A", "20A", "15A", "10A", "5A", "2.5A"]
currentLabels = ["10A", "5A", "2.5A"]
for load in currentLabels:
  voltagePts = np.hstack((voltagePts, dischargeData[load].to_numpy()))

currentPts = np.empty(0)
#currentValues = [30,20,10,3,1.5,.6]
# currentValues = [10,3,1.5,.6]
currentValues = [10,5,2.5]
for load in currentValues:
  currentPts = np.hstack((currentPts, np.ones(100)*load))

capacityValues = np.empty(0)
for i in range(len(currentLabels)):
  capacityValues = np.hstack((capacityValues, dischargeData["Capacity"].to_numpy()))
  
# clean data
voltagePts = np.nan_to_num(voltagePts)
invalidValuesMask = np.ones(len(currentLabels)*100, dtype=bool)
invalidValuesMask[voltagePts==0] = 0
voltagePts = voltagePts[invalidValuesMask]
currentPts = currentPts[invalidValuesMask]
capacityValues = capacityValues[invalidValuesMask]

# generates grid on which we get our interpolated values
VOTLAGE_MIN = 2.5
VOLTAGE_MAX = 4.25
CURRENT_MAX = 12
gridVoltage, gridCurrent = np.mgrid[VOTLAGE_MIN:VOLTAGE_MAX:1000j, 0:CURRENT_MAX:1000j]

# interpolates for values
linearInterp = griddata((voltagePts,currentPts),capacityValues,(gridVoltage,gridCurrent), method='linear')
nearestInterp = griddata((voltagePts,currentPts),capacityValues,(gridVoltage,gridCurrent), method='nearest')
interpData = np.nan_to_num(linearInterp)
mask = np.ma.make_mask(interpData)
np.putmask(interpData,~mask,nearestInterp)

if SHOW_PLOTS:
  # figures
  def plotTemplate():
    plt.gca().set_xlabel('Voltage (V)')
    plt.gca().set_ylabel('Current (A)')
    figure = plt.scatter(voltagePts, currentPts, c=capacityValues, s=10, edgecolors='black',linewidths=0.2,vmin=0,vmax=CAPACITY_MAX)
    plt.gcf().colorbar(figure)
  CAPACITY_MAX = 2500
  interpPlots = plt.figure(1,[14,10])
  plt.subplot(221)
  figure = plt.imshow(interpData.T, extent=(VOTLAGE_MIN,VOLTAGE_MAX,0,CURRENT_MAX), aspect="auto", origin="lower",vmin=0,vmax=CAPACITY_MAX) # interp plot
  plotTemplate()
  plt.title("Linear & Nearest Combined")
  plt.subplot(222)
  figure = plt.imshow(linearInterp.T, extent=(VOTLAGE_MIN,VOLTAGE_MAX,0,CURRENT_MAX), aspect="auto", origin="lower",vmin=0,vmax=CAPACITY_MAX) # interp plot
  plotTemplate()
  plt.title("Linear Intperolation")
  plt.subplot(223)
  figure = plt.imshow(nearestInterp.T, extent=(VOTLAGE_MIN,VOLTAGE_MAX,0,CURRENT_MAX), aspect="auto", origin="lower",vmin=0,vmax=CAPACITY_MAX) # interp plot
  plotTemplate()
  plt.title("Nearest Interpolation")

  # test points
  rand = np.random.default_rng()
  testVoltage = rand.random(100)*(VOLTAGE_MAX-VOTLAGE_MIN) + VOTLAGE_MIN
  testCurrent = rand.random(100)*(CURRENT_MAX)
  testInterp = griddata((voltagePts,currentPts),capacityValues,(testVoltage,testCurrent), method='nearest')
  plt.scatter(testVoltage,testCurrent,c=testInterp, edgecolors='black', vmin=0,vmax=CAPACITY_MAX)
  plt.show()

  # Reading Test Run Dataframe
  runParquet = "data_sources/CombinedEndurance_0810_0817_2025.parquet"
  runData = pl.read_parquet(os.path.join(os.getcwd(), runParquet))

  # Interpolating Run Data
  CELLS_IN_SERIES = 30 
  MILLIAMPS_IN_AMPS = 1000
  runLap = runData["Lap"]
  runIndex = np.linspace(0,runLap.len(), runLap.len())
  runVoltage = runData["ACC_POWER_PACK_VOLTAGE"].to_numpy()/CELLS_IN_SERIES
  runCurrent = runData["ACC_POWER_CURRENT"].to_numpy()/MILLIAMPS_IN_AMPS
  runSOC = runData["ACC_POWER_SOC"]
  estimatedSOC = griddata((voltagePts,currentPts),capacityValues,(runVoltage,runCurrent), method='nearest')

  # Plotting Run Data Agaisnt Estimated SOC
  CELL_CAPACITY = 30
  plt.figure(2,figsize=[15,5])
  fig2 = host_subplot(111)
  fig2p = fig2.twinx()
  fig2.set_ylabel("SOC %")
  fig2p.set_ylabel("Current")
  #fig2p.set_ylabel("Voltage")

  fig2.plot(runIndex,100-(estimatedSOC/CELL_CAPACITY), label="estimated")
  fig2.plot(runIndex,runSOC, label="run")
  fig2p.plot(runIndex,runCurrent,label="Current")
  #fig2p.plot(runIndex,runVoltage,label="Voltage")
  plt.ylim((0,105))
  plt.xlim((0,2.02e5))
  plt.legend()

# Output Data for C
if (GENERATE_BATTERY_LUT):
  voltageSamples, currentSamples = np.mgrid[2.5:4.0:20j, 0:5:10j]
  fileName = "tools/batteryLUT.h"
  batteryName = "US18650VTC5A"
  
  # Create / Overwrite file
  try:
    open(fileName,"xt")
  except:
    os.remove(fileName)
    open(fileName,"xt")
  finally:
    file = open(fileName,"wt")

  # Header
  file.write("// "+batteryName+" Lookup Table\n")
  file.write("// Voltages (V): "+str(voltageSamples[0][0])+" to "+str(voltageSamples[-1][0])+" ("+str(len(voltageSamples[0]))+" samples)\n")
  file.write("// Currents (A): "+np.array2string(currentSamples[0],precision=2,separator=', ',floatmode="fixed")[1:-2]+"\n")
  
  # C++ & LUT Setup
  file.write("\n#include \"voltage_lookup.h\"\n\n")
  file.write("static const DischargeDataPoint DISCHARGE_CAPACITY_LUT["+str(len(voltageSamples))+"]["+str(len(currentSamples[0]))+"] = {\n")

  # Generate sample points
  linearSample = griddata((voltagePts,currentPts),capacityValues,(voltageSamples,currentSamples), method='linear')
  nearestSample = griddata((voltagePts,currentPts),capacityValues,(voltageSamples,currentSamples), method='nearest')
  capacitySamples = np.nan_to_num(linearSample)
  np.putmask(capacitySamples,~np.ma.make_mask(capacitySamples),nearestSample)

  # Write to samples to file
  data = ""
  for voltageIndex in range(len(voltageSamples)):
    line = "  {"
    for currentIndex in range(len(currentSamples[0])):
      line += "{"+str(int(voltageSamples[voltageIndex][0]*1000))[:5]+","+str(int(currentSamples[0][currentIndex]*1000))[:5]+","+str(capacitySamples[voltageIndex][currentIndex])[:8]+"}, "
    data += line[:-2]+"},\n" 
  file.write(data[:-2]+"\n")

  # LUT Close
  file.write("};\n")