import os
import polars as pl 
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import host_subplot

SHOW_PLOTS = False
DATA_PROFILE = 2
# 0 = FS3 Battery, low end amperages
# 1 = FS4 Battery, low end amperages
# 2 = FS3 Battery, all amperages
# 3 = FS4 Battery, all amperages

CURRENT_LABLELS = [["10A", "5A", "2.5A"],
                   ["10A", "1C", "0.5C", "0.2C"],
                   ["35A", "30A", "20A", "15A", "10A", "5A", "2.5A"],
                   ["30A", "20A", "10A", "1C", "0.5C", "0.2C"]]
CURRENT_VALUES = [[10,5,2.5],
                   [10,3,1.5,.6],
                   [35,30,20,15,10,5,2.5],
                   [30,20,10,3,1.5,.6]]

# Reading Battery Discharge Dataframe
DISCHARGE_FILE = "data_sources/US18650VTC5A-discharge-data.parquet" if (DATA_PROFILE == 0 or DATA_PROFILE == 2) else "data_sources/p30b-discharge.parquet"
dischargeParquet = DISCHARGE_FILE if SHOW_PLOTS else "tools/"+DISCHARGE_FILE
dischargeData = pl.read_parquet(os.path.join(os.getcwd(), dischargeParquet))

# Pull Voltage Data
voltagePts = np.empty(0)
for load in CURRENT_LABLELS[DATA_PROFILE]:
  voltagePts = np.hstack((voltagePts, dischargeData[load].to_numpy()))

# Pull Current Data
currentPts = np.empty(0)
for load in CURRENT_VALUES[DATA_PROFILE]:
  currentPts = np.hstack((currentPts, np.ones(100)*load))

# Pull Discharge Capacity Data
colCount = len(CURRENT_LABLELS[DATA_PROFILE])
capacityValues = np.empty(0)
for i in range(colCount):
  capacityValues = np.hstack((capacityValues, dischargeData["Capacity"].to_numpy()))
  
# Clean data
voltagePts = np.nan_to_num(voltagePts)
invalidValuesMask = np.ones(colCount*100, dtype=bool)
invalidValuesMask[voltagePts==0] = 0
voltagePts = voltagePts[invalidValuesMask]
currentPts = currentPts[invalidValuesMask]
capacityValues = capacityValues[invalidValuesMask]

VOTLAGE_MIN = 2.5
VOLTAGE_MAX = 4.25
CURRENT_MAX = 25
GRID_SIZE = 25

# Create 2D array of independent voltage against current
voltageCurrent = np.vstack((voltagePts,currentPts)).T

# Generate values to interpolate on
gridPoints = np.mgrid[VOTLAGE_MIN:VOLTAGE_MAX:(GRID_SIZE*1j), 0:CURRENT_MAX:(GRID_SIZE*1j)].reshape(2,-1).T

# Linear interpolate
interpolatedData = interpolate.LinearNDInterpolator(voltageCurrent,capacityValues)(gridPoints)

# Clean interpolated data
interpolatedData = np.nan_to_num(interpolatedData)
invalidValuesMask = np.ones(GRID_SIZE*GRID_SIZE, dtype=bool)
invalidValuesMask[interpolatedData == 0] = 0
maskedInterpolatedData = interpolatedData[invalidValuesMask]
gridMask = np.vstack((invalidValuesMask,invalidValuesMask)).T
maskedGrid = gridPoints[gridMask].reshape(-1,2)

# Add 2d array of interpolated points
interpolatedVoltageCurrent = np.vstack((voltageCurrent,maskedGrid))
interpolatedCapacityValues = np.hstack((capacityValues,maskedInterpolatedData))

if(SHOW_PLOTS):
  # Extrapolate based on interpolated values
  extrapolatedData = interpolate.RBFInterpolator(interpolatedVoltageCurrent,interpolatedCapacityValues,smoothing=1,kernel="quintic")(gridPoints)

  # Merge interpolation and extrapolation
  interpolatedData = interpolatedData.reshape(GRID_SIZE,GRID_SIZE)
  extrapolatedData[invalidValuesMask] = 0
  extrapolatedData = extrapolatedData.reshape(GRID_SIZE,GRID_SIZE)
  allData = interpolatedData + extrapolatedData

  # Plot interpolated Data
  interpDischargeFig, interpDischargeplot = plt.subplots()
  colorbar = interpDischargeplot.imshow(allData.T,extent=(VOTLAGE_MIN,VOLTAGE_MAX,0,CURRENT_MAX),aspect="auto",origin="lower",vmin=0,vmax=3000)
  interpDischargeplot.set_title("Voltage & Current - Discharge")
  interpDischargeplot.set_xlabel("Voltage (V)")
  interpDischargeplot.set_ylabel("Current (A)")
  interpDischargeplot.set_ylim((0,CURRENT_MAX))
  interpDischargeFig.colorbar(colorbar)
  interpDischargeFig.show()

  # Plot discharge graph datapoints
  interpDischargeplot.scatter(voltagePts,currentPts,c=capacityValues,cmap=colorbar.cmap, vmax=3000, vmin=0,edgecolors='black',linewidths=0.25)

else:
  VOTLAGE_COUNT = 40
  CURRENT_COUNT = 40
  voltageSamples, currentSamples = np.mgrid[VOTLAGE_MIN:VOLTAGE_MAX:VOTLAGE_COUNT*1j, 0:CURRENT_MAX:CURRENT_COUNT*1j]
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
  file.write("// Currents (A): "+str(currentSamples[0][0])+" to "+str(currentSamples[0][-1])[:4]+" ("+str(len(currentSamples[0]))+" samples)\n")
  
  # C++ & LUT Setup
  file.write("\n#include \"voltage_lookup.h\"\n\n")
  file.write("static const int LUT_VOLTAGE_ENTRIES = "+str(VOTLAGE_COUNT)+";\n")
  file.write("static const int LUT_AMPERAGE_ENTRIES = "+str(CURRENT_COUNT)+";\n\n")
  file.write("static const DischargeDataPoint DISCHARGE_CAPACITY_LUT["+str(len(voltageSamples))+"]["+str(len(currentSamples[0]))+"] = {\n")

  # Generate sample points
  allSamplePoints = np.stack((voltageSamples,currentSamples),2).reshape(-1,2)
  interpolatedSample = interpolate.LinearNDInterpolator(voltageCurrent,capacityValues)(allSamplePoints)
  extrapolatedSample = interpolate.RBFInterpolator(interpolatedVoltageCurrent,interpolatedCapacityValues,smoothing=.1,kernel="quintic")(allSamplePoints)
  capacitySamples = np.nan_to_num(interpolatedSample)
  np.putmask(capacitySamples,~np.ma.make_mask(capacitySamples),extrapolatedSample)

  # Clean Up
  capacitySamples = capacitySamples.reshape(VOTLAGE_COUNT,CURRENT_COUNT)
  capacitySamples[capacitySamples < 0] = 0

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