import polars as polars
import numpy as np
import scipy.io
from scipy.integrate import cumulative_simpson
import pandas as pd

Q_NOM = 3000 # mAh

# SOC = Q_releasable/Q_nom
# DOD = Q_released/Q_nom
# SOH = Q_releasable + Q_released = Q_total

# Load Matlab data

matData = scipy.io.loadmat("Molicel_INR18650P30B_measurement.mat", squeeze_me = True, struct_as_record=False)
measurements = matData["measurement"]

dcc = measurements.fu.DCC # fields are name, T_amb, t, I, V, T_surf
chc = measurements.fu.CHC
dcp = measurements.fu.DCP
chp = measurements.fu.CHP
pro = measurements.fu.PRO

# Calculate SOC using Coulomb counting

def Calculate_SOC(I, t, starting_SOH):

    q_io = cumulative_simpson(y=I*1000, x=t, initial=0)/3600 # mAh
    soc = starting_SOH + q_io/Q_NOM # SOC = SOH - DOD
    return np.clip(soc, 0.0, 1.0)

# Calculate OCV

def OCV_terminal_voltage(SOC, T, a0, a1, a2, a3, a4, a5, a6, K_T):
    return (a0 + a1 * SOC + a2 * SOC**2 + a3 * SOC**3 + a4 * SOC**4 + a5 * SOC**5 + a6 * SOC**6) * (1 + K_T * (T - 25))

# Save data into parquet

data = {

    "Current": dcc[0].I,
    "SOC": Calculate_SOC(dcc[0].I, dcc[0].t, 1),
    # OCV

}

df = polars.DataFrame(data)
df.write_parquet("DCC0.parquet")