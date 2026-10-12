import polars as polars
import numpy as numpy;

def time(parquet_file) :
    timedf = polars.read_parquet(parquet_file, columns=["Time_ms"])
    return timedf.to_numpy().tolist()

def current(parquet_file) :
    currdf = polars.read_parquet(parquet_file, columns=["SME_TEMP_BusCurrent"])
    return currdf.to_numpy().tolist()