#ifndef SOC_EST_H
#define SOC_EST_H

#include <Python.h>

#define Q_NOM 1
#define PYTHONHOME L"C:/Users/emmas/AppData/Local/Programs/Python/Python313"
#define PYTHONPATH L"C:/Users/emmas/AppData/Local/Programs/Python/Python313/Lib"
#define PYTHONPOLARS L"C:/Users/emmas/AppData/Local/Programs/Python/Python313/Lib/site-packages"

uint8_t Coulomb_Counting(uint8_t t1, uint8_t t2, int curr1, int curr2);

void Read_Parquet(char *argv[], PyObject **pCurr, PyObject **pTime, int *timelen, int *currlen);

PyStatus Init_Python(const char* program_name);

float* Time_CArray(PyObject **pTime, int timelen); // put data in C array

float* Curr_CArray(PyObject **pCurr, int currlen);

#endif