#define PY_SSIZE_T_CLEAN
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "soc_est.h"

PyStatus Init_Python(const char *program_name) {

    PyStatus status;
    PyConfig config;
    PyConfig_InitPythonConfig(&config);

    status = PyConfig_SetBytesString(&config, &config.program_name, program_name); // set program name
    if (PyStatus_Exception(status)) {
        goto done;
    }

    status = PyConfig_SetString(&config, &config.home, PYTHONHOME);
    if (PyStatus_Exception(status)) {
        goto done;
    }

    // config.module_search_paths_set = 1; // search specified paths
    status = PyWideStringList_Append(&config.module_search_paths, PYTHONPATH); // search for stdlib
    if (PyStatus_Exception(status)) {
        goto done;
    }
    status = PyWideStringList_Append(&config.module_search_paths, PYTHONPOLARS); // search for polars
    if (PyStatus_Exception(status)) {
        goto done;
    }

    status = Py_InitializeFromConfig(&config);

done:
    PyConfig_Clear(&config);
    return status;

}

void Read_Parquet(char *argv[], PyObject **pCurr, PyObject **pTime, int *timelen, int *currlen) { // argv[1] = script name argv[2] = current function argv[3] = time function argv[4] = parquet file

    PyObject *pName, *pModule, *pCurrFunc, *pTimeFunc, *parquet;

    pName = PyUnicode_FromString(argv[1]); // construct string for input to import function
    pModule = PyImport_Import(pName); // load python script

    if (pModule != NULL) {

        pCurrFunc = PyObject_GetAttrString(pModule, argv[2]); // retrieve read current function from script
        parquet = Py_BuildValue("(s)", argv[4]);

        if (!pCurrFunc) {
            PyErr_Print();
            goto exception;
        }

        else if (pCurrFunc && PyCallable_Check(pCurrFunc)) { // check pCurrFunc is a function
            *pCurr = PyObject_CallObject(pCurrFunc, parquet); // call pCurrFunc 
        }

        pTimeFunc = PyObject_GetAttrString(pModule, argv[3]); // retrieve read time function

        if (!pTimeFunc) {
            PyErr_Print();
            goto exception;
        }

        else if (pTimeFunc && PyCallable_Check(pTimeFunc)) {
            *pTime = PyObject_CallObject(pTimeFunc, parquet);
        }

        *timelen = PyList_Size(*pTime);
        *currlen = PyList_Size(*pCurr);

    }

    else if (!pModule) {
        PyErr_Print();
        goto exception;
    }

exception:
    Py_XDECREF(pName); Py_XDECREF(pModule); Py_XDECREF(pCurrFunc); Py_XDECREF(pTimeFunc); Py_XDECREF(parquet);
    if (PyErr_Occurred()) {
        PyErr_Print();
    }

}

float* Time_CArray(PyObject **pTime, int timelen) {
    
    float *time = malloc(timelen * sizeof(float));

    for (int i=0; i<timelen; i++) {
        PyObject *row = PyList_GetItem(*pTime, i);
        PyObject *t = PyList_GetItem(row, 0);
        time[i] = (float) PyFloat_AsDouble(t);
    }

    return time;

}

float* Curr_CArray(PyObject **pCurr, int currlen) {
    
    float *current = malloc(currlen * sizeof(float));

    for (int i=0; i<currlen; i++) {
        PyObject *row = PyList_GetItem(*pCurr, i);
        PyObject *c = PyList_GetItem(row, 0);
        current[i] = (float) PyFloat_AsDouble(c);
    }

    return current;

}