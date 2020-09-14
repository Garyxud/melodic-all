/*
 * GPLv2 License. Modified from sigmask.c in qmtest.
 */

/******************************************************************
 *
 * File: sigblock.c
 * Author: Nathaniel Smith
 * Date: 2004-05-01
 *
 * Contents:
 *   Simple Python support for saving and restoring the signal mask.
 *
 * Copyright (c) 2004 by CodeSourcery, LLC.  All rights reserved.
 *
 * For license terms see the file COPYING.
 *
 ******************************************************************/

#include <Python.h>
#include <signal.h>

static PyObject* SigblockError;

static int the_mask_is_set;
static sigset_t the_mask;

static PyObject *
save_mask(PyObject* self, PyObject* args)
{
    /* We take no arguments. */
    if (!PyArg_ParseTuple(args, "")) return NULL;

    if (sigprocmask(SIG_BLOCK, NULL, &the_mask) == -1)
    {
        PyErr_SetString(SigblockError, "Error fetching mask");
        return NULL;
    }

    the_mask_is_set = 1;
    
    Py_INCREF(Py_None);
    return Py_None;
}


static PyObject *
restore_mask(PyObject* self, PyObject* args)
{
    /* We take no arguments. */
    if (!PyArg_ParseTuple(args, "")) return NULL;

    if (!the_mask_is_set)
    {
        PyErr_SetString(SigblockError,
                        "Must call save_mask before restore_mask");
        return NULL;
    }
    
    if (sigprocmask(SIG_SETMASK, &the_mask, NULL) == -1)
    {
        PyErr_SetString(SigblockError, "Error setting mask");
        return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}


static PyObject *
block_signal(PyObject* self, PyObject* args)
{
    int sig;
   
    /* We take no arguments. */
    if (!PyArg_ParseTuple(args, "i", &sig))
      return NULL;

    sigset_t mask;
    sigemptyset (&mask);
    sigaddset(&mask, sig);

    if (sigprocmask(SIG_BLOCK, &mask, NULL) == -1)
    {
        PyErr_SetString(SigblockError, "Error setting mask");
        return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}


static PyMethodDef module_methods[] = 
{
    {"save_mask", save_mask, METH_VARARGS,
     "Saves the current signal mask internally."},
    {"restore_mask", restore_mask, METH_VARARGS,
     "Sets the current signal mask to match that of the last call to save_mask."},
    {"block_signal", block_signal, METH_VARARGS,
     "Remove specified signal from the signal mask."},
    {NULL, NULL, 0, NULL}  /* Sentinel */
};


#ifndef PyMODINIT_FUNC  /* Declarations for DLL import/export */
#define PyMODINIT_FUNC void
#endif
PyMODINIT_FUNC
initsigblock(void)
{
    PyObject* m;
    
    m = Py_InitModule3("sigblock", module_methods,
                       "Module to save/restore signal mask.");

    the_mask_is_set = 0;

    SigblockError = PyErr_NewException("sigblock.SigblockError", NULL, NULL);
    PyModule_AddObject(m, "SigblockError", SigblockError);
}

