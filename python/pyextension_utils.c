/*
pyextension_utils.c : C extension utilities for Python

Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Python.h>

static void raise_argument_exception(const char * classname, const char *  methodname, const char * details)
{
    char errmsg[1000];
    sprintf(errmsg, "Wrong/insufficient arguments passed to %s.%s()%s %s", 
        classname, methodname, 
        details ? ":" : "", details ? details : "");
    PyErr_SetString(PyExc_TypeError, errmsg);
    
}

PyObject * null_on_raise_argument_exception_with_details(const char * classname, const char *  methodname, const char * details)
{
    raise_argument_exception(classname, methodname, details);
    return NULL;
}

PyObject *null_on_raise_argument_exception(const char * classname, const char *  methodname)
{
    return null_on_raise_argument_exception_with_details(classname, methodname, NULL);
}

int error_on_raise_argument_exception_with_details(const char * classname, const char *  methodname, const char * details)
{
    raise_argument_exception(classname, methodname, details);
    return -1;
}

int error_on_raise_argument_exception(const char * classname)
{
    return error_on_raise_argument_exception_with_details(classname, "__init__", NULL);
}

int type_is_ready(PyTypeObject * type)
{
    return (PyType_Ready(type) >= 0);
}


void add_class(PyObject * module, PyTypeObject * type, const char * classname)
{
    Py_INCREF(type);
    PyModule_AddObject(module, classname, (PyObject *)type);
    
}

int double_from_tuple(PyObject * tup,   int pos,   double * val)
{
    PyObject * py_val = PyTuple_GetItem(tup, pos);
    
    if (py_val)
    {
        *val = PyFloat_AsDouble(py_val);
        return PyErr_Occurred() ? 0 : 1;         
    }
    
    return 0;
}


int
double_from_obj(
    PyObject * obj,
    const char * name,
    double * val)
{
    PyObject * attr = PyObject_GetAttrString(obj, name);
    
    if (!attr)
    {
        return 0;
    }
    
    *val = PyFloat_AsDouble(attr);
    
    return 1;
}


int
int_from_obj(
    PyObject * obj,
    const char * name,
    int * val)
{    
    PyObject * attr = PyObject_GetAttrString(obj, name);
    
    if (!attr)
    {
        return 0;
    }
    
    *val = PyLong_AsLong(attr);
    
    return 1;
}


int 
error_on_check_argument_type(
    PyObject * obj, 
    PyTypeObject * typ, 
    int pos,
    const char * typname, 
    const char * classname,
    const char *methodname)
{
    
    if (!PyObject_TypeCheck(obj, typ))
    {
        char details[200];
        sprintf(details, "argument %d is not of type %s", pos+1, typname);
        raise_argument_exception(classname, methodname, details);
        return -1;
    }
    
    return 0;
}
