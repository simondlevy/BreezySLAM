/*
pyextension_utils.h : header for Python C extension utilities

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


/**
* Raises a bad-argument exception and returns NULL
* @param classname name of class in which exception happened
* @param methodname name of method in which exception happened
* @param details details of exception
* @return NULL
*/

PyObject *
null_on_raise_argument_exception_with_details(
    const char * classname, 
    const char * methodname, 
    const char * details);

/**
* Raises a bad-argument exception and returns NULL
* @param classname name of class in which exception happened
* @param methodname name of method in which exception happened
* @return NULL
*/
PyObject *
null_on_raise_argument_exception(
    const char * classname, 
    const char *  methodname);

/**
* Raises a bad-argument exception and returns -1
* @param classname name of class in which exception happened
* @param methodname name of method in which exception happened
* @param details details of exception
* @return -1
*/
int
error_on_raise_argument_exception_with_details(
    const char * classname, 
    const char *  methodname, 
    const char * details);

/**
* Raises a bad-argument exception and returns -1 on __init__.
* @param classname name of class in which exception happened
* @param methodname name of method in which exception happened
* @return -1
*/
int
error_on_raise_argument_exception(
    const char * classname);

/**
* Returns 1 if type is ready, 0 otherwise
* @param type the type object
* @return 1 if type is ready, 0 otherwise
*/
int 
type_is_ready(
    PyTypeObject * type);

/**
* Adds a class to a module.
* @param module the module
* @param type the type object
* @param name the class name
*/
void 
add_class(
    PyObject * module, 
    PyTypeObject * type, 
    const char * classname);
    
 /**
 * Extracts a double from a tuple.
 * @param tup the tuple
 * @param pos position in the tuple
 * @param val gets the returned value
 * @return true on sucess, false on failure
 */
 int
 double_from_tuple(
   PyObject * tup,
   int pos,
   double * val);
 
/**
 * Extracts a named double from an object.
 * @param obj the obj
 * @param name the name
 * @param val gets the returned value
 * @return true on sucess, false on failure
 */
 int
 double_from_obj(
   PyObject * obj,
   const char * name,
   double * val);
 
/**
 * Extracts a named int from an object.
 * @param obj the obj
 * @param name the name
 * @param val gets the returned value
 * @return true on sucess, false on failure
 */
 int
int_from_obj(
   PyObject * obj,
   const char * name,
   int * val); 

   
/**
* Raises a bad-argument exception and returns -1 on passing of incorrent argument type
* @param classname name of class in which exception happened
* @param methodname name of method in which exception happened
* @param typname name of expected type
* @return -1 on error; 0 on success
*/
int 
error_on_check_argument_type(
    PyObject * obj, 
    PyTypeObject * typ, 
    int pos,
    const char * typname, 
    const char * classname,
    const char *methodname);


