/*
pybreezyslam.c : C extensions for BreezySLAM in Python

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
#include <structmember.h>

#include "../c/coreslam.h"
#include "../c/random.h"
#include "pyextension_utils.h"

// Position class  -------------------------------------------------------------

typedef struct 
{
    PyObject_HEAD
    
    double x_mm;
    double y_mm;
    double theta_degrees;
    
} Position;

static void Position_dealloc(Position* self)
{                
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject * Position_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{                    
    Position *self;
    
    self = (Position *)type->tp_alloc(type, 0);
    
    return (PyObject *)self;
}

static int Position_init(Position *self, PyObject *args, PyObject *kwds)
{       
    if (!PyArg_ParseTuple(args,"ddd", 
        &self->x_mm, 
        &self->y_mm, 
        &self->theta_degrees))
    {
        return error_on_raise_argument_exception("Position");
    }
    
    return 0;    
}


static PyObject * Position_str(Position *self)
{        
    char str[100];
    
    sprintf(str, "Position: x = %7.0f mm  y = %7.0f mm  theta = %+04.0f degrees",
        self->x_mm, self->y_mm, self->theta_degrees);
    
    return  PyUnicode_FromString(str);
}

PyObject * Position_copy(Position *self, PyObject *args, PyObject *kwds);

static PyMethodDef Position_methods[] = 
{    
    {"copy", (PyCFunction)Position_copy, METH_VARARGS, 
    "Returns a mutable copy of this position."},
    {NULL}  // Sentinel 
};

static PyMemberDef Position_members[] = {
    {"x_mm", T_DOUBLE, offsetof(Position, x_mm), 0,
    "Distance of robot from left edge of map, in millimeters"},
    {"y_mm", T_DOUBLE, offsetof(Position, y_mm), 0,
    "Distance of robot from top edge of map, in millimeters"},
    {"theta_degrees", T_DOUBLE, offsetof(Position, theta_degrees), 0,
    "Clockwise rotation of robot with respect to three o'clock (east), in degrees"},
    {NULL}  /* Sentinel */
};

#define TP_DOC_POSITION \
"A class representing the position (pose; location and orientation) of a robot.\n" \
"See data descriptors for details.\n"\
"Position.__init__(x_mm, y_mm, theta_degrees)"

static PyTypeObject pybreezyslam_PositionType = 
{
    #if PY_MAJOR_VERSION < 3
    PyObject_HEAD_INIT(NULL)
    0,                                          // ob_size
    #else
    PyVarObject_HEAD_INIT(NULL, 0)
    #endif
    "pybreezyslam.Position",                  // tp_name
    sizeof(Position),                           // tp_basicsize
    0,                                          // tp_itemsize
    (destructor)Position_dealloc,               // tp_dealloc
    0,                                          // tp_print
    0,                                          // tp_getattr
    0,                                          // tp_setattr
    0,                                          // tp_compare
    (reprfunc)Position_str,                     // tp_repr
    0,                                          // tp_as_number
    0,                                          // tp_as_sequence
    0,                                          // tp_as_positionping
    0,                                          // tp_hash 
    0,                                          // tp_call
    (reprfunc)Position_str,                     // tp_str
    0,                                          // tp_getattro
    0,                                          // tp_setattro
    0,                                          // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,   // tp_flags
    TP_DOC_POSITION,                            // tp_doc 
    0,                                          // tp_traverse 
    0,                                          // tp_clear 
    0,                                          // tp_richcompare 
    0,                                          // tp_weaklistoffset 
    0,                                          // tp_iter 
    0,                                          // tp_iternext 
    Position_methods,         					// tp_methods 
    Position_members,          					// tp_members 
    0,                                          // tp_getset 
    0,                                          // tp_base 
    0,                                          // tp_dict 
    0,                                          // tp_descr_get 
    0,                                          // tp_descr_set 
    0,                                          // tp_dictoffset 
    (initproc)Position_init,                    // tp_init 
    0,                                          // tp_alloc 
    Position_new,                               // tp_new 
};


PyObject * Position_copy(Position *self, PyObject *args, PyObject *kwds)
{        
    PyObject * argList = Py_BuildValue("ddd", 
        self->x_mm,  self->y_mm, self->theta_degrees); 
    PyObject * new_position = 
    PyObject_CallObject((PyObject *) &pybreezyslam_PositionType, argList);
    Py_DECREF(argList);	
    
    return new_position;
}

static position_t pypos2cpos(Position * pypos)
{
    position_t  cpos;
    
    cpos.x_mm = pypos->x_mm; 
    cpos.y_mm = pypos->y_mm; 
    cpos.theta_degrees = pypos->theta_degrees; 
    
    return cpos;
}


// Scan class ------------------------------------------------------------

typedef struct 
{
    PyObject_HEAD
    
    scan_t scan;
    int   * lidar_distances_mm;
    float * lidar_angles_deg;
    
} Scan;


static void
Scan_dealloc(Scan* self)
{        
    scan_free(&self->scan);
    
    free(self->lidar_distances_mm);
    free(self->lidar_angles_deg);
    
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
Scan_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{    
    Scan *self;
    
    self = (Scan *)type->tp_alloc(type, 0);
    
    return (PyObject *)self;
}

static int
Scan_init(Scan *self, PyObject *args, PyObject *kwds)
{                 
    PyObject * py_laser = NULL;
    int span = 1;
    
    static char* argnames[] = {"laser", "span", NULL};

    int scan_size = 0;
    double scan_rate_hz = 0;                
    double detection_angle_degrees = 0;     
    double distance_no_detection_mm = 0;
    int detection_margin = 0;
    double offset_mm = 0;

    if(!PyArg_ParseTupleAndKeywords(args, kwds,"O|i", argnames, 
        &py_laser, 
        &span))
    {
         return error_on_raise_argument_exception("Scan");
    }
    
    if (!int_from_obj(py_laser, "scan_size", &scan_size) ||
        !double_from_obj(py_laser, "scan_rate_hz", &scan_rate_hz) ||
        !double_from_obj(py_laser, "detection_angle_degrees", &detection_angle_degrees) ||
        !double_from_obj(py_laser, "distance_no_detection_mm", &distance_no_detection_mm) ||
        !int_from_obj(py_laser, "detection_margin", &detection_margin) ||
        !double_from_obj(py_laser, "offset_mm", &offset_mm))
    {
         return error_on_raise_argument_exception("Scan");
    }
      
    scan_init(
            &self->scan, 
            span,
            scan_size, 
            scan_rate_hz,                
            detection_angle_degrees,     
            distance_no_detection_mm,    
            detection_margin,               
            offset_mm);
 
    self->lidar_distances_mm = int_alloc(self->scan.size);
    self->lidar_angles_deg   = float_alloc(self->scan.size);
    
    return 0;
}


static PyObject *
Scan_str(Scan *self)
{        
    char scanstr[100];
    scan_string(self->scan, scanstr);
    
    char str[200];
    sprintf(str, "Scan: %s", scanstr);
    
    return  PyUnicode_FromString(str);
}


static PyObject *
Scan_update(Scan *self, PyObject *args, PyObject *kwds)
{
    PyObject * py_lidar = NULL;
    double hole_width_mm = 0;
    PyObject * py_velocities = NULL;
    PyObject * py_scan_angles_degrees = NULL;

    static char* argnames[] = {"scans_mm", "hole_width_mm", "velocities", "scan_angles_degrees", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds,"Od|OO", argnames,
        &py_lidar, 
        &hole_width_mm,
        &py_velocities,
        &py_scan_angles_degrees))
    {
        return null_on_raise_argument_exception("Scan", "update");
    }

    // Bozo filter on LIDAR argument
    if (!PyList_Check(py_lidar))
    {
        return null_on_raise_argument_exception_with_details("Scan", "update", 
            "lidar must be a list");
    }

    // Scan angles provided
    if (py_scan_angles_degrees != Py_None) 
    {
        // Bozo filter #1: SCAN_ANGLES_DEGREES  must be a list
        if (!PyList_Check(py_scan_angles_degrees))
        {
            return null_on_raise_argument_exception_with_details("Scan", "update", 
                    "scan angles must be a list");
        }

        // Bozo filter #2: must have same number of scan angles as scan distances
        if (PyList_Size(py_lidar) != PyList_Size(py_scan_angles_degrees))
        {
            return null_on_raise_argument_exception_with_details("Scan", "update", 
                    "number of scan angles must equal number of scan distances");
        }

        // Extract scan angle values from argument
        for (int k=0; k<PyList_Size(py_scan_angles_degrees); ++k)
        {
            self->lidar_angles_deg[k] = (float)PyFloat_AsDouble(PyList_GetItem(py_scan_angles_degrees, k));
        }
    }

    // No scan angles provided; lidar-list size must match scan size
    else if (PyList_Size(py_lidar) != self->scan.size)
    {        
        return null_on_raise_argument_exception_with_details("Scan", "update", 
                "lidar size mismatch");
    }

    // Default to no velocities
    double dxy_mm = 0;
    double dtheta_degrees = 0;

    // Bozo filter on velocities tuple
    if (py_velocities != Py_None)
    {
        if (!PyTuple_Check(py_velocities))
        {
            return null_on_raise_argument_exception_with_details("Scan", "update", 
                    "velocities must be a tuple");    
        }

        if (!double_from_tuple(py_velocities, 0, &dxy_mm) ||
                !double_from_tuple(py_velocities, 1, &dtheta_degrees))
        {
            return null_on_raise_argument_exception_with_details("Scan", "update", 
                    "velocities tuple must contain at least two numbers");    

        }
    }

    // Extract LIDAR values from argument
    for (int k=0; k<PyList_Size(py_lidar); ++k)
    {
        self->lidar_distances_mm[k] = (int)PyFloat_AsDouble(PyList_GetItem(py_lidar, k));
    }

    // Update the scan
    scan_update(
            &self->scan, 
            (py_scan_angles_degrees != Py_None) ? self->lidar_angles_deg :NULL,
            self->lidar_distances_mm, 
            PyList_Size(py_lidar),
            hole_width_mm,
            dxy_mm,
            dtheta_degrees);

    Py_RETURN_NONE;

} // Scan_update


static PyMethodDef Scan_methods[] = 
{
    {"update", (PyCFunction)Scan_update, METH_VARARGS | METH_KEYWORDS, 
        "Scan.update(scans_mm, hole_width_mm, velocities=None) updates scan.\n"\
            "scans_mm is a list of integers representing scanned distances in mm.\n"\
            "hole_width_mm is the width of holes (obstacles, walls) in millimeters.\n"\
            "velocities is an optional tuple containing (dxy_mm/dt, dtheta_degrees/dt);\n"\
            "i.e., robot's (forward, rotational velocity) for improving the quality of the scan."
    },
    {NULL}  // Sentinel 
};

#define TP_DOC_SCAN \
    "A class for Lidar scans.\n" \
"Scan.__init__(laser, span=1)\n"\
"laser is a Laser object containing parameters of your laser rangefinder (Lidar)\n"\
"    span supports spanning laser scan to cover the space better"


static PyTypeObject pybreezyslam_ScanType = 
{
#if PY_MAJOR_VERSION < 3
    PyObject_HEAD_INIT(NULL)
        0,                                          // ob_size
#else
    PyVarObject_HEAD_INIT(NULL, 0)
#endif
        "pybreezyslam.Scan",                      // tp_name
    sizeof(Scan),                               // tp_basicsize
    0,                                          // tp_itemsize
    (destructor)Scan_dealloc,                   // tp_dealloc
    0,                                          // tp_print
    0,                                          // tp_getattr
    0,                                          // tp_setattr
    0,                                          // tp_compare
    (reprfunc)Scan_str,                         // tp_repr
    0,                                          // tp_as_number
    0,                                          // tp_as_sequence
    0,                                          // tp_as_positionping
    0,                                          // tp_hash 
    0,                                          // tp_call
    (reprfunc)Scan_str,                         // tp_str
    0,                                          // tp_getattro
    0,                                          // tp_setattro
    0,                                          // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,   // tp_flags
    TP_DOC_SCAN,                                // tp_doc 
    0,                                          // tp_traverse 
    0,                                          // tp_clear 
    0,                                          // tp_richcompare 
    0,                                          // tp_weaklistoffset 
    0,                                          // tp_iter 
    0,                                          // tp_iternext 
    Scan_methods,                         		// tp_methods 
    0,                         					// tp_members 
    0,                                          // tp_getset 
    0,                                          // tp_base 
    0,                                          // tp_dict 
    0,                                          // tp_descr_get 
    0,                                          // tp_descr_set 
    0,                                          // tp_dictoffset 
    (initproc)Scan_init,                        // tp_init 
    0,                                          // tp_alloc 
    Scan_new,                                   // tp_new 
};


// Map class ------------------------------------------------------------

typedef struct 
{
    PyObject_HEAD
    
    map_t map;
    
} Map;

// Helper for Map.__init__(), Map.set()
static int bad_mapbytes(PyObject * py_mapbytes, int size_pixels, const char * methodname)
{    
    if (!PyByteArray_Check(py_mapbytes))
    {
        return error_on_raise_argument_exception_with_details("Map", methodname, 
            "argument is not a byte array");        
    }
    
    if (PyByteArray_GET_SIZE(py_mapbytes) != (size_pixels * size_pixels))
    {        
        return error_on_raise_argument_exception_with_details("Map", methodname, 
            "mapbytes are wrong size");
    }

    return 0;
}

static void
Map_dealloc(Map* self)
{            
    map_free(&self->map);
    
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
Map_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{    
    Map *self;
    
    self = (Map *)type->tp_alloc(type, 0);
    
    return (PyObject *)self;
}

static int
Map_init(Map *self, PyObject *args, PyObject *kwds)
{                    
	int size_pixels;
	double size_meters;
	PyObject * py_bytes = NULL;
	
    static char * argnames[] = {"size_pixels", "size_meters", "bytes", NULL};

    if(!PyArg_ParseTupleAndKeywords(args, kwds,"id|O", argnames, 
        &size_pixels, 
        &size_meters, 
        &py_bytes))
    {
        return error_on_raise_argument_exception("Map");
    }
           
    map_init(&self->map, size_pixels, size_meters);
    
    if (py_bytes && !bad_mapbytes(py_bytes, size_pixels, "__init__"))
    {    
        map_set(&self->map, PyByteArray_AsString(py_bytes));
    }
    
    return 0;
}

static PyObject *
Map_str(Map * self)
{            
    char mapstr[100];
    map_string(self->map, mapstr);
    
    char str[200];
    sprintf(str, "Map: %s", mapstr);
    
    return  PyUnicode_FromString(str);
}

static PyObject *
Map_get(Map * self, PyObject * args, PyObject * kwds)
{        
    PyObject * py_mapbytes = NULL;

    if (!PyArg_ParseTuple(args, "O", &py_mapbytes))
    {
        return null_on_raise_argument_exception("Map", "get");
    }
    
    if (bad_mapbytes(py_mapbytes, self->map.size_pixels, "get"))
    {
        Py_RETURN_NONE;
    }
    
    map_get(&self->map, PyByteArray_AsString(py_mapbytes));
    
    Py_RETURN_NONE;
}

static PyObject *
Map_set(Map * self, PyObject * args, PyObject * kwds)
{        
    PyObject * py_mapbytes = NULL;

    if (!PyArg_ParseTuple(args, "O", &py_mapbytes))
    {
        return null_on_raise_argument_exception("Map", "get");
    }
    
    if (bad_mapbytes(py_mapbytes, self->map.size_pixels, "get"))
    {
        Py_RETURN_NONE;
    }
    
    map_set(&self->map, PyByteArray_AsString(py_mapbytes));
    
    Py_RETURN_NONE;
}

static PyObject *
Map_update(Map *self, PyObject *args, PyObject *kwds)
{   
    Scan * py_scan = NULL;
    Position * py_position = NULL;
    int map_quality = 0;
    double hole_width_mm = 0;
	
    if (!PyArg_ParseTuple(args, "OOid",
        &py_scan,
        &py_position,
        &map_quality,
        &hole_width_mm))
    {
        return null_on_raise_argument_exception("Map", "update");
    }
         
    if (error_on_check_argument_type((PyObject *)py_scan, &pybreezyslam_ScanType, 0,
            "pybreezyslam.Scan", "Map", "update") ||
        error_on_check_argument_type((PyObject *)py_position, &pybreezyslam_PositionType, 0,
            "pybreezyslam.Position", "Map", "update"))
    {
        return NULL;
    }
            
    position_t position = pypos2cpos(py_position);
    
    map_update(
        &self->map, 
        &py_scan->scan, 
        position,
        map_quality, 
        hole_width_mm);

    Py_RETURN_NONE;
}

static PyMethodDef Map_methods[] = 
{
    {"update", (PyCFunction)Map_update, METH_VARARGS, 
    "Map.update(Scan, Position, quality, hole_width_mm) updates map based on scan and position.\n"\
    "Quality from 0 through 255 determines integration speed of scan into map.\n"\
    "Hole width determines width of obstacles (walls)."
    },
    {"get", (PyCFunction)Map_get, METH_VARARGS,
    "Map.get(bytearray) fills byte array with map pixels, where bytearray length is square of size of map."
    },
    {"set", (PyCFunction)Map_set, METH_VARARGS,
    "Map.set(bytearray) fills current map with pixels in bytearray, where bytearray length is square of size of map."
    },
    {NULL}  // Sentinel 
};

#define TP_DOC_MAP \
"A class for maps used in SLAM.\n"\
"Map.__init__(size_pixels, size_meters, bytes=None)"


static PyTypeObject pybreezyslam_MapType = 
{
    #if PY_MAJOR_VERSION < 3
    PyObject_HEAD_INIT(NULL)
    0,                                          // ob_size
    #else
    PyVarObject_HEAD_INIT(NULL, 0)
    #endif
    "pybreezyslam.Map",                       // tp_name
    sizeof(Map),                                // tp_basicsize
    0,                                          // tp_itemsize
    (destructor)Map_dealloc,                    // tp_dealloc
    0,                                          // tp_print
    0,                                          // tp_getattr
    0,                                          // tp_setattr
    0,                                          // tp_compare
    (reprfunc)Map_str,                          // tp_repr
    0,                                          // tp_as_number
    0,                                          // tp_as_sequence
    0,                                          // tp_as_positionping
    0,                                          // tp_hash 
    0,                                          // tp_call
    (reprfunc)Map_str,                          // tp_str
    0,                                          // tp_getattro
    0,                                          // tp_setattro
    0,                                          // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,   // tp_flags
    TP_DOC_MAP,                                 // tp_doc 
    0,                                          // tp_traverse 
    0,                                          // tp_clear 
    0,                                          // tp_richcompare 
    0,                                          // tp_weaklistoffset 
    0,                                          // tp_iter 
    0,                                          // tp_iternext 
    Map_methods,                         		// tp_methods 
    0,                         					// tp_members 
    0,                                          // tp_getset 
    0,                                          // tp_base 
    0,                                          // tp_dict 
    0,                                          // tp_descr_get 
    0,                                          // tp_descr_set 
    0,                                          // tp_dictoffset 
    (initproc)Map_init,                         // tp_init 
    0,                                          // tp_alloc 
    Map_new,                                    // tp_new 
};

// Randomizer class ------------------------------------------------------------

typedef struct 
{
    PyObject_HEAD
    
    void * randomizer;
    
} Randomizer;


static void
Randomizer_dealloc(Randomizer* self)
{            
    random_free(self->randomizer);
    
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
Randomizer_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{    
    Randomizer *self;
    
    self = (Randomizer *)type->tp_alloc(type, 0);
    
    return (PyObject *)self;
}

static int
Randomizer_init(Randomizer *self, PyObject *args, PyObject *kwds)
{                    
	int seed;
	
    if (!PyArg_ParseTuple(args, "i", &seed))
    {
        return error_on_raise_argument_exception("Randomizer");
    }
    
    self->randomizer = random_new(seed);
    
    return 0;
}

#define TP_DOC_RANDOMIZER \
""

static PyTypeObject pybreezyslam_RandomizerType = 
{
    #if PY_MAJOR_VERSION < 3
    PyObject_HEAD_INIT(NULL)
    0,                                          // ob_size
    #else
    PyVarObject_HEAD_INIT(NULL, 0)
    #endif
    "pybreezyslam.Randomizer",                // tp_name
    sizeof(Randomizer),                         // tp_basicsize
    0,                                          // tp_itemsize
    (destructor)Randomizer_dealloc,             // tp_dealloc
    0,                                          // tp_print
    0,                                          // tp_getattr
    0,                                          // tp_setattr
    0,                                          // tp_compare
    0,                                          // tp_repr
    0,                                          // tp_as_number
    0,                                          // tp_as_sequence
    0,                                          // tp_as_positionping
    0,                                          // tp_hash 
    0,                                          // tp_call
    0,                                          // tp_str
    0,                                          // tp_getattro
    0,                                          // tp_setattro
    0,                                          // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,   // tp_flags
    TP_DOC_RANDOMIZER,                          // tp_doc 
    0,                                          // tp_traverse 
    0,                                          // tp_clear 
    0,                                          // tp_richcompare 
    0,                                          // tp_weaklistoffset 
    0,                                          // tp_iter 
    0,                                          // tp_iternext 
    0,                         					// tp_methods 
    0,                         					// tp_members 
    0,                                          // tp_getset 
    0,                                          // tp_base 
    0,                                          // tp_dict 
    0,                                          // tp_descr_get 
    0,                                          // tp_descr_set 
    0,                                          // tp_dictoffset 
    (initproc)Randomizer_init,                  // tp_init 
    0,                                          // tp_alloc 
    Randomizer_new,                             // tp_new 
};


// pybreezyslam module ------------------------------------------------------------


static PyObject *
distanceScanToMap(PyObject *self, PyObject *args)
{   
    Map * py_map = NULL;
    Scan * py_scan = NULL;
    Position * py_position = NULL;
    
    // Extract Python objects for map, scan, and position
    if (!PyArg_ParseTuple(args, "OOO", &py_map, &py_scan, &py_position))
    {        
        return null_on_raise_argument_exception("breezyslam", "distanceScanToMap");
    }
    
    // Check object types
    if (error_on_check_argument_type((PyObject *)py_map, &pybreezyslam_MapType, 0,
            "pybreezyslam.Map", "pybreezyslam", "distanceScanToMap") ||
        error_on_check_argument_type((PyObject *)py_scan, &pybreezyslam_ScanType, 1,
            "pybreezyslam.Scan", "pybreezyslam", "distanceScanToMap") ||
        error_on_check_argument_type((PyObject *)py_position, &pybreezyslam_PositionType, 2, 
            "pybreezyslam.Position", "pybreezyslam", "distanceScanToMap"))
    {
            return NULL;
    }
    
        
    // Translate position object from Python to C
    position_t c_position = pypos2cpos(py_position);
    
    // Run C version and return Python integer
    return PyLong_FromLong(distance_scan_to_map(&py_map->map, &py_scan->scan, c_position));
}

// Called internally, so minimal type-checking on arguments
static PyObject *
rmhcPositionSearch(PyObject *self, PyObject *args)
{   	    
    Position * py_start_pos = NULL;
	Map * py_map = NULL;
    Scan * py_scan = NULL;
    PyObject * py_laser = NULL;
	double sigma_xy_mm = 0;
	double sigma_theta_degrees = 0;
	int max_search_iter = 0;
	Randomizer * py_randomizer = NULL;
	
    // Extract Python objects for map, scan, and position
    if (!PyArg_ParseTuple(args, "OOOOddiO", 
        &py_start_pos,
        &py_map,
        &py_scan,
        &py_laser,
        &sigma_xy_mm,
        &sigma_theta_degrees,
        &max_search_iter,
        &py_randomizer))
    {        
        return null_on_raise_argument_exception("breezyslam.algorithms", "rmhcPositionSearch");
    }
    
    // Convert Python objects to C structures
    position_t start_pos = pypos2cpos(py_start_pos);

	position_t likeliest_position = 
    rmhc_position_search(
        start_pos,
        &py_map->map,
        &py_scan->scan,
        sigma_xy_mm,
        sigma_theta_degrees,
        max_search_iter,
        py_randomizer->randomizer);    
    
    
    // Convert C position back to Python object
    PyObject * argList = Py_BuildValue("ddd", 
        likeliest_position.x_mm, 
        likeliest_position.y_mm, 
        likeliest_position.theta_degrees); 
    PyObject * py_likeliest_position = 
    PyObject_CallObject((PyObject *) &pybreezyslam_PositionType, argList);
    Py_DECREF(argList);	
    
    return py_likeliest_position;
    
}


static PyMethodDef module_methods[] = 
{
    {"distanceScanToMap", distanceScanToMap, METH_VARARGS,
        "distanceScanToMap(map, scan, position)\n"
    "Computes distance between a scan and map, given hypothetical position, to support particle filtering.\n"\
    "Returns -1 for infinity.\n"\
    "map is a breezyslam.components.Map object\n"\
    "scan is a breezyslam.components.Scan object\n"\
    "position is a breezyslam.components.Position object\n"\
    },
    {"rmhcPositionSearch", rmhcPositionSearch, METH_VARARGS,
        "rmhcPositionSearch(startpos, map, scan, laser, sigma_xy_mm, max_iter, randomizer)\n"
    "Internal use only."
    },
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static void add_classes(PyObject * module)
{
    add_class(module, &pybreezyslam_ScanType, "Scan");
    add_class(module, &pybreezyslam_MapType, "Map");
    add_class(module, &pybreezyslam_PositionType, "Position");
    add_class(module, &pybreezyslam_RandomizerType, "Randomizer");
}

static int types_are_ready(void)
{
return 
    type_is_ready(&pybreezyslam_ScanType) &&
    type_is_ready(&pybreezyslam_MapType) &&
    type_is_ready(&pybreezyslam_PositionType) &&
    type_is_ready(&pybreezyslam_RandomizerType);
}

#if PY_MAJOR_VERSION < 3

PyMODINIT_FUNC
initpybreezyslam(void) 
{    
    if (!types_are_ready())    {
        return;
    }
    
    PyObject * module = Py_InitModule("pybreezyslam", module_methods);
       
    if (module == NULL)
    {
        return;
    }

    add_classes(module);    
}

#else

static PyModuleDef moduledef = 
{
    PyModuleDef_HEAD_INIT,
    "pybreezyslam",
    "BreezySLAM module",  
    -1,                 // m_size
    module_methods,     
    NULL,               // m_reload
    NULL,               // m_traverse
    NULL,               // m_clear
    NULL                // m_free
};

PyMODINIT_FUNC
PyInit_pybreezyslam(void) 
{    
    if (!types_are_ready())
    {
        return NULL;
    }
    
    PyObject* module = PyModule_Create(&moduledef);
    
    if (module == NULL)
    {
        return NULL;
    }
    
    add_classes(module);
    
    return module;
}

#endif

