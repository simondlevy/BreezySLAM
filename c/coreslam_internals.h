/*
coreslam_internals.h internal support for CoreSLAM

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
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
*/


#ifdef _MSC_VER
typedef __int64 int64_t;       /* Define it from MSVC's internal type */
#define _USE_MATH_DEFINES
#include <math.h>
#else
#include <stdint.h>            /* Use the C99 official header */
#endif


static const int NO_OBSTACLE            = 65500;
static const int OBSTACLE               = 0;

static double 
radians(double degrees)
{
    return degrees * M_PI / 180;
}
