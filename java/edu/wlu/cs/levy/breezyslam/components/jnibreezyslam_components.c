/*
 * jnibreezyslam_components.c Java Native Interface code for BreezySLAM components
 *
 * Copyright (C) 2014 Simon D. Levy
 *
 * This code is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this code.  If not, see <http:#www.gnu.org/licenses/>.
 */

#include "../jni_utils.h"

#include "Map.h" 
#include "Scan.h" 

#include <jni.h>
#include <stdlib.h>

#define MAXSTR 100

// Map methods -----------------------------------------------------------------------------------------

JNIEXPORT void JNICALL Java_edu_wlu_cs_levy_breezyslam_components_Map_init (JNIEnv *env, jobject thisobject, jint size_pixels, jdouble size_meters)
{
    map_t * map = (map_t *)malloc(sizeof(map_t));
    map_init(map, (int)size_pixels, (double)size_meters);
    ptr_to_obj(env, thisobject, map);
}

JNIEXPORT jstring JNICALL Java_edu_wlu_cs_levy_breezyslam_components_Map_toString (JNIEnv *env, jobject thisobject)
{
    map_t * map = cmap_from_jmap(env, thisobject);
   
    char str[MAXSTR];

    map_string(*map, str);

    return (*env)->NewStringUTF(env, str);
}

JNIEXPORT void JNICALL Java_edu_wlu_cs_levy_breezyslam_components_Map_get (JNIEnv *env, jobject thisobject, jbyteArray bytes)
{
    map_t * map = cmap_from_jmap(env, thisobject);

    jbyte * ptr = (*env)->GetByteArrayElements(env, bytes, NULL);

    map_get(map, ptr);

    (*env)->ReleaseByteArrayElements(env, bytes, ptr, 0);
}

JNIEXPORT void JNICALL Java_edu_wlu_cs_levy_breezyslam_components_Map_update (JNIEnv *env, jobject thisobject, 
            jobject scanobject, 
            jdouble position_x_mm,   
            jdouble position_y_mm,   
            jdouble position_theta_degrees,
            jint quality, 
            jdouble hole_width_mm)
{
    map_t * map = cmap_from_jmap(env, thisobject);

    scan_t * scan = cscan_from_jscan(env, scanobject);

    position_t position;

    position.x_mm = position_x_mm;
    position.y_mm = position_y_mm;
    position.theta_degrees = position_theta_degrees;

    map_update(map, scan, position, quality, hole_width_mm);
}


// Scan methods -----------------------------------------------------------------------------------------

JNIEXPORT void JNICALL Java_edu_wlu_cs_levy_breezyslam_components_Scan_init (JNIEnv *env, jobject thisobject, 
                            jint span,
                            jint scan_size,
                            jdouble scan_rate_hz,
                            jdouble detection_angle_degrees,
                            jdouble distance_no_detection_mm,
                            jint detection_margin,
                            jdouble offset_mm)
 {
    scan_t * scan = (scan_t *)malloc(sizeof(scan_t));

    scan_init(scan, 
           span,
           scan_size,
           scan_rate_hz,
           detection_angle_degrees,
           distance_no_detection_mm,
           detection_margin,
           offset_mm);

     ptr_to_obj(env, thisobject, scan);
}

JNIEXPORT jstring JNICALL Java_edu_wlu_cs_levy_breezyslam_components_Scan_toString (JNIEnv *env, jobject thisobject)
{
    scan_t * scan = cscan_from_jscan(env, thisobject);

    char str[MAXSTR];

    scan_string(*scan, str);

    return (*env)->NewStringUTF(env, str);
}

JNIEXPORT void JNICALL Java_edu_wlu_cs_levy_breezyslam_components_Scan_update (JNIEnv *env, jobject thisobject, 
                            jintArray lidar_mm,
                            jdouble hole_width_mm,
                            jdouble velocities_dxy_mm,
                            jdouble velocities_dtheta_degrees)
{
    scan_t * scan = cscan_from_jscan(env, thisobject);

    jint * lidar_mm_c = (*env)->GetIntArrayElements(env, lidar_mm, 0);

    // no support for angles/interpolation yet
    scan_update(scan, NULL, lidar_mm_c, scan->size, hole_width_mm, velocities_dxy_mm, velocities_dtheta_degrees);

    (*env)->ReleaseIntArrayElements(env, lidar_mm, lidar_mm_c, 0);
}
