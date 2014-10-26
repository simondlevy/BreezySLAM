#include "../../../../../../../c/random.h"
#include "../jni_utils.h"

#include <jni.h>


// RMHC_SLAM methods -----------------------------------------------------------------------------------------

JNIEXPORT void JNICALL Java_edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_init (JNIEnv *env, jobject thisobject, jint random_seed)
{
    ptr_to_obj(env, thisobject, random_new(random_seed));
}

JNIEXPORT jobject JNICALL Java_edu_wlu_cs_levy_breezyslam_algorithms_RMHCSLAM_positionSearch (JNIEnv *env, jobject thisobject, 
        jobject startpos_object,
        jobject map_object,
        jobject scan_object,
        jdouble sigma_xy_mm,
        jdouble sigma_theta_degrees,
        jint max_search_iter)
{
    position_t startpos;

    startpos.x_mm          = get_double_field(env, startpos_object, "x_mm");
    startpos.y_mm          = get_double_field(env, startpos_object, "y_mm");
    startpos.theta_degrees = get_double_field(env, startpos_object, "theta_degrees");

    void * random = ptr_from_obj(env, thisobject);

    position_t newpos =  
    rmhc_position_search(
            startpos,
            cmap_from_jmap(env, map_object),
            cscan_from_jscan(env, scan_object),
            sigma_xy_mm,
            sigma_theta_degrees,
            max_search_iter, 
            random);

    jclass cls = (*env)->FindClass(env, "edu/wlu/cs/levy/breezyslam/components/Position");

    jmethodID constructor = (*env)->GetMethodID(env, cls, "<init>", "(DDD)V");

    jobject newpos_object = (*env)->NewObject(env, cls, constructor, newpos.x_mm, newpos.y_mm, newpos.theta_degrees);

    return newpos_object;
} 
