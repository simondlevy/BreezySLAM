#include "../../../../../../c/coreslam.h"

#include <jni.h>
#include <stdlib.h>

static jfieldID get_fid(JNIEnv *env, jobject object, const char * fieldname, const char * fieldsig)
{
    jclass cls = (*env)->GetObjectClass(env, object);
    return (*env)->GetFieldID(env, cls, fieldname, fieldsig);
}

static jfieldID get_this_fid(JNIEnv *env, jobject thisobject)
{
    return get_fid(env, thisobject, "native_ptr", "J");
}

static void * ptr_from_obj(JNIEnv *env, jobject thisobject)
{
    return (void *)(*env)->GetLongField (env, thisobject, get_this_fid(env, thisobject));
}

static void ptr_to_obj(JNIEnv *env, jobject thisobject, void * ptr)
{
    (*env)->SetLongField (env, thisobject, get_this_fid(env, thisobject), (long)ptr);
}

static scan_t * cscan_from_jscan(JNIEnv *env, jobject thisobject)
{
    return (scan_t *)ptr_from_obj(env, thisobject);
}

static map_t * cmap_from_jmap(JNIEnv *env, jobject thisobject)
{
    return (map_t *)ptr_from_obj(env, thisobject);
}    

static double get_double_field(JNIEnv *env, jobject object, const char * fieldname)
{
    return (*env)->GetDoubleField (env, object, get_fid(env, object, fieldname, "D"));
}


