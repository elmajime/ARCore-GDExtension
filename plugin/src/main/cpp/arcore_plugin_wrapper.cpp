#include "arcore_plugin_wrapper.h"

#include "utils.h"
#include <godot_cpp/godot.hpp>
#include <godot_cpp/classes/os.hpp>

JNIEnv *ARCorePluginWrapper::env = nullptr;
jobject ARCorePluginWrapper::arcore_plugin_instance = nullptr;
jobject ARCorePluginWrapper::godot_instance = nullptr;
jobject ARCorePluginWrapper::activity = nullptr;
jclass ARCorePluginWrapper::godot_class = nullptr;
jclass ARCorePluginWrapper::activity_class = nullptr;

// jmethodID ARCorePluginWrapper::_get_surface = nullptr;
// jmethodID ARCorePluginWrapper::_is_activity_resumed = nullptr;
// jmethodID ARCorePluginWrapper::_get_display_rotation = nullptr;

ARCorePluginWrapper::ARCorePluginWrapper() {
    // _get_surface = nullptr;
    // _is_activity_resumed = nullptr;
    // _get_display_rotation = nullptr;
}

ARCorePluginWrapper::~ARCorePluginWrapper() {}

void ARCorePluginWrapper::initialize_wrapper(JNIEnv *p_env, jobject p_activity) {

    env = p_env;
    
    activity = p_env->NewGlobalRef(p_activity);

    // get info about our Godot class so we can get pointers and stuff...
    godot_class = p_env->FindClass("org/godotengine/godot/Godot");
    if (godot_class) {
        godot_class = (jclass)p_env->NewGlobalRef(godot_class);
    } else {
        ALOGE("Godot ARCore: did not find org/godotengine/godot/Godot");
        // this is a pretty serious fail.. bail... pointers will stay 0
        return;
    }
    activity_class = p_env->FindClass("android/app/Activity");
    if (activity_class) {
        activity_class = (jclass)p_env->NewGlobalRef(activity_class);
    } else {
        ALOGE("Godot ARCore: did not find android/app/Activity");
        // this is a pretty serious fail.. bail... pointers will stay 0
        return;
    }

    //! Maxime something similar to get the display orientation ?
    // // get some Godot method pointers...
    // _get_surface = p_env->GetMethodID(godot_class, "getSurface", "()Landroid/view/Surface;");
    // _is_activity_resumed = p_env->GetMethodID(godot_class, "isActivityResumed", "()Z");
    // _get_display_rotation = p_env->GetMethodID(godot_class, "getDisplayRotation", "()I");
}

void ARCorePluginWrapper::uninitialize_wrapper(JNIEnv *env) {
    if (arcore_plugin_instance) {
        env->DeleteGlobalRef(arcore_plugin_instance);

        arcore_plugin_instance = nullptr;

        env->DeleteGlobalRef(godot_instance);
        env->DeleteGlobalRef(godot_class);
        env->DeleteGlobalRef(activity);
        env->DeleteGlobalRef(activity_class);
    }
}

JNIEnv *ARCorePluginWrapper::get_env() {
    return env;
}

jobject ARCorePluginWrapper::get_godot_class() {
    return godot_class;
}

jobject ARCorePluginWrapper::get_activity() {
    return activity;
}

jobject ARCorePluginWrapper::get_global_context()
{   
    jclass activityThread = env->FindClass("android/app/ActivityThread");
    jmethodID currentActivityThread = env->GetStaticMethodID(activityThread, "currentActivityThread", "()Landroid/app/ActivityThread;");
    jobject activityThreadObj = env->CallStaticObjectMethod(activityThread, currentActivityThread);
    
    jmethodID getApplication = env->GetMethodID(activityThread, "getApplication", "()Landroid/app/Application;");
    jobject context = env->CallObjectMethod(activityThreadObj, getApplication);
    return context;
}

// jobject ARCorePluginWrapper::get_surface(JNIEnv *p_env) {
//     if (_get_surface) {
//         return p_env->CallObjectMethod(godot_instance, _get_surface);
//     } else {
//         return nullptr;
//     }
// }

// bool ARCorePluginWrapper::is_activity_resumed(JNIEnv *p_env) {
//     if (_is_activity_resumed) {
//         return p_env->CallBooleanMethod(godot_instance, _is_activity_resumed);
//     } else {
//         return false;
//     }
// }

// int ARCorePluginWrapper::get_display_rotation(JNIEnv *p_env) {
//     if (_get_display_rotation) {
//         return p_env->CallIntMethod(godot_instance, _get_display_rotation);
//     } else {
//         return 0;
//     }
// }
