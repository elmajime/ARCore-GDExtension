//
// Created by Patrick on 07.09.2023.
//

#ifndef ARCOREGDEXTENSION_ARCORE_PLUGIN_WRAPPER_H
#define ARCOREGDEXTENSION_ARCORE_PLUGIN_WRAPPER_H

#include "utils.h"

class ARCorePluginWrapper {
public:
    static void initialize_wrapper(JNIEnv *env, jobject activity);
    static void uninitialize_wrapper(JNIEnv *env);

private:
    static JNIEnv *env;
    static jobject arcore_plugin_instance;
    static jobject godot_instance;
    static jobject activity;
    static jclass godot_class;
    static jclass activity_class;

    // static jmethodID _get_surface;
    // static jmethodID _is_activity_resumed;
    // static jmethodID _get_display_rotation;

public:
    ARCorePluginWrapper();
    ~ARCorePluginWrapper();

    static JNIEnv* get_env();
    static jobject get_activity();
    // static jobject get_surface(JNIEnv *p_env);
    // static bool is_activity_resumed(JNIEnv *p_env);
    // static int get_display_rotation(JNIEnv *p_env);
};

#endif //ARCOREGDEXTENSION_ARCORE_PLUGIN_WRAPPER_H
