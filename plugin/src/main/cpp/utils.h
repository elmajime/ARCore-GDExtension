#ifndef UTILS_H
#define UTILS_H

#include <android/log.h>
#include <godot_cpp/variant/string.hpp>
#include "godot_cpp/variant/projection.hpp"
#include "godot_cpp/variant/transform3d.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include "godot_cpp/variant/vector4.hpp"
#include "glm.hpp"
#include <jni.h>

#define LOG_TAG "ARCoreGDExtension"

#define ALOG_ASSERT(_cond, ...) \
    if (!(_cond)) __android_log_assert("conditional", LOG_TAG, __VA_ARGS__)
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define ALOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
#define ALOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)

/** Auxiliary macros */
#define __JNI_METHOD_BUILD(package, class_name, method) \
    Java_ ## package ## _ ## class_name ## _ ## method
#define __JNI_METHOD_EVAL(package, class_name, method) \
    __JNI_METHOD_BUILD(package, class_name, method)

/**
 * Expands the JNI signature for a JNI method.
 *
 * Requires to redefine the macros JNI_PACKAGE_NAME and JNI_CLASS_NAME.
 * Not doing so will raise preprocessor errors during build.
 *
 * JNI_PACKAGE_NAME must be the JNI representation Java class package name.
 * JNI_CLASS_NAME must be the JNI representation of the Java class name.
 *
 * For example, for the class com.example.package.SomeClass:
 * JNI_PACKAGE_NAME: com_example_package
 * JNI_CLASS_NAME: SomeClass
 *
 * Note that underscores in Java package and class names are replaced with "_1"
 * in their JNI representations.
 */
#define JNI_METHOD(method) \
    __JNI_METHOD_EVAL(JNI_PACKAGE_NAME, JNI_CLASS_NAME, method)

/**
 * Expands a Java class name using slashes as package separators into its
 * JNI type string representation.
 *
 * For example, to get the JNI type representation of a Java String:
 * JAVA_TYPE("java/lang/String")
 */
#define JAVA_TYPE(class_name) "L" class_name ";"

/**
 * Default definitions for the macros required in JNI_METHOD.
 * Used to raise build errors if JNI_METHOD is used without redefining them.
 */
#define JNI_CLASS_NAME "Error: JNI_CLASS_NAME not redefined"
#define JNI_PACKAGE_NAME "Error: JNI_PACKAGE_NAME not redefined"

/**
* Converts JNI jstring to Godot String.
* @param source Source JNI string. If null an empty string is returned.
* @param env JNI environment instance.
* @return Godot string instance.
*/
inline godot::String jstring_to_string(JNIEnv *env, jstring source) {
    if (env && source) {
        const char *const source_utf8 = env->GetStringUTFChars(source, NULL);
        if (source_utf8) {
            godot::String result(source_utf8);
            env->ReleaseStringUTFChars(source, source_utf8);
            return result;
        }
    }
    return godot::String();
}

inline godot::Transform3D glm_to_godot_transform(const glm::mat4& glm_matrix) {
    // // Extract the basis (3x3 part)
    // godot::Basis basis(
    //     glm_matrix[0][0], glm_matrix[1][0], glm_matrix[2][0],
    //     glm_matrix[0][1], glm_matrix[1][1], glm_matrix[2][1],
    //     glm_matrix[0][2], glm_matrix[1][2], glm_matrix[2][2]
    // );

    // // Extract the origin (translation part)
    // godot::Vector3 origin(
    //     glm_matrix[3][0],
    //     glm_matrix[3][1],
    //     glm_matrix[3][2]
    // );

    // Create the Transform
    // return godot::Transform3D(basis, origin);

	// Transform3D(real_t xx, real_t xy, real_t xz, real_t yx, real_t yy, real_t yz, real_t zx, real_t zy, real_t zz, real_t ox, real_t oy, real_t oz);
    godot::Transform3D res(godot::Vector3(glm_matrix[0][0], glm_matrix[0][1], glm_matrix[0][2]), 
                           godot::Vector3(glm_matrix[1][0], glm_matrix[1][1], glm_matrix[1][2]), 
                           godot::Vector3(glm_matrix[2][0], glm_matrix[2][1], glm_matrix[2][2]), 
                           godot::Vector3(glm_matrix[3][0], glm_matrix[3][1], glm_matrix[3][2]));

    return res;
}

inline godot::Projection glm_to_godot_projection(const glm::mat4& glm_matrix) 
{
    // Create a godot::Projection from the glm::mat4
    godot::Projection godot_projection;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            godot_projection[i][j] = glm_matrix[i][j];
        }
    }

    return godot_projection;
}

#endif // UTILS_H
