// TODO: Update to match your plugin's package name.
package org.godotengine.plugin.android.gdextension.arcore

import android.util.Log
import androidx.annotation.NonNull
import org.godotengine.godot.Godot
import org.godotengine.godot.plugin.GodotPlugin
import org.godotengine.godot.plugin.UsedByGodot
import org.godotengine.plugin.android.gdextension.arcore.BuildConfig

class GDExtensionAndroidPluginARCore(godot: Godot): GodotPlugin(godot) {

    companion object {
        val TAG = GDExtensionAndroidPluginARCore::class.java.simpleName

        init {
            try {
                Log.v(TAG, "MCT Loading ${BuildConfig.GODOT_PLUGIN_NAME} library")
                System.loadLibrary(BuildConfig.GODOT_PLUGIN_NAME)
            } catch (e: UnsatisfiedLinkError) {
                Log.e(TAG, "MCT Unable to load ${BuildConfig.GODOT_PLUGIN_NAME} shared library")
            }
        }
    }

    override fun getPluginName() = BuildConfig.GODOT_PLUGIN_NAME

    override fun getPluginGDExtensionLibrariesPaths() = setOf("res://addons/${BuildConfig.GODOT_PLUGIN_NAME}/plugin.gdextension")

    // @Suppress("DEPRECATION")
    // @NonNull
    //override fun getPluginMethods(): List<String> {
    //      return listOf("getPluginName", "getPluginGDExtensionLibrariesPaths", "hello_world")
    // }

    /**
     * Example showing how to declare a native method that uses GDExtension C++ bindings and is
     * exposed to gdscript.
     *
     * Print a 'Hello World' message to the logcat.
     */
    @UsedByGodot
    private external fun helloWorld()

    @UsedByGodot
    private external fun initializeWrapper()

    @UsedByGodot
    private external fun uninitializeWrapper()
}
