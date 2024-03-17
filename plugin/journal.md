Right now:
The run on Android manages to call arcore_interface.cpp functions !!
The best way to get logs is using logcat:

'''
adb shell
logcat | grep Godot
'''

Or using logcat as is but pausing it as soon as the camera usage notification has been clicked

The 2 current errors as listed in the log bellow are:

- set_name is not find even on feed (maybe not usefull though) 
    
    it maybe need a new godot "--dump-extension-api", " --dump-gdextension-interface" on godot repo with my changes

- env is not correct for use by ARCore

It fails when creating ARCore session with this error :
03-18 01:40:40.547 24013 24104 V ARCoreGDExtension: MCT Godot ARCore: Creating camera feed...
03-18 01:40:40.547 24013 24104 E godot   : USER ERROR: Parameter "mb" is null.
03-18 01:40:40.547 24013 24104 E godot   :    at: gdextension_classdb_get_method_bind (core/extension/gdextension_interface.cpp:1347)
03-18 01:40:40.547 24013 24104 E godot   : USER ERROR: Method bind was not found. Likely the engine method changed to an incompatible version.
03-18 01:40:40.547 24013 24104 E godot   :    at: set_name (gen\src\classes\camera_feed.cpp:68)
03-18 01:40:40.547 24013 24104 I godot   : Activate ???
03-18 01:40:40.547 24013 24104 V ARCoreGDExtension: MCT Godot ARCore: Feed 1 added
03-18 01:40:40.547 24013 24104 V ARCoreGDExtension: MCT Godot ARCore: Getting environment
03-18 01:40:40.547 24013 24104 V ARCoreGDExtension: MCT Godot ARCore: Create ArSession
03-18 01:40:40.547 24013 24104 I third_party/arcore/ar/core/android/sdk/session_create.cc: Entering ArSession_create
03-18 01:40:40.547 24013 24104 I third_party/arcore/ar/core/android/sdk/session_create.cc: ARCore Version: SDK build name: 1.31
03-18 01:40:40.547 24013 24104 W System.err: java.lang.NoSuchMethodError: no non-static method "Lorg/godotengine/plugin/android/gdextension/arcore/GDExtensionAndroidPluginARCore;.getClassLoader()Ljava/lang/ClassLoader;"
03-18 01:40:40.547 24013 24104 W System.err:    at org.godotengine.godot.GodotLib.step(Native Method)
03-18 01:40:40.547 24013 24104 W System.err:    at org.godotengine.godot.gl.GodotRenderer.onDrawFrame(GodotRenderer.java:57)
03-18 01:40:40.548 24013 24104 W System.err:    at org.godotengine.godot.gl.GLSurfaceView$GLThread.guardedRun(GLSurfaceView.java:1576)
03-18 01:40:40.548 24013 24104 W System.err:    at org.godotengine.godot.gl.GLSurfaceView$GLThread.run(GLSurfaceView.java:1278)
03-18 01:40:40.548 24013 24104 E third_party/redwood/infrastructure/jni_common/class_util.cc: Failed to find getClassLoader in context.
03-18 01:40:40.548 24013 24104 E third_party/arcore/ar/core/android/sdk/session_create.cc: Failed to get ClassLoader object from app context.
03-18 01:40:40.548 24013 24104 E third_party/arcore/ar/core/android/sdk/session_create.cc: LoadSymbols returning status.
03-18 01:40:40.548 24013 24104 E ARCoreGDExtension: Godot ARCore: ARCore couldn't be created.