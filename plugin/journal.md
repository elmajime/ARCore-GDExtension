Right now :
- gradle build seems OK, it :
 - extracts the arcore libraries in build_gradle/arcore-native
 - compiles the cpp code using cmake
 - links to the extracted arcore library
 - generates the .so files in build_gradle/generated/intermediates/cxx/<hour>/obj/arm64-v8a
 - generates an apk in build_gradle\outputs\apk\debug
 - partially copies the files in the addons folder of the demo project

 
 Now the plugin code and project compile, deploy and run but it fails finding the ARCoreGDExtension plugin.
 I needed to make it compile against SDK 31 as the phone is 31.
 There might be a need to convert the project to v2 Android Plugins https://docs.godotengine.org/fr/4.x/tutorials/platform/android/android_plugin.html