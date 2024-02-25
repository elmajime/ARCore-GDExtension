Right now :
- gradle build seems OK, it :
 - extracts the arcore libraries in build_gradle/arcore-native
 - compiles the cpp code using cmake
 - links to the extracted arcore library
 - generates the .so files in build_gradle/generated/intermediates/cxx/<hour>/obj/arm64-v8a
 - generates an apk in build_gradle\outputs\apk\debug
 - partially copies the files in the addons folder of the demo project

 