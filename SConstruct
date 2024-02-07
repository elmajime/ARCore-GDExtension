#!/usr/bin/env python
import os
import sys

def using_clang(env):
    return "clang" in os.path.basename(env["CC"])


def is_vanilla_clang(env):
    import subprocess
    if not using_clang(env):
        return False
    try:
        version = subprocess.check_output([env.subst(env["CXX"]), "--version"]).strip().decode("utf-8")
    except (subprocess.CalledProcessError, OSError):
        print("Couldn't parse CXX environment variable to infer compiler version.")
        return False
    return not version.startswith("Apple")

env = SConscript("godot-cpp/SConstruct")

# For reference:
# - CCFLAGS are compilation flags shared between C and C++
# - CFLAGS are for C-specific compilation flags
# - CXXFLAGS are for C++-specific compilation flags
# - CPPFLAGS are for pre-processor flags
# - CPPDEFINES are for pre-processor defines
# - LINKFLAGS are for linking flags

target_path = "plugin/demo/bin/"
target_name = "libGDExtensionAndroidPluginTemplate"

# tweak this if you want to use different folders, or more folders, to store your source code in.
if env.get("is_msvc", False):
    if env["debug_symbols"]:
        env.Append(CCFLAGS=["/Zi", "/FS"])
        env.Append(LINKFLAGS=["/DEBUG:FULL"])

    if env["optimize"] == "speed":
        env.Append(CCFLAGS=["/O2"])
        env.Append(LINKFLAGS=["/OPT:REF"])
    elif env["optimize"] == "speed_trace":
        env.Append(CCFLAGS=["/O2"])
        env.Append(LINKFLAGS=["/OPT:REF", "/OPT:NOICF"])
    elif env["optimize"] == "size":
        env.Append(CCFLAGS=["/O1"])
        env.Append(LINKFLAGS=["/OPT:REF"])
    elif env["optimize"] == "debug" or env["optimize"] == "none":
        env.Append(CCFLAGS=["/Od"])
else:
    if env["debug_symbols"]:
        # Adding dwarf-4 explicitly makes stacktraces work with clang builds,
        # otherwise addr2line doesn't understand them.
        env.Append(CCFLAGS=["-gdwarf-4"])
        if env.dev_build:
            env.Append(CCFLAGS=["-g3"])
        else:
            env.Append(CCFLAGS=["-g2"])
    else:
        if using_clang(env) and not is_vanilla_clang(env):
            # Apple Clang, its linker doesn't like -s.
            env.Append(LINKFLAGS=["-Wl,-S", "-Wl,-x", "-Wl,-dead_strip"])
        else:
            env.Append(LINKFLAGS=["-s"])

env.Append(CPPPATH=["plugin/src/main/cpp"])
sources = Glob("plugin/src/main/cpp/*.cpp")

env["use_hot_reload"] = True

if env["platform"] == "windows":
    sources = [item for item in sources if not "android" in str(item) ]
    sources = [item for item in sources if not "plugin_jni" in str(item) ]
    sources = [item for item in sources if not "arcore_plugin_wrapper" in str(item) ]
elif env["platform"] == "android":
    sources = [item for item in sources if not "windows" in str(item) ]

print("tuutu")
for file in sources:
    print(str(file))


if env["platform"] == "macos":
    library = env.SharedLibrary(
        "plugin/demo/bin/libGDExtensionAndroidPluginTemplate.{}.{}.framework/libGDExtensionAndroidPluginTemplate.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "{}/{}{}{}".format(target_path, target_name, env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    ) 

    env['CCPDBFLAGS'] = '/Zi /Fd${TARGET}.pdb'
    env['PDB'] = "{}/{}{}{}.pdb".format(target_path, target_name, env["suffix"], env["SHLIBSUFFIX"])

Default(library)