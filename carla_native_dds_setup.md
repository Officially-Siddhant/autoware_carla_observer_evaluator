# CARLA Native DDS Setup for Autoware Bridge
## Complete Instruction Set (From Scratch)

This document describes how to enable native DDS (ROS2) support in a CARLA source build,
which is required for the `autoware_carla_leaderboard` bridge. If you have a non-source build of CARLA, this might not be relevant to you and you can simply skip this and follow this brilliant guide for setting up the Autoware Bridge [here](https://github.com/TUMFTM/autoware_carla_leaderboard?tab=readme-ov-file).

**Requirements and Assumptions:**
- CARLA source build is located at `/home/username/carla`
- Unreal Engine 4.26 is located at `/home/username/UnrealEngine_4.26`
- ROS2 Humble is installed on the host (your system, not in docker)
- The original CARLA build was compiled **without** `--ros2`
- Ubuntu 22.04

---

## Step 1: Add `launch-dds` Target to Linux.mk

Edit `/home/username/carla/Util/BuildTools/Linux.mk` and add the following target
after the existing `launch` target:

```makefile
launch-dds: LibCarla.server.release osm2odr downloadplugins
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildUE4Plugins.sh --build $(ARGS)
	@${CARLA_BUILD_TOOLS_FOLDER}/BuildCarlaUE4.sh --build --launch --ros2 --editor-flags="-ros2" $(ARGS)
```

This gives you two separate launch commands:
- `make launch` — original behavior, unchanged
- `make launch-dds` — launches CARLA with native DDS enabled for Autoware

**Important:** The `--editor-flags="-ros2"` part is critical. Without it, `-ros2` is
only a build-time flag and is never passed to the UE4Editor executable at runtime.
Both flags are required: `--ros2` enables the build configuration and
`--editor-flags="-ros2"` passes the flag to the running CARLA process.

---

## Step 2: Set Environment Variables

Open a **fresh terminal** (do NOT source `Environment.sh` directly — it uses `set -e`
and will close your terminal if any check fails). Set all required variables manually:

```bash
export CURDIR=/home/skodas/carla
export CARLA_ROOT_FOLDER=${CURDIR}
export CARLA_BUILD_FOLDER=${CURDIR}/Build
export CARLAUE4_ROOT_FOLDER=${CURDIR}/Unreal/CarlaUE4
export CARLAUE4_PLUGIN_ROOT_FOLDER=${CURDIR}/Unreal/CarlaUE4/Plugins/Carla
export LIBCARLA_INSTALL_SERVER_FOLDER=${CARLAUE4_PLUGIN_ROOT_FOLDER}/CarlaDependencies
export LIBCPP_TOOLCHAIN_FILE=${CARLA_BUILD_FOLDER}/LibCppToolChain.cmake
export UE4_ROOT=/home/skodas/UnrealEngine_4.26
export CC="$UE4_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v17_clang-10.0.1-centos7/x86_64-unknown-linux-gnu/bin/clang"
export CXX="$UE4_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v17_clang-10.0.1-centos7/x86_64-unknown-linux-gnu/bin/clang++"
export PATH="$UE4_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v17_clang-10.0.1-centos7/x86_64-unknown-linux-gnu/bin:$PATH"
export LLVM_INCLUDE="$UE4_ROOT/Engine/Source/ThirdParty/Linux/LibCxx/include/c++/v1"
export LLVM_LIBPATH="$UE4_ROOT/Engine/Source/ThirdParty/Linux/LibCxx/lib/Linux/x86_64-unknown-linux-gnu"
export UNREAL_HOSTED_CFLAGS="--sysroot=$UE4_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v17_clang-10.0.1-centos7/x86_64-unknown-linux-gnu/"
export FASTDDS_BASENAME=fast-dds
export FASTDDS_INSTALL_DIR=${CARLA_BUILD_FOLDER}/${FASTDDS_BASENAME}-install
export FASTDDS_INCLUDE=${FASTDDS_INSTALL_DIR}/include
export FASTDDS_LIB=${FASTDDS_INSTALL_DIR}/lib
export BOOST_INCLUDE=${CARLA_BUILD_FOLDER}/boost-1.84.0-c10-install/include
```

Verify the compilers exist before proceeding:

```bash
ls $CC
ls $CXX
```

Both paths should exist. If not, verify your UE4_ROOT path is correct.

---

## Step 3: Build Foonathan Memory Vendor

Foonathan is a dependency of FastDDS.

```bash
cd ${CARLA_BUILD_FOLDER}
git clone --depth 1 --branch v1.3.1 \
    https://github.com/eProsima/foonathan_memory_vendor.git \
    foonathan-memory-vendor-source

mkdir -p foonathan-memory-vendor-source/build
cd foonathan-memory-vendor-source/build

cmake -G "Ninja" \
    -DCMAKE_INSTALL_PREFIX="${FASTDDS_INSTALL_DIR}" \
    -DCMAKE_TOOLCHAIN_FILE=${LIBCPP_TOOLCHAIN_FILE} \
    -DBUILD_SHARED_LIBS=OFF \
    -DFOONATHAN_MEMORY_FORCE_VENDORED_BUILD=ON \
    ..

ninja
ninja install
```

---

## Step 4: Copy OpenSSL from UE4

**This is a critical step that is easy to miss.** FastDDS requires OpenSSL headers
and libraries. CARLA's Setup.sh copies these from UE4, but since we are building
manually we must do it ourselves BEFORE running cmake for FastDDS.

```bash
mkdir -p ${FASTDDS_INCLUDE}
mkdir -p ${FASTDDS_LIB}

cp -r ${UE4_ROOT}/Engine/Source/ThirdParty/OpenSSL/1.1.1c/include/Linux/x86_64-unknown-linux-gnu/* \
    ${FASTDDS_INCLUDE}

cp -r ${UE4_ROOT}/Engine/Source/ThirdParty/OpenSSL/1.1.1c/lib/Linux/x86_64-unknown-linux-gnu/* \
    ${FASTDDS_LIB}
```

Verify the OpenSSL header is in place:

```bash
ls ${FASTDDS_INCLUDE}/openssl/conf.h
```

This must succeed before continuing.

---

## Step 5: Build FastDDS

```bash
cd ${CARLA_BUILD_FOLDER}
git clone --recurse-submodules --depth 1 --branch v2.11.2 \
    https://github.com/eProsima/Fast-DDS.git \
    fast-dds-lib-source

mkdir -p fast-dds-lib-source/build
cd fast-dds-lib-source/build

cmake -G "Ninja" \
    -DCMAKE_INSTALL_PREFIX="${FASTDDS_INSTALL_DIR}" \
    -DFORCE_CXX="14" \
    -DCMAKE_CXX_FLAGS="-fPIC -std=c++14 -stdlib=libc++ -I${LLVM_INCLUDE} -I${FASTDDS_INCLUDE} -Wl,-L${LLVM_LIBPATH} -DBOOST_NO_EXCEPTIONS ${UNREAL_HOSTED_CFLAGS}" \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_TESTING=OFF \
    -DCOMPILE_EXAMPLES=OFF \
    -DCOMPILE_TOOLS=OFF \
    -DTHIRDPARTY_Asio=FORCE \
    -DTHIRDPARTY_fastcdr=FORCE \
    -DTHIRDPARTY_TinyXML2=FORCE \
    -DSQLITE3_SUPPORT=OFF \
    -DOPENSSL_FOUND:BOOL=ON \
    -DOPENSSL_INCLUDE_DIR:FILEPATH=${FASTDDS_INCLUDE} \
    -DOPENSSL_SSL_LIBRARY:FILEPATH=${FASTDDS_LIB}/libssl.a \
    -DOPENSSL_CRYPTO_LIBRARY:FILEPATH=${FASTDDS_LIB}/libcrypto.a \
    ..

ninja
ninja install
```

**Expected warnings (harmless):**
- `Couldn't compile thirdparty/boost. SHM Transport feature will be disabled` — Shared
  Memory Transport is not needed; the bridge uses network DDS.
- `-Wl,-L...: 'linker' input unused` — clang warning, does not affect the build.

Verify the libraries were produced:

```bash
ls ${FASTDDS_LIB}/*.a
```

You should see: `libcrypto.a`, `libfastcdr.a`, `libfastrtps.a`, `libssl.a`

---

## Step 6: Copy FastDDS Libraries to CarlaDependencies

```bash
cp -p ${FASTDDS_LIB}/*.a ${LIBCARLA_INSTALL_SERVER_FOLDER}/lib/
```

---

## Step 7: Copy Foonathan Library from ROS2 Humble

The foonathan vendored build does not produce a static library because cmake finds
the system ROS2 Humble version instead. Copy it directly from ROS2 Humble with the
version name the linker expects:

```bash
cp /opt/ros/humble/lib/libfoonathan_memory-0.7.1.a \
    ${LIBCARLA_INSTALL_SERVER_FOLDER}/lib/libfoonathan_memory-0.7.3.a
```

---

## Step 8: Build LibCarla with ROS2 Support

This produces `libcarla_fastdds.a` — the CARLA ROS2 publishers/subscribers library.

```bash
cd /home/skodas/carla
make LibCarla.server.release ARGS="--ros2"
```

This will install `libcarla_fastdds.a` and all ROS2 message headers into
`CarlaDependencies/`.

---

## Step 9: Verify All 4 Libraries Are Present

```bash
ls ${LIBCARLA_INSTALL_SERVER_FOLDER}/lib/ | grep -E "fastrtps|fastcdr|foonathan|carla_fastdds"
```

Expected output:
```
libcarla_fastdds.a
libfastcdr.a
libfastrtps.a
libfoonathan_memory-0.7.3.a
```

All 4 must be present before proceeding.

---

## Step 10: Enable WITH_ROS2 in the UE4 Build

This is a critical step. The `Carla.Build.cs` file reads `OptionalModules.ini` to
define the `WITH_ROS2` preprocessor macro. Without this, the entire ROS2 code path
is compiled out even though the libraries are linked.

Write `Ros2 ON` to `OptionalModules.ini` in the correct format:

```bash
echo "Fast_dds ON
Unity ON
Ros2 ON
Pytorch OFF
Chrono OFF
CarSim OFF
SimReady ON" > /home/skodas/carla/Unreal/CarlaUE4/Config/OptionalModules.ini
```

Then force a full recompile of the Carla plugin by touching `Carla.Build.cs` and
deleting the intermediate files:

```bash
touch /home/skodas/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Carla.Build.cs
rm -rf /home/skodas/carla/Unreal/CarlaUE4/Plugins/Carla/Intermediate/
rm /home/skodas/carla/Unreal/CarlaUE4/Plugins/Carla/Binaries/Linux/libUE4Editor-Carla.so
```

Then recompile:

```bash
cd /home/skodas/carla/Unreal/CarlaUE4
make CarlaUE4Editor 2>&1 | tee /tmp/ue4_build.log
grep -i "enabling ros2" /tmp/ue4_build.log
```

You must see `Enabling ros2` in the build output before proceeding. If you do not
see it, the `WITH_ROS2` macro was not defined and the ROS2 code path will not work.

---

## Step 11: Fix CarlaEngine.cpp to Check Command Line at Runtime

The `-ros2` flag is parsed too early in initialization before `FCommandLine` is
populated. Edit `CarlaEngine.cpp` to also check the command line directly at the
point where ROS2 is initialized:

```bash
sed -n '218,228p' /home/skodas/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaEngine.cpp
```

The block should look like this after editing:

```cpp
// create ROS2 manager
#if defined(WITH_ROS2)
if (Settings.ROS2 || FParse::Param(FCommandLine::Get(), TEXT("-ros2")))
{
    UE_LOG(LogCarla, Log, TEXT("ROS2 enabling: Settings.ROS2=%d CommandLine=%d"), Settings.ROS2, (int)FParse::Param(FCommandLine::Get(), TEXT("-ros2")));
    auto ROS2 = carla::ros2::ROS2::GetInstance();
    ROS2->Enable(true);
}
#endif
```

Apply the edit using sed or manually in your editor. Then force recompile
`CarlaEngine.cpp` specifically:

```bash
touch /home/skodas/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game/CarlaEngine.cpp
cd /home/skodas/carla/Unreal/CarlaUE4
make CarlaUE4Editor
```

---

## Step 12: Launch CARLA with Native DDS

```bash
cd /home/skodas/carla
make launch-dds
```

Press **Play** in the CARLA editor once it opens.

Verify DDS is working by checking the process arguments:

```bash
ps aux | grep UE4Editor | grep -v grep
```

You should see `-ros2` in the UE4Editor arguments:

```
UE4Editor /home/skodas/carla/Unreal/CarlaUE4/CarlaUE4.uproject -vulkan -ros2
```

---

## Summary of Libraries and Their Sources

| Library | Source | How Built |
|---|---|---|
| `libfastcdr.a` | FastDDS v2.11.2 (Step 5) | Built manually with ninja |
| `libfastrtps.a` | FastDDS v2.11.2 (Step 5) | Built manually with ninja |
| `libfoonathan_memory-0.7.3.a` | ROS2 Humble system install | Copied from `/opt/ros/humble/lib/` |
| `libcarla_fastdds.a` | LibCarla ROS2 build (Step 8) | `make LibCarla.server.release ARGS="--ros2"` |

---

## Why We Built Manually Instead of Using Setup.sh --ros2

Running `bash Util/BuildTools/Setup.sh --ros2` fails silently because:

1. `Setup.sh` uses `set -e` which causes it to exit on any error without reporting it.
2. It does not copy OpenSSL headers into `FASTDDS_INCLUDE` before running cmake,
   causing the FastDDS build to fail with `fatal error: 'openssl/conf.h' file not found`.
3. It removes the build directory (`rm -Rf ${FAST_DDS_LIB_SOURCE_DIR}`) regardless of
   whether the build succeeded, destroying any evidence of the failure.

The manual approach above replicates exactly what Setup.sh intends to do, with the
OpenSSL step added and full visibility into errors.

---

## Important Notes on UE4 Build System Behavior

- **`OptionalModules.ini` must exist before compilation.** The `Carla.Build.cs` reads
  it at compile time to define `WITH_ROS2`. If it contains `Ros2 OFF` or is written
  after compilation, the macro will not be defined.

- **Touching `Carla.Build.cs` is required** to force UBT to re-evaluate the build
  rules. Without this, UBT uses cached object files and skips recompilation even if
  `OptionalModules.ini` has changed.

- **Deleting `Intermediate/` forces full recompilation** of all `.cpp` files in the
  Carla plugin. This is necessary the first time `WITH_ROS2` is added.

- **The `-ros2` runtime flag** is separate from the build-time `WITH_ROS2` macro.
  Both are required: the macro enables the code path, and the runtime flag activates
  it when CARLA starts.

---

## Future Workflow

Once this setup is complete, you never need to repeat Steps 2-11. Your future workflow is:

```bash
# Normal CARLA launch (no DDS)
make launch

# CARLA launch with native DDS for Autoware bridge
make launch-dds
# Then press Play in the CARLA editor
```
