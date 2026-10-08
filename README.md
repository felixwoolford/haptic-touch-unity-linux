# HapticsDirect for Linux

A Linux x86-64 native plugin for the existing `HapticPlugin.cs` Unity scripts.
The C# imports work unchanged with `libHapticsDirect.so`.

The aim of this plugin is to allow projects using Touch or Touch X devices previously developped in Windows to also work identically in Linux. A few obvious bugs from the Windows version have also been corrected. A later tool will introduce new features and C# scripts that are not compatible with the Windows DLL.

The plugin is **black-box tested against the original Windows
`HapticsDirect.dll`**: identical scripted devices and calls are run through
both implementations, comparing HD force outputs and API results. Tests use
fake devices, not hardware. Documented fixes intentionally differ from DLL bugs;
see below. The current regression suite covers 581 standing cases and 2,048 seeded cases. This does not validate physical
calibration or hardware workspace scaling.


## Build and install in Unity

Install the Linux OpenHaptics SDK (HD/HDU headers and libraries) and the device's
Linux driver/runtime. Configure and calibrate the device using the vendor tools.
You also need `g++`. With SDK headers/libraries on the compiler's search paths,
run from this repository's root:

```bash
mkdir -p release-build
g++ -std=gnu++11 -O2 -fPIC -shared -Wl,--no-undefined \
  -o release-build/libHapticsDirect.so \
  src/HapticsDirect.cpp -lHD -lHDU -pthread
ldd release-build/libHapticsDirect.so
```

Ensure `ldd` reports no missing dependencies and resolves `libHD` to the **real
installed OpenHaptics runtime**, not anything under `difftest/build/`.
If the SDK is in a nonstandard location, supply its include/library paths with
`-I`/`-L` and make its runtime libraries available to the Linux loader.

1. Close Unity and back up the project's plugin folder.
2. Copy `release-build/libHapticsDirect.so` into the Unity project's
   `Assets/3DSystems/HapticsDirect/HapticPlugin/` directory (or its equivalent if you moved
   the Haptics Direct assets).
3. In this **Linux-only deployment copy**, delete all Windows native `.dll`
   files from the HapticPlugin directory, including `HapticsDirect.dll` and
   `hd.dll`. 
4. Reopen Unity. In the `.so` Plugin Inspector, enable Linux Editor/Standalone,
   select x86-64, and disable other platforms. Keep `HapticPlugin.cs` unchanged.


## Bug fixes

- All published contacts contribute to force output, rather than only the last;
  contact-force getters report current sums and clear when there are no contacts.
- `setForce` now renders the supplied force, with component clamping and scaling
  by the device's maximum force. Torque output is not implemented.
- Viscosity filtering is per device and advances once per active servo tick,
  avoiding interference between devices and dependence on contact count.
- Resetting contact staging leaves the active contacts intact until publication;
  duplicate contact updates use a defined point-and-normal lookup.
- Transform handedness conversion preserves proper rotations, and workspace
  bounds use the same coordinate frame as position.
- Getters no longer create phantom devices when given an unknown device name.
- String getters copy, terminate and truncate within the supplied buffer instead
  of appending or using an incorrect buffer size.
- Friction getters report the actual force and distinguish static/dynamic terms.
- Previous-button state is no longer OR-latched, and reading the inkwell state
  no longer clears it. Current-button press latching is retained.
- Initialization no longer applies the incompatible Windows OpenHaptics version
  gate, and devices initialized after scheduler start join the servo callback.
- Disconnect stops and unschedules callbacks under owned synchronization rather
  than releasing a mutex that may not be owned.

Linux also checks and updates device calibration on the scheduler thread,
without blocking while manual input is required. Hardware validation remains
necessary. Full snapshot/list thread safety, soft-band ramp correction and
vibration-cap correction remain TODOs, not completed fixes.
