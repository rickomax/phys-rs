
## PhysX-RS (No Rust)

This repository contains modified components derived from the following projects:
- MagicPhysX (https://github.com/Cysharp/MagicPhysX)
- physx-rs (https://github.com/EmbarkStudios/physx-rs)

It uses the C bindings from the physx-sys layer:
- https://github.com/EmbarkStudios/physx-rs/tree/main/physx-sys

All components have been adapted to build with CMake and do not require Rust.

This repository uses only the low-level P/Invoke bindings and related interop structures from MagicPhysX.  
The higher-level C# interface provided by MagicPhysX is not included.  
A higher-level Unity interface is in progress and will be published separately.

---

### Overview

- The generated wrapper code (trampolines, callbacks, etc.) has been ported from Rust to C.
- All PhysX bindings are exposed through a flat C API.
- A custom internal generator was used (based on the original), with additional generation of:
  - struct offsets

---

### Purpose

This project aims to provide a clean and portable foundation for compiling PhysX-based tooling on platforms where Rust is not available or not desirable. It is still in its early stages.

---

### Platform Support

- Tested: Windows x64
- Other platforms: Not officially tested

Contributions, fixes, and platform support improvements are welcome.

---

### Compiling

This project uses CMake.

Before building this project, you must compile the underlying PhysX as a static library. We use PhysX 5.1.3 modified by Embark Studios:
- https://github.com/EmbarkStudios/Physx-5/tree/94ca32881287dc2a2063551e8ed3c5af5d6a1742

Notes:
- This PhysX version has some features stripped or altered. Refer to the usage of the `PX_DEPRECATED` macro in the source code for details.
- Contributions to align it with the upstream PhysX 5.1.3 repository are welcome.

After building the PhysX static libraries for your target platform, pass the local PhysX repository path to CMake using the `PHYSX_ROOT_DIR` variable.

---

### License & Attribution

This project includes code derived from:
- MagicPhysX (MIT License) — P/Invoke bindings and low-level interop code only
- physx-rs / physx-sys (MIT License)

All original authors retain copyright over their respective portions.
