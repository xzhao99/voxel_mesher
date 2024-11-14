# voxel_mesher

## set up vcpkg and dependencies
```
https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-powershell
```
Notes about the four json files:
1. ```vcpkg.json``` is a manifest file created in your project's directory when you run ```vcpkg new --application and vcpkg add port your_package```. vcpkg reads the manifest file to learn what dependencies to install and integrates with CMake to provide the dependencies required by your project.

2. The default ```vcpkg-configuration.json``` file introduces baseline constraints, specifying the minimum versions of dependencies that your project should use. it plays a crucial role in defining version constraints for your project's dependencies. it's a good practice to add vcpkg-configuration.json to your source control to ensure version consistency across different development environments.

3. CMake can automatically link libraries installed by vcpkg when CMAKE_TOOLCHAIN_FILE is set to use vcpkg's custom toolchain. This can be acomplished using ```CMakePresets.json and CMakeUserPresets.json``` files. The ```CMakePresets.json``` file contains a single preset named "vcpkg", which sets the CMAKE_TOOLCHAIN_FILE variable. The ```CMakeUserPresets.json``` file sets the VCPKG_ROOT environment variable to point to the absolute path containing your local installation of vcpkg. It is recommended to not check CMakeUserPresets.json into version control systems.



## build with CMake and vcpkg
```
# cmake --preset=default  -DCMAKE_BUILD_TYPE=Release
# cmake --build build --config Release

```