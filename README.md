# hexapodControl




### Setup

1. Clone Hecapod Github Repo `git clonehttps://github.com/Hunter-dJ03/hexapodControl.git`
2. Clone Raisim Repo `git clone https://github.com/raisimTech/raisimLib.git`
3. Get Raisim License (https://docs.google.com/forms/d/e/1FAIpQLScNL0vbZPDNS93L6Jv6fgR51WTsvXxfhnVOtKDVRdAmHIoG4w/viewform)
4. Add Raisim License Download to .raisim folder
5. Install Dependencies
    - Build Essentials: `sudo apt install build-essential`
    - Eigen: `sudo apt install libeigen3-dev`
    - FMT: `sudo apt install libfmt-dev`
    - Boost: `sudo apt install libboost-all-dev`
    - Matplotlib-cpp: 
        - Install Python Dependencies: `sudo apt install python3-numpy python3-matplotlib python3-dev`
        - Clone Repo `git clone https://github.com/lava/matplotlib-cpp.git`
6. Navigate to worspace (singleLegControl, multilegControl, etc)
7. Update VSCode C++ Intellisense include path:
    ```
    ${workspaceFolder}/**
    /home/hunter/Documents/github/importedRepos/**
    /usr/include/python3.10/**
    /usr/include/eigen3/**
    ```
8. Update matplotlip-cpp directory full path in CMakeLists.txt to cloned location
9. Make build folder
10. Prep Cmake: `cmake .. -DCMAKE_PREFIX_PATH={Cloned Raisim Repo Full Path}/raisim/linux`
11. Compile `cmake`

### Using RaisimUnity

1. Follow instruction on the linked page (https://raisim.com/sections/RaisimUnity.html)
2. Also add the symlink if using ubuntu 22.04: `sudo ln -s /usr/lib/x86_64-linux-gnu/libdl.so.2 /usr/lib/x86_64-linux-gnu/libdl.so`