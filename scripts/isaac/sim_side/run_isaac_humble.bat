@echo off
REM Launch Isaac Sim with the **Humble** internal ROS 2 libraries.
REM
REM Comments here are ASCII only on purpose: cmd.exe reads .bat files in the OEM
REM codepage (cp949 on a Korean Windows), so non-ASCII REM lines break parsing
REM ("EM is not recognized as an internal or external command").
REM Rationale in Korean: docs/simulation/isaac_backend_skeleton.md section 6.
REM
REM 1) The stock launcher sources setup_ros_env.bat, which has
REM      set DEFAULT_ROS_DISTRO=jazzy
REM    and sets ROS_DISTRO **and** PATH together inside one if-block. Presetting
REM    ROS_DISTRO alone skips that block, so humble\lib never lands on PATH and
REM    Isaac crashes looking for system ROS 2 DLLs. We reproduce both here.
REM
REM 2) Fast DDS profile disables shared-memory transport. Under WSL2 mirrored
REM    networking both sides look like the same host, so Fast DDS picks SHM -
REM    and Windows SHM is not Linux /dev/shm, so discovery silently never lands.
REM    **Set the same file on the WSL side too** (FASTRTPS_DEFAULT_PROFILES_FILE).

set "ISAAC_ROOT=C:\isaacsim"
set "ROS_DISTRO=humble"
set "PATH=%PATH%;%ISAAC_ROOT%\exts\isaacsim.ros2.core\humble\lib"
set "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
set "ROS_DOMAIN_ID=31"
set "FASTRTPS_DEFAULT_PROFILES_FILE=%ISAAC_ROOT%\fastdds_wsl_bridge.xml"

echo [run_isaac_humble] ROS_DISTRO=%ROS_DISTRO%  RMW=%RMW_IMPLEMENTATION%  DOMAIN=%ROS_DOMAIN_ID%
echo [run_isaac_humble] FASTRTPS_PROFILES=%FASTRTPS_DEFAULT_PROFILES_FILE%
call "%ISAAC_ROOT%\isaac-sim.bat" %*
