# Run in every new terminal:  source ~/dhikriya/env.sh
source /opt/ros/jazzy/setup.bash
source ~/dhikriya/.venv/bin/activate
[ -f ~/dhikriya/install/setup.bash ] && source ~/dhikriya/install/setup.bash
# Render Gazebo (world + camera + LiDAR) on the NVIDIA RTX GPU, not Intel
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
# Where Gazebo finds our models (city_vla, prius_vla, traffic_car, auto_rickshaw, street_dog, signs)
export GZ_SIM_RESOURCE_PATH=$HOME/dhikriya/src/cogndn_sim/models:${GZ_SIM_RESOURCE_PATH:-}
# Clear snap (VS Code) settings that crash Gazebo/ROS GUIs
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR LOCPATH
