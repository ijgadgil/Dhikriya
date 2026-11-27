#!/usr/bin/env bash
# Build cogndn_msgs against Fast-CDR 2.2.5 to match this machine's ROS runtime (libfastcdr.so.2 -> 2.2.5).
set -e
cd ~/dhikriya
sed -i '/\.libfix/d;/Project-only fix: the system/d;/cogndn_msgs needs the official ROS 2.2.7/d' env.sh
rm -rf .libfix
source env.sh
echo "== 0. runtime libfastcdr: $(readlink -f /opt/ros/jazzy/lib/libfastcdr.so.2)"
echo "   LD_LIBRARY_PATH has libfix? $(echo $LD_LIBRARY_PATH | grep -c libfix || true)"
echo "== 1. std_msgs node test (must work before anything else)"
python3 -c "import rclpy; rclpy.init(); rclpy.create_node('t0'); print('   ROS node OK')"
P=$HOME/dhikriya/.fastcdr-2.2.5
if [ ! -f $P/lib/cmake/fastcdr/fastcdr-config.cmake ]; then
  echo "== 2. building Fast-CDR 2.2.5 into $P"
  rm -rf ~/fastcdr-2.2.5-src
  git clone -q --depth 1 -b v2.2.5 https://github.com/eProsima/Fast-CDR.git ~/fastcdr-2.2.5-src
  cmake -S ~/fastcdr-2.2.5-src -B ~/fastcdr-2.2.5-src/build -DCMAKE_INSTALL_PREFIX=$P -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release > /dev/null
  cmake --build ~/fastcdr-2.2.5-src/build -j4 > /dev/null
  cmake --install ~/fastcdr-2.2.5-src/build > /dev/null
fi
echo "   Fast-CDR 2.2.5 header: $(grep -m1 'FASTCDR_VERSION_STR' $P/include/fastcdr/config.h)"
echo "== 3. rebuilding cogndn_msgs against 2.2.5"
sudo mv /usr/local/include/fastcdr /usr/local/include/fastcdr.hidden 2>/dev/null || true
trap 'sudo mv /usr/local/include/fastcdr.hidden /usr/local/include/fastcdr 2>/dev/null || true' EXIT
rm -rf build/cogndn_msgs install/cogndn_msgs
python -m colcon build --symlink-install --base-paths src --packages-select cogndn_msgs --allow-overriding cogndn_msgs \
  --cmake-args -Dfastcdr_DIR=$P/lib/cmake/fastcdr "-DCMAKE_C_FLAGS=-I$P/include" "-DCMAKE_CXX_FLAGS=-I$P/include" \
  2>&1 | grep -E "Finished|Failed|error" || true
source env.sh
echo "== 4. checks"
echo "   uses 2.2.7-only function (must be 0): $(nm -DC install/cogndn_msgs/lib/libcogndn_msgs__rosidl_typesupport_fastrtps_c.so | grep -c 'Cdr::serialize(unsigned int)' || true)"
python3 -c "
import rclpy; rclpy.init(); n = rclpy.create_node('msgtest')
from cogndn_msgs.msg import DetectionArray, CognitiveState
n.create_publisher(DetectionArray, '/t1', 1).publish(DetectionArray())
n.create_publisher(CognitiveState, '/t2', 1).publish(CognitiveState())
print('   cogndn_msgs OK')"
