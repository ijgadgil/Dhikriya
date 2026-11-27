#!/usr/bin/env bash
# Checks every dependency of Cognitive-NDN and prints what is missing.
ok(){ printf "  [ OK ] %s\n" "$1"; }; bad(){ printf "  [FAIL] %s  -> %s\n" "$1" "$2"; FAIL=1; }
FAIL=0
echo "Cognitive-NDN environment check"
[ -n "${ROS_DISTRO:-}" ] && ok "ROS 2 ($ROS_DISTRO)" || bad "ROS 2" "source /opt/ros/jazzy/setup.bash"
command -v gz >/dev/null && ok "Gazebo ($(gz sim --versions 2>/dev/null | head -1))" || bad "Gazebo Harmonic" "sudo apt install ros-jazzy-ros-gz"
ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 && ok "ros_gz_bridge" || bad "ros_gz_bridge" "sudo apt install ros-jazzy-ros-gz"
command -v ndnd >/dev/null && ok "NDNd ($(ndnd --version 2>/dev/null | head -1))" || bad "NDNd" "see setup guide step 5"
redis-cli ping >/dev/null 2>&1 && ok "Redis" || bad "Redis" "sudo apt install redis-server && sudo systemctl enable --now redis-server"
curl -s localhost:11434/api/tags >/dev/null && ok "Ollama" || bad "Ollama" "curl -fsSL https://ollama.com/install.sh | sh"
curl -s localhost:11434/api/tags | grep -q llava-phi3 && ok "llava-phi3 model" || bad "llava-phi3" "ollama pull llava-phi3"
for m in ndn pydantic redis ultralytics cv2 scipy matplotlib yaml; do
  python3 -c "import $m" 2>/dev/null && ok "python: $m" || bad "python: $m" "pip install -r requirements.txt"
done
command -v tesseract >/dev/null && python3 -c "import pytesseract" 2>/dev/null && ok "Tesseract OCR (sign reading)" || bad "Tesseract OCR" "sudo apt install tesseract-ocr && pip install pytesseract"
python3 -c "import cogndn_msgs.msg, rclpy; rclpy.init(); n=rclpy.create_node('envchk'); from cogndn_msgs.msg import DetectionArray; n.create_publisher(DetectionArray,'/envchk',1).publish(DetectionArray())" >/dev/null 2>&1 && ok "cogndn_msgs load and publish" || bad "cogndn_msgs" "bash ~/dhikriya/fix_fastcdr.sh (Fast-CDR mismatch)"
python3 -c "import torch;assert torch.cuda.is_available()" 2>/dev/null && ok "CUDA for PyTorch" || bad "CUDA" "YOLO will fall back to CPU"
[ -f "$HOME/.cogndn/keys/road.key.der" ] && ok "NDN keys" || bad "NDN keys" "ros2 run cogndn_ndn make_keys"
sudo -n true 2>/dev/null && ok "passwordless sudo (for netns/netem)" || echo "  [INFO] sudo will ask for a password when the data plane starts"
exit $FAIL
