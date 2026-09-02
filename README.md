This file uses python3.6.9 which is the default for this jetson nano

use docker
# Pull the specific image for your older JetPack version
sudo docker pull ultralytics/ultralytics:latest-jetson-jetpack4

# Run the container
sudo docker run -itd \
  --name jetson_core \
  --restart unless-stopped \
  --ipc=host \
  --runtime=nvidia \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v /home/sreeram/major_project:/workspace \
  jetson-bridge:latest-v2
# Convert AI model
yolo export model=ai_models/boxes_ai.pt format=engine half=true device=0 opset=12 simplify=True

# chmod +x js-bridge.sh
# sudo cp js-bridge.sh /usr/local/bin/bridge
