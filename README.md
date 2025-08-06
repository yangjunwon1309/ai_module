## Quick Start 
### - scene graph generation per frame(/camera/image/compressed)

```bash
cd ~/catkin_ws/src
git clone -b sgg --single-branch https://github.com/yangjunwon1309/ai_module.git ai_module
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# Unity + RViz + SGG 한 번에 (가중치 자동 다운로드)
roslaunch reltr_scene_graph system_with_sgg.launch \
  camera_topic:=/camera/image/compressed \
  use_compressed:=true \
  use_rviz:=true \
  predownload:=true
