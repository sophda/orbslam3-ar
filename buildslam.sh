cd /home/sophda/project/OrbSlam3AR/ORB_SLAM3/arm64_build
rm -rf ./*
cmake-gui -S /home/sophda/project/OrbSlam3AR/ORB_SLAM3 -B /home/sophda/project/OrbSlam3AR/ORB_SLAM3/arm64_build
make -j64