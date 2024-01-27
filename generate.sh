cd /home/sophda/project/OrbSlam3AR/arm64_build
rm -rf ./*
cmake-gui -S /home/sophda/project/OrbSlam3AR -B /home/sophda/project/OrbSlam3AR/arm64_build
make -j64
# cp /home/sophda/project/OrbSlam3AR/ORB_SLAM3/arm64_build/libORB_SLAM3.so /home/sophda/project/OrbSlam3AR/arm64_build/