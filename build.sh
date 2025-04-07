cd /home/sophda/project/OrbSlam3AR/arm64_build
# rm -rf ./*
cmake \
    -DCMAKE_TOOLCHAIN_FILE=${NDK}/build/cmake/android.toolchain.cmake \
    -DANDROID_PLATFORM=android-30 \
    -DANDROID_ABI="arm64-v8a" \
    -DBUILD_ANDROID=false \
    ..  
make -j64
echo "build slam done"
rm /mnt/d/MyProject/ARBSLAM/Assets/Plugins/Android/libslamAR.so
cp /home/sophda/project/OrbSlam3AR/arm64_build/libslamAR.so /mnt/d/MyProject/ARBSLAM/Assets/Plugins/Android