rm -rf /home/sophda/project/OrbSlam3AR/ACE-Libtorch/build/*
rm -rf /home/sophda/project/OrbSlam3AR/ACE-Libtorch/dsacstar/build/*


cd /home/sophda/project/OrbSlam3AR/arm64_build
rm -rf ./*
# make clean
cmake \
    -DCMAKE_TOOLCHAIN_FILE=${NDK27}/build/cmake/android.toolchain.cmake \
    -DANDROID_PLATFORM=android-30 \
    -DANDROID_ABI="arm64-v8a" \
    -DBUILD_ANDROID=false \
    ..  
make -j64
echo "build slam done"
rm /mnt/d/MyProject/ARBSLAM/Assets/Plugins/Android/libslamAR.so
rm /mnt/d/MyProject/ARBSLAM/Assets/Plugins/Android/libace.so

cp /home/sophda/project/OrbSlam3AR/arm64_build/libslamAR.so /mnt/d/MyProject/ARBSLAM/Assets/Plugins/Android
cp /home/sophda/project/OrbSlam3AR/ACE-Libtorch/build/libace.so /mnt/d/MyProject/ARBSLAM/Assets/Plugins/Android

echo "copy done~ "