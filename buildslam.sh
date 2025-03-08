# cd /home/sophda/project/OrbSlam3AR/ORB_SLAM3/arm64_build
# rm -rf ./*
# cmake-gui -S /home/sophda/project/OrbSlam3AR/ORB_SLAM3 -B /home/sophda/project/OrbSlam3AR/ORB_SLAM3/arm64_build
# make -j64

read -r -p "Build slam? [y/n] " buildslam

case ${buildslam} in 
    [yY])
        echo "build slam..."
        cd /home/sophda/project/OrbSlam3AR/ORB_SLAM3/arm64_build
        rm -rf ./*
        cmake \
            -DCMAKE_TOOLCHAIN_FILE=${NDK}/build/cmake/android.toolchain.cmake \
            -DANDROID_PLATFORM=android-30 \
            -DANDROID_ABI="arm64-v8a" \
            /home/sophda/project/OrbSlam3AR/ORB_SLAM3
        make -j64
        echo "build slam done"
        rm /mnt/d/MyProject/FakeGenshin/Assets/Plugins/Android/libORB_SLAM3.so
        cp /home/sophda/project/OrbSlam3AR/ORB_SLAM3/arm64_build/libORB_SLAM3.so /mnt/d/MyProject/FakeGenshin/Assets/Plugins/Android
esac

read -r -p "Build native library [y/n]" buildnative
case ${buildnative} in 
    [yY])   
        echo "build native..."

        cd /home/sophda/project/OrbSlam3AR/arm64_build
        rm -rf ./*
        cmake \
            -DCMAKE_TOOLCHAIN_FILE=${NDK}/build/cmake/android.toolchain.cmake \
            -DANDROID_PLATFORM=android-30 \
            -DANDROID_ABI="arm64-v8a" \
            /home/sophda/project/OrbSlam3AR
        make -j64
        echo "build native done"

        rm /mnt/d/MyProject/FakeGenshin/Assets/Plugins/Android/libslamAR.so
        cp /home/sophda/project/OrbSlam3AR/arm64_build/libslamAR.so /mnt/d/MyProject/FakeGenshin/Assets/Plugins/Android
esac



# make -j64

