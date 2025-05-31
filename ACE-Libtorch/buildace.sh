# cd /home/sophda/project/VinsAR/MonoRelocalization/androidbuild
# rm -rf ./*
# cmake \
#     -DCMAKE_TOOLCHAIN_FILE=${NDK27}/build/cmake/android.toolchain.cmake \
#     -DANDROID_PLATFORM=android-30 \
# 	-DANDROID_ABI=arm64-v8a \
#     ..

# make clean && make -j20 

cd /home/sophda/project/VinsAR/MonoRelocalization/build
# rm -rf ./*
cmake \
    -DBUILD_ANDROID=FALSE \
    ..

make clean && make -j20 