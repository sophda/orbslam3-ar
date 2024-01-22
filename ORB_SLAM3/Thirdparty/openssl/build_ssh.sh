#!/bin/bash
set -e
set -x
# 改成自己路径
SCRIPTPATH=/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/openssl
export ANDROID_NDK_ROOT=/home/sophda/src/android-ndk-r25c
export OPENSSL_DIR=/home/sophda/project/OrbSlam3AR/ORB_SLAM3/Thirdparty/openssl/
toolchains_path=/home/sophda/src/android-ndk-r25c/toolchains/llvm/prebuilt/linux-x86_64
# 这里用的最新的ndk-r24版本使用clang编译。低版本ndk自行修改成gcc编译
CC=clang
PATH=$toolchains_path/bin:$PATH
# 自行修改Android版本
ANDROID_API=21
# 64位so：arm64-v8a
outdir=armeabi-v7a
# 64位so：android-arm64
architecture=android-arm
cd ${OPENSSL_DIR}
# make clean
./Configure ${architecture} -D__ANDROID_API__=$ANDROID_API
make
OUTPUT_INCLUDE=$SCRIPTPATH/output/include
OUTPUT_LIB=$SCRIPTPATH/output/lib/${outdir}
mkdir -p $OUTPUT_INCLUDE
mkdir -p $OUTPUT_LIB
cp -RL include/openssl $OUTPUT_INCLUDE
cp libcrypto.so $OUTPUT_LIB
cp libcrypto.a $OUTPUT_LIB
cp libssl.so $OUTPUT_LIB
cp libssl.a $OUTPUT_LIB
