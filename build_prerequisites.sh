#!/bin/bash
set -e

BUILDDIR=$(readlink -f $PWD)
ARCHITECTURE=$1

echo builddir=$BUILDDIR

if [[ "$(uname)" == 'Darwin' ]]; then
  alias nproc="sysctl -n hw.logicalcpu" # As opposed to `hw.physicalcpu`
fi

function checkout_ffmpeg() {
  pushd $BUILDDIR
  if [ ! -d "FFmpeg" ] ; then
    git clone https://github.com/FFmpeg/FFmpeg.git
  fi
  cd FFmpeg
  git checkout release/5.1
  popd

}

function checkout_open3d() {
  pushd $BUILDDIR
  if [ ! -d "Open3D" ] ; then
    git clone https://github.com/isl-org/Open3D.git
  fi
  cd Open3D
  git checkout v0.16.1
  popd
}

function build_open3d() {
  pushd $BUILDDIR/Open3D
  git checkout v0.16.1
  mkdir -p build
  cd build
  cmake -DCMAKE_MACOSX_RPATH=0 -DCMAKE_CXX_FLAGS="-I $BUILDDIR/libjxl/lib/include/jxl" \
          -DCMAKE_OSX_ARCHITECTURES=$ARCHITECTURE \
          -DBUILD_GUI=OFF \
	  -DBUILD_PYTHON_MODULE=OFF \
          -DWITH_OPENMP=ON \
          -DWITH_SIMD=ON \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_JUPYTER_EXTENSION=OFF \
          -DBUILD_WEBRTC=OFF \
          -DDEVELOPER_BUILD=OFF \
          -DBUILD_COMMON_ISPC_ISAS=ON \
          -DUSE_SYSTEM_BLAS=OFF \
          -DGLIBCXX_USE_CXX11_ABI=1 \
          -DCMAKE_INSTALL_PREFIX=$BUILDDIR/sysroot ..
  make -j$(nproc) install
  #We want a static build, let's remove all shared objects
  #Note that we must delete lib64 for Centos
  #rm -f $BUILDDIR/sysroot/lib*/*dylib $BUILDDIR/sysroot/lib*/*so
  popd
}

function build_ffmpeg() {
  pushd $BUILDDIR/FFmpeg
  mkdir -p build
  cd build
  ../configure --prefix=$BUILDDIR/sysroot --disable-avcodec --disable-avformat --disable-avfilter --disable-swresample --extra-cflags="-fPIC"
  make -j$(nproc) install
  #We want a static build, let's remove all shared objects
  #Note that we must delete lib64 for Centos
  #rm -f $BUILDDIR/sysroot/lib*/*dylib $BUILDDIR/sysroot/lib*/*so
  popd
}

checkout_ffmpeg
build_ffmpeg
checkout_open3d
build_open3d

