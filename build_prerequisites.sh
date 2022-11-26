#!/bin/bash
set -e

BUILDDIR=$(readlink -f $PWD)
ARCHITECTURE=$1

echo builddir=$BUILDDIR

if [[ "$(uname)" == 'Darwin' ]]; then
  NPROC=$(sysctl -n hw.logicalcpu) # As opposed to `hw.physicalcpu`
  ARCHFLAGS="-arch $ARCHITECTURE"
else
  NPROC=$(nproc)
  ARCHFLAGS=""
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
  # This makes no sense... it will fail if we compile twice
  # I may git reset hard
  git reset --hard
  git apply "$BUILDDIR/open3d.patch"
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
          -DUSE_SYSTEM_BLAS=OFF \
          -DBUILD_ISPC_MODULE=OFF \
          -DBUILD_COMMON_ISPC_ISAS=OFF \
          -DGLIBCXX_USE_CXX11_ABI=1 \
          -DCMAKE_INSTALL_PREFIX=$BUILDDIR/sysroot ..
  make -j$NPROC install
  #We want a static build, let's remove all shared objects
  #Note that we must delete lib64 for Centos
  #rm -f $BUILDDIR/sysroot/lib*/*dylib $BUILDDIR/sysroot/lib*/*so
  popd
}

function build_ffmpeg() {
  pushd $BUILDDIR/FFmpeg
  mkdir -p build
  cd build
  # I could use --enable-pic instead of --extra-cflags="-fPIC"
  # I've no idea if this -fPIC is needed for macos
  CONFIGOPTIONS=(
     --prefix=$BUILDDIR/sysroot
     --arch=$ARCHITECTURE
     --enable-cross-compile
     --extra-cflags="$ARCHFLAGS -fPIC"
     --extra-ldflags="$ARCHFLAGS"
     --disable-avcodec
     --disable-avformat
     --disable-avfilter
     --disable-swresample
  )
  # Probably this is needed because we're "cross-compiling" on macos
  if [[ "$(uname)" == 'Darwin' ]]; then
    CONFIGOPTIONS += "--target-os=darwin"

  ../configure ${CONFIGOPTIONS[@]}
  make V=1 -j$NPROC install
  #We want a static build, let's remove all shared objects
  #Note that we must delete lib64 for Centos
  #rm -f $BUILDDIR/sysroot/lib*/*dylib $BUILDDIR/sysroot/lib*/*so
  popd
}

checkout_ffmpeg
build_ffmpeg
checkout_open3d
build_open3d

