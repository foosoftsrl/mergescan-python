from setuptools import Extension, setup
from distutils.command import build as build_module
import os
import shutil
import subprocess
import platform
from pybind11 import get_cmake_dir
from pybind11.setup_helpers import Pybind11Extension, build_ext

machine=platform.machine()
print(f"platform machine={machine}")

# This is the way in which CIBUILDWHEEL tells us what the architecture is on MACOS. There should be a way to get it in other places, I don't know it
archflags = os.environ.get("ARCHFLAGS", None)
print(f"archflags={archflags}")
if archflags == "-arch arm64":
  machine="arm64"
elif archflags == "-arch x86_64":
  machine="x86_64"
print(f"target machine={machine}")

buildscript=os.getcwd() + "/build_prerequisites.sh"
builddir=os.getcwd() + f"/build/{machine}"


class build(build_module.build):
  def run(self):
    os.makedirs(builddir,exist_ok=True)
    p = subprocess.Popen(["bash", buildscript, machine], cwd=builddir)
    p.wait()
    build_module.build.run(self)

with open("README.md", 'r') as f:
    long_description = f.read()

extra_link_args = []

# On Linux, we need -Wl,-Bsymbolic in order to statically link ffmpeg.
# See https://stackoverflow.com/questions/44379426/building-shared-library-with-ffmpeg-results-in-relocation-error
if platform.system() == "Linux":
    extra_link_args += [
      "-Wl,-Bsymbolic",
    ]

extra_link_args+=[
    f"-L{builddir}/sysroot/lib",
    f"-L{builddir}/sysroot/lib64",
    "-lOpen3D",
    "-lOpen3D_3rdparty_qhullcpp",
    "-lOpen3D_3rdparty_qhull_r",
    "-lOpen3D_3rdparty_jpeg",
    "-lOpen3D_3rdparty_png",
    "-lOpen3D_3rdparty_zlib",
    "-lOpen3D_3rdparty_jsoncpp",
    "-lswscale",
    "-lavutil"
]

if platform.system() == "Linux":
    extra_link_args += [
      "-lstdc++fs",
      "-lgomp",
    ]

if platform.system() == "Darwin":
  extra_link_args +=[
    "-framework","CoreFoundation",
    "-framework","CoreVideo"
    ]

print(f"extra_link_args = {extra_link_args}")

ext_modules = [
    Pybind11Extension("mergescan",
        ["src/main.cpp"],
        # Example: passing in the version to the compiled code
        #define_macros = [('VERSION_INFO', __version__)],
        cxx_std=17,
        include_dirs=[
           f"{builddir}/sysroot/include",
           f"{builddir}/sysroot/include/open3d/3rdparty",
           f"{builddir}/sysroot/include/open3d"
           ],
        extra_link_args = extra_link_args
        ),
]

setup(name='mergescan',
      version='0.0.0',
      description='Mergescan native module',
      long_description=long_description,
      long_description_content_type='text/markdown',
      license='MIT License',
      author='Luca Piccarreta',
      author_email='luca@topologyeyewaer.com',
      packages=[],
      include_package_data=True,
      install_requires=['cython'],
      extras_require={'pillow': ['Pillow']},
      python_requires='>=3.4',
      ext_modules=ext_modules,
      cmdclass = {
        'build': build,
      },
      classifiers=[
          'Development Status :: 3 - Alpha',
          'Intended Audience :: Developers',
          'Topic :: System :: Archiving :: Compression',
          'Topic :: Multimedia :: Graphics',
          'Topic :: Multimedia :: Graphics :: Graphics Conversion',
          'Operating System :: OS Independent',
          'License :: OSI Approved :: MIT License',
          'Programming Language :: Cython',
          'Programming Language :: Python :: 3'
      ]
)
