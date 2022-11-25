from setuptools import Extension, setup
from Cython.Build import cythonize
from distutils.command import build as build_module
import os
import subprocess
import platform

machine=platform.machine()
print(f"platform machine={machine}")

# This is the way in which CIBUILDWHEEL tells us what the architecture is on MACOS. There should be a way to get it in other places, I don't know it
archflags = os.environ.get("ARCHFLAGS", None)
print(f"archflags={archflags}")
if archflags == "-arch arm64":
  machine="arm64"
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

jxlpy_ext = Extension(
    name="_jxlpy",
    sources=["_jxlpy/_jxl.pyx"],
    include_dirs=[f"{builddir}/sysroot/include"],
    extra_compile_args=['-O2'],
    extra_link_args=[
        f"-L{builddir}/sysroot/lib",
        f"-L{builddir}/sysroot/lib64",
        "-ljxl","-ljxl_threads","-lbrotlidec-static","-lbrotlienc-static","-lbrotlicommon-static","-lhwy"],
    language='c++',
)


setup(name='mergescan',
      version='0.0.0',
      description='Mergescan native module',
      long_description=long_description,
      long_description_content_type='text/markdown',
      license='MIT License',
      author='Luca Piccarreta',
      author_email='luca@topologyeyewaer.com',
      packages=['mergescan'],
      package_data={
          'jxlpy': ['*.pyx', '*.py'],
          '': ['README.md']
      },
      include_package_data=True,
      install_requires=['cython'],
      extras_require={'pillow': ['Pillow']},
      python_requires='>=3.4',
      ext_modules=cythonize([jxlpy_ext]),
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
