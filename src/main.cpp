#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/pytypes.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/io/ImageIO.h>
#include <open3d/pipelines/registration/GeneralizedICP.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <geometry/BoundingVolume.h>
#include <geometry/Geometry.h>
#include <geometry/Image.h>
#include <geometry/RGBDImage.h>
#include <geometry/PointCloud.h>
#include "ThreadPool.h"

extern "C" {
#include <libswscale/swscale.h>
}

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

using namespace std;
using namespace open3d;
namespace py = pybind11;

ThreadPool threadPool(8);

// Create a Python object that will free the allocated memory when destroyed:
template <class T>
static py::array_t<T> makePyArray(const T* data, int size) {
    // Create a Python object that will free the allocated
    // memory when destroyed:
    std::unique_ptr<T[]> copy(new T[size]);
    memcpy(copy.get(), data, size * sizeof(T));
    py::capsule free_when_done(copy.get(), [](void *f) {
        T *dataPtr = reinterpret_cast<T *>(f);
        delete[] dataPtr;
    });
    return py::array_t<T>(
        {size}, // shape
        {sizeof(T)}, // C-style contiguous strides
        copy.release(),
        free_when_done); // numpy array references this parent
}

// Create a Python object that will free the allocated memory when destroyed:
template <class T>
static py::array_t<T> makePyArray(const T* data, int width, int height) {
    int size = width * height;
    std::unique_ptr<T[]> copy(new T[width * height]);
    memcpy(copy.get(), data, size * sizeof(T));
    py::capsule free_when_done(copy.get(), [](void *f) {
        T *dataPtr = reinterpret_cast<T *>(f);
        delete[] dataPtr;
    });
    return py::array_t<T>(
        {height, width}, // shape
        {width * sizeof(T), sizeof(T)}, // C-style contiguous strides
        copy.release(),
        free_when_done); // numpy array references this parent
}

template <class T>
static py::array_t<T> makePyArray(const T* data, int width, int height, int planes) {
    int size = width * height * planes;
    std::unique_ptr<T[]> copy(new T[size]);
    memcpy(copy.get(), data, size * sizeof(T));
    py::capsule free_when_done(copy.get(), [](void *f) {
        T *dataPtr = reinterpret_cast<T *>(f);
        delete[] dataPtr;
    });
    return py::array_t<T>(
        {height, width, 3}, // shape
        {width * planes * sizeof(T), planes * sizeof(T),sizeof(T) }, // C-style contiguous strides
        copy.release(),
        free_when_done); // numpy array references this parent
}

struct DepthImage : public JoinableFuture<geometry::Image> {
  DepthImage(JoinableFuture<geometry::Image>&& future) 
	  : JoinableFuture<geometry::Image>(future) 
  {
  }

  int width() {
    return get().width_;
  }

  int height() {
    return get().height_;
  }

  py::array_t<float> data() {
    return makePyArray((float*)get().data_.data(), width(), height());
  }

  static DepthImage fromData(py::array_t<float, py::array::c_style | py::array::forcecast> array) {
    pybind11::gil_scoped_release release;
    if(array.ndim() != 2)
      throw std::runtime_error("DepthImage::fromData wants a bidimensional array");
    py::buffer_info buf = array.request();

    geometry::Image depth;
    int width = buf.shape[1];
    int height = buf.shape[0];
    depth.Prepare(width, height, 1, sizeof(float));
    float* src = static_cast<float*>(buf.ptr);
    float* dst = reinterpret_cast<float*>(depth.data_.data());
    memcpy(dst, src, width * height * sizeof(float));
    auto future = JoinableFuture<geometry::Image>();
    future.setValue(depth);
    return future;
  }

  static DepthImage load(const std::string& path, float min, float max) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([path, min, max]() -> geometry::Image {
      geometry::Image img;
      if(!io::ReadImageFromPNG(path, img)) {
         throw std::runtime_error("Failed loading image");
      }
      if(img.bytes_per_channel_ != 2) {
        throw std::runtime_error("Not a 16 bit image");
      }
      int width = img.width_;
      int height = img.height_;
      geometry::Image depth;
      depth.Prepare(width, height, 1, sizeof(float));
      const uint16_t* src = (uint16_t*)img.data_.data();
      float* dest = (float*)depth.data_.data();
      float scale = (max - min) / 65535;
      for (int i = 0; i < height * width; i++) {
        uint16_t s = src[i];
        if(s == 0 || s == 65535)
          dest[i] = NAN;
        else
          dest[i] = min + s * scale;
      }
      return depth;
   });
  }
};

struct Image : public JoinableFuture<geometry::Image> {
  Image(JoinableFuture<geometry::Image>&& future) 
	  : JoinableFuture<geometry::Image>(future) 
  {
  }

  int width() {
    return get().width_;
  }

  int height() {
    return get().height_;
  }

  py::array_t<uint8_t> data() {
    return makePyArray((uint8_t*)get().data_.data(), width(), height(), get().num_of_channels_);
  }

  static Image load(const std::string& path) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([path]() -> geometry::Image {
      geometry::Image img;
      if(!io::ReadImageFromJPG(path, img)) {
         throw std::runtime_error("Failed loading image");
      }
      if(img.bytes_per_channel_ != 1) {
        throw std::runtime_error("Not a 8 bit image");
      }
      if(img.num_of_channels_ != 3) {
        throw std::runtime_error("Not a RGB image");
      }
      return img;
   });
  }

  Image resize(int width, int height) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, width, height]() -> geometry::Image {
      geometry::Image& selfImg = (geometry::Image&)Self.get();
      geometry::Image img;
      img.Prepare(width, height, 3, 1);
      auto resizeContext= sws_getContext(selfImg.width_, selfImg.height_, AV_PIX_FMT_RGB24, width, height, AV_PIX_FMT_RGB24, 0, NULL, NULL, NULL);
      uint8_t *srcPlanes[1] = {(uint8_t*)selfImg.data_.data()};
      uint8_t *dstPlanes[1] = {(uint8_t*)img.data_.data()};
      int srcStrides[1] = {selfImg.width_ * 3};
      int dstStrides[1] = {img.width_ * 3};
      sws_scale(resizeContext, srcPlanes, srcStrides, 0, selfImg.height_, dstPlanes, dstStrides);      
      sws_freeContext(resizeContext);
      return img;
   }, *this);
  }
};

struct PointCloud : public JoinableFuture<std::shared_ptr<geometry::PointCloud>> {
  PointCloud() {

  }

  PointCloud(JoinableFuture<std::shared_ptr<geometry::PointCloud>> &&future) : JoinableFuture<std::shared_ptr<geometry::PointCloud>>(future)
  {
  }

  PointCloud removeStatisticalOutliers(int nb_neighbors, double std_ratio)
  {
      pybind11::gil_scoped_release release;
      return threadPool.submit([future = *this, nb_neighbors, std_ratio]()
                             { return ::get<0>(future.get()->RemoveStatisticalOutliers(nb_neighbors, std_ratio, false)); },
                             *this);
  }

  py::array_t<double> points()
  {
    auto &points = get()->points_;
    return makePyArray((const double *)points.data(), 3, points.size());
  }

  py::array_t<double> normals()
  {
    auto &normals = get()->normals_;
    return makePyArray((const double *)normals.data(), 3, normals.size());
  }

  py::array_t<uint8_t> colors()
  {
    auto &colors = get()->colors_;
    return makePyArray((const uint8_t*)colors.data(), 3, colors.size());
  }

  size_t size()
  {
    return get()->points_.size();
  }

  static PointCloud fromDepth(DepthImage depth, double width, double height, double fx, double fy, double cx, double cy) {
    pybind11::gil_scoped_release release;
    camera::PinholeCameraIntrinsic intrinsics(width,height,fx,fy,cx,cy);
    return threadPool.submit([depth, intrinsics]() -> std::shared_ptr<geometry::PointCloud> {
        return geometry::PointCloud::CreateFromDepthImage(depth.get(), intrinsics, Eigen::Matrix4d::Identity());
    }, depth);
  }

  static PointCloud fromRGBD(Image img, DepthImage depth, double width, double height, double fx, double fy, double cx, double cy) {
    pybind11::gil_scoped_release release;
    camera::PinholeCameraIntrinsic intrinsics(width,height,fx,fy,cx,cy);
    return threadPool.submit([img, depth, intrinsics]() -> std::shared_ptr<geometry::PointCloud> {
        geometry::RGBDImage rgbd(img.get(), depth.get());
        return geometry::PointCloud::CreateFromRGBDImage(rgbd, intrinsics, Eigen::Matrix4d::Identity());
    }, depth, img);
  }

  friend PointCloud operator+(PointCloud p1, PointCloud p2) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([p1, p2]() -> std::shared_ptr<geometry::PointCloud> {
      auto transformed = std::make_shared<geometry::PointCloud>();
      *transformed.get() = *p1.get();
      *transformed.get() += *p2.get();
      return transformed;
    }, p1,p2);
  }
  
  PointCloud transform(const Eigen::Matrix4d& transform) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, transform]() -> std::shared_ptr<geometry::PointCloud> {
      auto transformed = std::make_shared<geometry::PointCloud>();
      transformed->points_ = Self.get()->points_;
      transformed->colors_ = Self.get()->colors_;
      transformed->normals_ = Self.get()->normals_;
      transformed->Transform(transform);
      return transformed;
    }, *this);
  }

  PointCloud crop(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, minX, minY, minZ, maxX, maxY, maxZ]() -> std::shared_ptr<geometry::PointCloud> {
      auto transformed = std::make_shared<geometry::PointCloud>();
      auto original = Self.get();
      auto& points = original->points_;
      auto& normals = original->normals_;
      auto& colors = original->colors_;
      for(size_t i = 0; i < points.size(); i++) {
        auto& point = points[i];
        if (point(0) >= minX && point(0) <= maxX &&
            point(1) >= minY && point(1) <= maxY &&
            point(2) >= minZ && point(2) <= maxZ) {
          transformed->points_.push_back(point);
          if(colors.size())
            transformed->colors_.push_back(colors[i]);
          if(normals.size())
            transformed->normals_.push_back(normals[i]);
        }
      }
      return transformed;
    }, *this);
  }

  PointCloud filterBackground(double backgroundDistance) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, backgroundDistance]() -> std::shared_ptr<geometry::PointCloud> {
      auto transformed = std::make_shared<geometry::PointCloud>();
      auto original = Self.get();
      auto& points = original->points_;
      auto& colors = original->colors_;
      double maxDepth = -1e7;
      for(const auto& point: points) {
        maxDepth = std::max(maxDepth, point(2));
      }
      double limit = maxDepth - backgroundDistance; 
      for(size_t i = 0; i < points.size(); i++) {
        auto& point = points[i];
        if (point(2) >= limit) {
          transformed->points_.push_back(point);
          if(!colors.empty()) {
            transformed->colors_.push_back(colors[i]);
          }
        }
      }
      return transformed;
    }, *this);
  }

  PointCloud estimateNormals(double radius, int max_nn) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, radius, max_nn]() -> std::shared_ptr<geometry::PointCloud> {
      auto transformed = std::make_shared<geometry::PointCloud>();
      auto original = Self.get();
      transformed->points_ = original->points_;
      transformed->colors_ = original->colors_;
      transformed->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius, max_nn));
      for(auto& normal: transformed->normals_) {
        if(normal(2) < 0) {
          normal = -normal;
        } 
      }
      return transformed;
    }, *this);
  }

  PointCloud filterByNormalAngle(double x, double y, double z, double threshold) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, x, y, z,threshold]() -> std::shared_ptr<geometry::PointCloud> {
      auto transformed = std::make_shared<geometry::PointCloud>();
      auto original = Self.get();
      // this is a hack...
      auto& normals = original->normals_;
      auto& points = original->points_;
      auto& colors = original->colors_;
      if(points.size() != normals.size()) {
        fprintf(stderr, "Uhm... normal and points arrays have different sizes: normals = %d points = %d\n", (int)normals.size(), (int)points.size());
        throw std::runtime_error("Filter with normals: Uhm... normal and points arrays have different sizes");
      }
      for(size_t i = 0; i < points.size(); i++) {
        auto& normal = normals[i];
        double scalar = normal(0) * x + normal(1) * y + normal(2) * z;
        if(scalar > threshold) {
          transformed->points_.push_back(points[i]);
          transformed->normals_.push_back(normals[i]);
          if(!colors.empty())
            transformed->colors_.push_back(colors[i]);
        }
      }
      return transformed;
    }, *this);
  }

  PointCloud selectByIndex(const std::vector<size_t> &indices) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, indices]() -> std::shared_ptr<geometry::PointCloud> {
      auto original = Self.get();
      auto filtered = std::make_shared<geometry::PointCloud>();
      const auto& orgNormals = original->normals_;
      const auto& orgPoints = original->points_;
      const auto& orgColors = original->colors_;
      for(const auto index : indices) {
          filtered->points_.push_back(orgPoints[index]);
          if(!orgNormals.empty())
            filtered->normals_.push_back(orgNormals[index]);
          if(!orgColors.empty())
            filtered->colors_.push_back(orgColors[index]);
      }
      return filtered;
    }, *this);
  }

  PointCloud voxelDownSample(double voxel_size) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, voxel_size]() -> std::shared_ptr<geometry::PointCloud> {
      return Self.get()->VoxelDownSample(voxel_size);
    }, *this);
  }
  PointCloud randomDownSample(double pct) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, pct]() -> std::shared_ptr<geometry::PointCloud> {
      return Self.get()->RandomDownSample(pct);
    }, *this);
  }
  PointCloud randomDownSampleMax(int count) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([Self = *this, count]() -> std::shared_ptr<geometry::PointCloud> {
      auto original = Self.get();
      if(original->points_.size() > count) {
        return original->RandomDownSample(double(count) / original->points_.size());
      } else {
        return original;
      }
    }, *this);
  }
};

struct RegistrationResult : public JoinableFuture<open3d::pipelines::registration::RegistrationResult> {
  RegistrationResult() {

  }

  RegistrationResult(JoinableFuture<open3d::pipelines::registration::RegistrationResult> &&future) : JoinableFuture<open3d::pipelines::registration::RegistrationResult>(future)
  {
  }
  Eigen::Matrix4d_u transformation() {
    return get().transformation_;
  }
  double fitness() {
    return get().fitness_;
  }
  double inlier_rmse() {
    return get().inlier_rmse_;
  }
};

RegistrationResult icp(PointCloud p1, PointCloud p2, double radius, const Eigen::Matrix4d& current_transformation, const open3d::pipelines::registration::ICPConvergenceCriteria& convergenceCriteria) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([p1, p2, radius,current_transformation, convergenceCriteria]() -> open3d::pipelines::registration::RegistrationResult {
    auto result = open3d::pipelines::registration::RegistrationICP(*p1.get(), *p2.get(), radius, current_transformation,
    open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
    convergenceCriteria
    );
    return result;
  }, p1, p2);
}

RegistrationResult coloredIcp(PointCloud p1, PointCloud p2, double radius, const Eigen::Matrix4d& current_transformation, const open3d::pipelines::registration::ICPConvergenceCriteria& convergenceCriteria) {
      pybind11::gil_scoped_release release;
      return threadPool.submit([p1, p2, radius,current_transformation, convergenceCriteria]() -> open3d::pipelines::registration::RegistrationResult {
    auto result = open3d::pipelines::registration::RegistrationColoredICP(*p1.get(), *p2.get(), radius, current_transformation,
    open3d::pipelines::registration::TransformationEstimationForColoredICP(0.95),
    convergenceCriteria
    );
    return result;
  }, p1, p2);
}


PYBIND11_MODULE(mergescan, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: mergescan

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";
    m.def("icp", &icp);
    m.def("coloredIcp", &coloredIcp);


    py::class_<open3d::pipelines::registration::ICPConvergenceCriteria>(
          m, "ICPConvergenceCriteria",
          "Class that defines the convergence criteria of ICP. ICP "
          "algorithm "
          "stops if the relative change of fitness and rmse hit "
          "``relative_fitness`` and ``relative_rmse`` individually, "
          "or the "
          "iteration number exceeds ``max_iteration``.")
          .def(py::init<double, double, int>())
          .def_readonly(
                  "relative_fitness",
                  &open3d::pipelines::registration::ICPConvergenceCriteria::relative_fitness_,
                  "If relative change (difference) of fitness score is lower "
                  "than ``relative_fitness``, the iteration stops.")
          .def_readonly(
                  "relative_rmse", &open3d::pipelines::registration::ICPConvergenceCriteria::relative_rmse_,
                  "If relative change (difference) of inliner RMSE score is "
                  "lower than ``relative_rmse``, the iteration stops.")
          .def_readonly("max_iteration",
                          &open3d::pipelines::registration::ICPConvergenceCriteria::max_iteration_,
                          "Maximum iteration before iteration stops.");

    py::class_<PointCloud>(m, "PointCloud")
      .def_static("fromDepth", &PointCloud::fromDepth, py::return_value_policy::copy,
        py::arg("depth"),
        py::arg("width"),py::arg("height"),
        py::arg("fx"),py::arg("fy"),
        py::arg("cx"),py::arg("cy"))
      .def_static("fromRGBD", &PointCloud::fromRGBD, py::return_value_policy::copy,
        py::arg("color"),
        py::arg("depth"),
        py::arg("width"),py::arg("height"),
        py::arg("fx"),py::arg("fy"),
        py::arg("cx"),py::arg("cy"))
      .def("size", &PointCloud::size)
      .def_property_readonly("points", &PointCloud::points)
      .def_property_readonly("normals", &PointCloud::normals)
      .def_property_readonly("colors", &PointCloud::colors)
      .def(py::self + py::self)
      .def("crop", &PointCloud::crop,
	   py::arg("minX") = std::numeric_limits<double>::lowest(),
	   py::arg("minY") = std::numeric_limits<double>::lowest(),
           py::arg("minZ") = std::numeric_limits<double>::lowest(),
           py::arg("maxX") = std::numeric_limits<double>::max(),
           py::arg("maxY") = std::numeric_limits<double>::max(),
           py::arg("maxZ") = std::numeric_limits<double>::max())
      .def("transform", &PointCloud::transform)
      .def("estimateNormals", &PointCloud::estimateNormals,
        py::arg("radius") = 2.5,
        py::arg("max_nn") = 10
      )
      .def("filterBackground", &PointCloud::filterBackground)
      .def("filterByNormalAngle", &PointCloud::filterByNormalAngle)
      .def("select_by_index", &PointCloud::selectByIndex)
      .def("voxelDownSample", &PointCloud::voxelDownSample)
      .def("randomDownSample", &PointCloud::randomDownSample)
      .def("randomDownSampleMax", &PointCloud::randomDownSampleMax)
      .def("removeStatisticalOutliers", &PointCloud::removeStatisticalOutliers,
        py::arg("nb_neighbors") = 10,
        py::arg("std_ratio") = 1
      );

    py::class_<DepthImage>(m, "DepthImage")
      .def_static("load", &DepthImage::load)
      .def_static("fromData", &DepthImage::fromData)
      .def_property_readonly("data", &DepthImage::data)
      .def_property_readonly("width", &DepthImage::width)
      .def_property_readonly("height", &DepthImage::height);

    py::class_<Image>(m, "Image")
      .def_static("load", &Image::load)
      .def("resize", &Image::resize)
      .def_property_readonly("data", &Image::data)
      .def_property_readonly("width", &Image::width)
      .def_property_readonly("height", &Image::height);

    py::class_<RegistrationResult>(m, "RegistrationResult")
      .def_property_readonly("transformation", &RegistrationResult::transformation)
      .def_property_readonly("fitness", &RegistrationResult::fitness)
      .def_property_readonly("inlier_rmse", &RegistrationResult::inlier_rmse);

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
