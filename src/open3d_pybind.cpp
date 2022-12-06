#define PYBIND11_DETAILED_ERROR_MESSAGES

#include "pybind/open3d_pybind.h"

#include "open3d/core/MemoryManagerStatistic.h"
#include "open3d/utility/Logging.h"
#include "open3d/geometry/Geometry3D.h"
#include "pybind/camera/camera.h"
#include "pybind/core/core.h"
#include "pybind/data/dataset.h"
#include "pybind/geometry/geometry.h"
#include "pybind/geometry/geometry_trampoline.h"
#include "pybind/io/io.h"
#include "pybind/ml/ml.h"
#include "pybind/pipelines/pipelines.h"
#include "pybind/t/t.h"
#include "pybind/utility/utility.h"
#include "pybind/pipelines/registration/registration.h"
#include "pybind/visualization/visualization.h"
#include "pybind/docstring.h"

using namespace open3d::geometry;

namespace open3d {

static void pybind_geometry_classes(py::module &m) {
    // open3d.geometry functions
    m.def("get_rotation_matrix_from_xyz", &Geometry3D::GetRotationMatrixFromXYZ,
          "rotation"_a);
    m.def("get_rotation_matrix_from_yzx", &Geometry3D::GetRotationMatrixFromYZX,
          "rotation"_a);
    m.def("get_rotation_matrix_from_zxy", &Geometry3D::GetRotationMatrixFromZXY,
          "rotation"_a);
    m.def("get_rotation_matrix_from_xzy", &Geometry3D::GetRotationMatrixFromXZY,
          "rotation"_a);
    m.def("get_rotation_matrix_from_zyx", &Geometry3D::GetRotationMatrixFromZYX,
          "rotation"_a);
    m.def("get_rotation_matrix_from_yxz", &Geometry3D::GetRotationMatrixFromYXZ,
          "rotation"_a);
    m.def("get_rotation_matrix_from_axis_angle",
          &Geometry3D::GetRotationMatrixFromAxisAngle, "rotation"_a);
    m.def("get_rotation_matrix_from_quaternion",
          &Geometry3D::GetRotationMatrixFromQuaternion, "rotation"_a);

    // open3d.geometry.Geometry
    py::class_<Geometry, PyGeometry<Geometry>, std::shared_ptr<Geometry>>
            geometry(m, "Geometry", "The base geometry class.");
    geometry.def("clear", &Geometry::Clear,
                 "Clear all elements in the geometry.")
            .def("is_empty", &Geometry::IsEmpty,
                 "Returns ``True`` iff the geometry is empty.")
            .def("get_geometry_type", &Geometry::GetGeometryType,
                 "Returns one of registered geometry types.")
            .def("dimension", &Geometry::Dimension,
                 "Returns whether the geometry is 2D or 3D.");
    docstring::ClassMethodDocInject(m, "Geometry", "clear");
    docstring::ClassMethodDocInject(m, "Geometry", "is_empty");
    docstring::ClassMethodDocInject(m, "Geometry", "get_geometry_type");
    docstring::ClassMethodDocInject(m, "Geometry", "dimension");

    // open3d.geometry.Geometry.Type
    py::enum_<Geometry::GeometryType> geometry_type(geometry, "Type",
                                                    py::arithmetic());
    // Trick to write docs without listing the members in the enum class again.
    geometry_type.attr("__doc__") = docstring::static_property(
            py::cpp_function([](py::handle arg) -> std::string {
                return "Enum class for Geometry types.";
            }),
            py::none(), py::none(), "");

    geometry_type.value("Unspecified", Geometry::GeometryType::Unspecified)
            .value("PointCloud", Geometry::GeometryType::PointCloud)
            .value("VoxelGrid", Geometry::GeometryType::VoxelGrid)
            .value("LineSet", Geometry::GeometryType::LineSet)
            .value("TriangleMesh", Geometry::GeometryType::TriangleMesh)
            .value("HalfEdgeTriangleMesh",
                   Geometry::GeometryType::HalfEdgeTriangleMesh)
            .value("Image", Geometry::GeometryType::Image)
            .value("RGBDImage", Geometry::GeometryType::RGBDImage)
            .value("TetraMesh", Geometry::GeometryType::TetraMesh)
            .export_values();

    py::class_<Geometry3D, PyGeometry3D<Geometry3D>,
               std::shared_ptr<Geometry3D>, Geometry>
            geometry3d(m, "Geometry3D",
                       "The base geometry class for 3D geometries.");
    geometry3d
            .def("get_min_bound", &Geometry3D::GetMinBound,
                 "Returns min bounds for geometry coordinates.")
            .def("get_max_bound", &Geometry3D::GetMaxBound,
                 "Returns max bounds for geometry coordinates.")
            .def("get_center", &Geometry3D::GetCenter,
                 "Returns the center of the geometry coordinates.")
            .def("get_axis_aligned_bounding_box",
                 &Geometry3D::GetAxisAlignedBoundingBox,
                 "Returns an axis-aligned bounding box of the geometry.")
            .def("get_oriented_bounding_box",
                 &Geometry3D::GetOrientedBoundingBox, "robust"_a = false,
                 R"doc(
Returns the oriented bounding box for the geometry.
Computes the oriented bounding box based on the PCA of the convex hull.
The returned bounding box is an approximation to the minimal bounding box.
Args:
     robust (bool): If set to true uses a more robust method which works in 
          degenerate cases but introduces noise to the points coordinates.
Returns:
     open3d.geometry.OrientedBoundingBox: The oriented bounding box. The
     bounding box is oriented such that the axes are ordered with respect to
     the principal components.
)doc")
            .def("transform", &Geometry3D::Transform,
                 "Apply transformation (4x4 matrix) to the geometry "
                 "coordinates.")
            .def("translate", &Geometry3D::Translate,
                 "Apply translation to the geometry coordinates.",
                 "translation"_a, "relative"_a = true)
            .def("scale",
                 (Geometry3D &
                  (Geometry3D::*)(const double, const Eigen::Vector3d &)) &
                         Geometry3D::Scale,
                 "Apply scaling to the geometry coordinates.", "scale"_a,
                 "center"_a)
            .def("scale", &Geometry3D::Scale,
                 "Apply scaling to the geometry coordinates.", "scale"_a,
                 "center"_a)
            .def("rotate",
                 py::overload_cast<const Eigen::Matrix3d &>(
                         &Geometry3D::Rotate),
                 "Apply rotation to the geometry coordinates and normals.",
                 "R"_a)
            .def("rotate",
                 py::overload_cast<const Eigen::Matrix3d &,
                                   const Eigen::Vector3d &>(
                         &Geometry3D::Rotate),
                 "Apply rotation to the geometry coordinates and normals.",
                 "R"_a, "center"_a)
            .def_static("get_rotation_matrix_from_xyz",
                        &Geometry3D::GetRotationMatrixFromXYZ, "rotation"_a)
            .def_static("get_rotation_matrix_from_yzx",
                        &Geometry3D::GetRotationMatrixFromYZX, "rotation"_a)
            .def_static("get_rotation_matrix_from_zxy",
                        &Geometry3D::GetRotationMatrixFromZXY, "rotation"_a)
            .def_static("get_rotation_matrix_from_xzy",
                        &Geometry3D::GetRotationMatrixFromXZY, "rotation"_a)
            .def_static("get_rotation_matrix_from_zyx",
                        &Geometry3D::GetRotationMatrixFromZYX, "rotation"_a)
            .def_static("get_rotation_matrix_from_yxz",
                        &Geometry3D::GetRotationMatrixFromYXZ, "rotation"_a)
            .def_static("get_rotation_matrix_from_axis_angle",
                        &Geometry3D::GetRotationMatrixFromAxisAngle,
                        "rotation"_a)
            .def_static("get_rotation_matrix_from_quaternion",
                        &Geometry3D::GetRotationMatrixFromQuaternion,
                        "rotation"_a);
    docstring::ClassMethodDocInject(m, "Geometry3D", "get_min_bound");
    docstring::ClassMethodDocInject(m, "Geometry3D", "get_max_bound");
    docstring::ClassMethodDocInject(m, "Geometry3D", "get_center");
    docstring::ClassMethodDocInject(m, "Geometry3D",
                                    "get_axis_aligned_bounding_box");
    docstring::ClassMethodDocInject(m, "Geometry3D", "transform");
    docstring::ClassMethodDocInject(
            m, "Geometry3D", "translate",
            {{"translation", "A 3D vector to transform the geometry"},
             {"relative",
              "If true, the translation vector is directly added to the "
              "geometry "
              "coordinates. Otherwise, the center is moved to the translation "
              "vector."}});
    docstring::ClassMethodDocInject(
            m, "Geometry3D", "scale",
            {{"scale",
              "The scale parameter that is multiplied to the points/vertices "
              "of the geometry."},
             {"center", "Scale center used for transformation."}});
    docstring::ClassMethodDocInject(
            m, "Geometry3D", "rotate",
            {{"R", "The rotation matrix"},
             {"center", "Rotation center used for transformation."}});

    // open3d.geometry.Geometry2D
    py::class_<Geometry2D, PyGeometry2D<Geometry2D>,
               std::shared_ptr<Geometry2D>, Geometry>
            geometry2d(m, "Geometry2D",
                       "The base geometry class for 2D geometries.");
    geometry2d
            .def("get_min_bound", &Geometry2D::GetMinBound,
                 "Returns min bounds for geometry coordinates.")
            .def("get_max_bound", &Geometry2D::GetMaxBound,
                 "Returns max bounds for geometry coordinates.");
    docstring::ClassMethodDocInject(m, "Geometry2D", "get_min_bound");
    docstring::ClassMethodDocInject(m, "Geometry2D", "get_max_bound");
}

static void pybind_geometry(py::module& m) {
    py::module m_submodule = m.def_submodule("geometry");

    pybind_geometry_classes(m_submodule);
    open3d::geometry::pybind_kdtreeflann(m_submodule);
    open3d::geometry::pybind_meshbase(m_submodule);
    open3d::geometry::pybind_pointcloud(m_submodule);
    open3d::geometry::pybind_trianglemesh(m_submodule);
    pybind_image(m_submodule);
    open3d::geometry::pybind_boundingvolume(m_submodule);
}

static void pybind_pipelines(py::module& m) {
    py::module m_pipelines = m.def_submodule("pipelines");
    open3d::pipelines::registration::pybind_registration(m_pipelines);
}

PYBIND11_MODULE(open3d, m) {
    utility::Logger::GetInstance().SetPrintFunction([](const std::string& msg) {
        py::gil_scoped_acquire acquire;
        py::print(msg);
    });

    m.doc() = "Python binding of Open3D";

    // Check Open3D CXX11_ABI with
    // import open3d as o3d; print(o3d.open3d_pybind._GLIBCXX_USE_CXX11_ABI)
    m.add_object("_GLIBCXX_USE_CXX11_ABI",
                 _GLIBCXX_USE_CXX11_ABI ? Py_True : Py_False);

    // The binding order matters: if a class haven't been binded, binding the
    // user of this class will result in "could not convert default argument
    // into a Python object" error.

    utility::pybind_utility(m);
    camera::pybind_camera(m);
    pybind_geometry(m);
    pybind_pipelines(m);
    // pybind11 will internally manage the lifetime of default arguments for
    // function bindings. Since these objects will live longer than the memory
    // manager statistics, the latter will report leaks. Reset the statistics to
    // ignore them and transfer the responsibility to pybind11.
    core::MemoryManagerStatistic::GetInstance().Reset();
}

}  // namespace open3d
