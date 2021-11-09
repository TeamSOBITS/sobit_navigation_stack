#include <sobit_navigation_library/sobit_navigation_library.hpp>
#include <sobit_navigation_library/sobit_navigation_library_python.hpp>

using namespace SOBITNavigationStack;
namespace py = pybind11;

PYBIND11_MODULE( sobit_navigation_module, m) {    
    py::class_<SOBITNavigationLibrary>(m, "SOBITNavigationLibrary")
        .def( py::init< const std::string& >() )
        .def_readwrite("exist_goal_", &SOBITNavigationLibrary::exist_goal_)
        .def_readwrite("status_id_", &SOBITNavigationLibrary::status_id_)
        .def( "move2Location", &SOBITNavigationLibrary::move2Location, "move to Location Pose", 
            py::arg("location_name"), py::arg("is_wait") = false )
        .def( "cancelMoving", &SOBITNavigationLibrary::cancelMoving, "cancel Moving" )
        .def( "clearCostmaps", &SOBITNavigationLibrary::clearCostmaps, "clear Costmaps" );

    py::class_<SOBITNavigationLibraryPython, SOBITNavigationLibrary>(m, "SOBITNavigationLibraryPython")
        .def( py::init< const std::string& >() )
        .def( "move2PositionPy", &SOBITNavigationLibraryPython::move2PositionPy, "move to Position", 
            py::arg("x"), py::arg("y"), py::arg("z"), py::arg("qx"), py::arg("qy"), py::arg("qz"), py::arg("qw"), py::arg("frame_id"), py::arg("is_wait") = false )
        .def( "addLocationPosePy", &SOBITNavigationLibraryPython::addLocationPosePy, "add Location Pose", 
            py::arg("name"), py::arg("frame_id"), py::arg("x"), py::arg("y"), py::arg("z"), py::arg("qx"), py::arg("qy"), py::arg("qz"), py::arg("qw") );
}