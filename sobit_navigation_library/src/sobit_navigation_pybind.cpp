#include <sobit_navigation_library/sobit_navigation_library.hpp>

using namespace SOBITNavigationStack;
namespace py = pybind11;

PYBIND11_MODULE( sobit_navigation_module, m) {    
    py::class_<SOBITNavigationLibrary>(m, "SOBITNavigationLibrary")
        .def( py::init<>() );
        // .def( "move2PositionPy", &SOBITNavigationLibrary::move2Position, "move to Position", 
        //     py::arg("x"), py::arg("y"), py::arg("z"), py::arg("qx"), py::arg("qy"), py::arg("qz"), py::arg("qw"), py::arg("frame_id"), py::arg("is_wait") = false )
        // .def( "move2Location", &SOBITNavigationLibrary::move2Location, "move to Location Pose", 
        //     py::arg("location_name"), py::arg("is_wait") = false )
        // .def( "cancelMoving", &SOBITNavigationLibrary::cancelMoving, "cancel Moving" );     
}