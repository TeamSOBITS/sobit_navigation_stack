#ifndef SOBIT_NAVIGATION_LIBRARY_PYTHON
#define SOBIT_NAVIGATION_LIBRARY_PYTHON

#include <sobit_navigation_library/sobit_navigation_library.hpp>
#include <pybind11/pybind11.h>

namespace SOBITNavigationStack {
    class SOBITNavigationLibraryPython : public SOBITNavigationLibrary{
        public :
            // コンストラクタ
            SOBITNavigationLibraryPython( const std::string &name );
            
            // 移動したい位置に移動する(Pybind用)
            bool move2PositionPy(
                const double x,
                const double y,
                const double z,
                const double qx,
                const double qy,
                const double qz,
                const double qw,
                const std::string& frame_id,
                const bool is_wait = false );
    };
}

#endif
