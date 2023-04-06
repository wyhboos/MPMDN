#include <boost/python.hpp>
#include "mppn.h"
#include <ompl/base/Planner.h>
// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/Planner.h>
 
// often useful headers:

// #include <torch/script.h> // One-stop header.
// BOOST_PYTHON_MODULE(MPPN) {
//     using namespace boost::python;

//     class_<ompl::geometric::MPPN>("MPPN", init<std::string>())
//         .def("test", &ompl::geometric::MPPN::test);

// }

BOOST_PYTHON_MODULE(MPPN) {
    using namespace boost::python;

    class_<ompl::geometric::MPPN, bases<ompl::base::Planner>>("MPPN", init<const ompl::base::SpaceInformationPtr>())
        .def("test", &ompl::geometric::MPPN::test);

}

