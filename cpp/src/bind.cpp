#include <boost/python.hpp>
#include "mppn.h"
#include <ompl/base/Planner.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/Planner.h>

BOOST_PYTHON_MODULE(MPPN) {
    using namespace boost::python;

    class_<ompl::geometric::MPPN, bases<ompl::base::Planner>>("MPPN", init<const ompl::base::SpaceInformationPtr>())
        .def("test", &ompl::geometric::MPPN::test)
        .def_readwrite("state_type", &ompl::geometric::MPPN::state_type)
        .def_readwrite("time_o", &ompl::geometric::MPPN::time_o);

}
