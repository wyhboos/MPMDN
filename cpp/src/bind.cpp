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
        .def_readwrite("env_index", &ompl::geometric::MPPN::env_index)
        .def_readwrite("nn_rep_cnt_lim", &ompl::geometric::MPPN::nn_rep_cnt_lim)
        .def_readwrite("iter_cnt_lim", &ompl::geometric::MPPN::iter_cnt_lim)
        .def_readwrite("state_type", &ompl::geometric::MPPN::state_type)
        .def_readwrite("time_o", &ompl::geometric::MPPN::time_o)
        .def_readwrite("time_nnrp", &ompl::geometric::MPPN::time_nnrp)
        .def_readwrite("time_classical", &ompl::geometric::MPPN::time_classical)
        .def_readwrite("forward_ori", &ompl::geometric::MPPN::forward_ori)
        .def_readwrite("forward_nnrep", &ompl::geometric::MPPN::forward_nnrep)
        .def_readwrite("invalid_o", &ompl::geometric::MPPN::invalid_o)
        .def_readwrite("invalid_nnrep", &ompl::geometric::MPPN::invalid_nnrep);

}
