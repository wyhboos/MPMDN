#include <boost/python.hpp>
#include "mppn.h"
#include <ompl/base/Planner.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/Planner.h>

BOOST_PYTHON_MODULE(MPN) {
    using namespace boost::python;

    class_<ompl::geometric::MPN, bases<ompl::base::Planner>>("MPN", init<const ompl::base::SpaceInformationPtr>())
        .def("test", &ompl::geometric::MPN::test)
        .def_readwrite("env_index", &ompl::geometric::MPN::env_index)
        .def_readwrite("nn_rep_cnt_lim", &ompl::geometric::MPN::nn_rep_cnt_lim)
        .def_readwrite("iter_cnt_lim", &ompl::geometric::MPN::iter_cnt_lim)
        .def_readwrite("state_type", &ompl::geometric::MPN::state_type)
        .def_readwrite("time_o", &ompl::geometric::MPN::time_o)
        .def_readwrite("time_nnrp", &ompl::geometric::MPN::time_nnrp)
        .def_readwrite("time_classical", &ompl::geometric::MPN::time_classical)
        .def_readwrite("forward_ori", &ompl::geometric::MPN::forward_ori)
        .def_readwrite("forward_nnrep", &ompl::geometric::MPN::forward_nnrep)
        .def_readwrite("invalid_o", &ompl::geometric::MPN::invalid_o)
        .def_readwrite("invalid_nnrep", &ompl::geometric::MPN::invalid_nnrep);


    class_<ompl::geometric::MPN, bases<ompl::base::Planner>>("MPMDN", init<const ompl::base::SpaceInformationPtr>())
        .def("test", &ompl::geometric::MPN::test)
        .def_readwrite("env_index", &ompl::geometric::MPN::env_index)
        .def_readwrite("nn_rep_cnt_lim", &ompl::geometric::MPN::nn_rep_cnt_lim)
        .def_readwrite("iter_cnt_lim", &ompl::geometric::MPN::iter_cnt_lim)
        .def_readwrite("state_type", &ompl::geometric::MPN::state_type)
        .def_readwrite("time_o", &ompl::geometric::MPN::time_o)
        .def_readwrite("time_nnrp", &ompl::geometric::MPN::time_nnrp)
        .def_readwrite("time_classical", &ompl::geometric::MPN::time_classical)
        .def_readwrite("forward_ori", &ompl::geometric::MPN::forward_ori)
        .def_readwrite("forward_nnrep", &ompl::geometric::MPN::forward_nnrep)
        .def_readwrite("invalid_o", &ompl::geometric::MPN::invalid_o)
        .def_readwrite("invalid_nnrep", &ompl::geometric::MPN::invalid_nnrep);


    

}
