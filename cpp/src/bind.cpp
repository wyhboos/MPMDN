#include <boost/python.hpp>
#include "mpn.h"
#include "mpmdn.h"

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
        .def_readwrite("use_orcle", &ompl::geometric::MPN::use_orcle)
        .def_readwrite("orcle_time_lim", &ompl::geometric::MPN::orcle_time_lim)
        .def_readwrite("ori_simplify", &ompl::geometric::MPN::ori_simplify)
        .def_readwrite("time_o", &ompl::geometric::MPN::time_o)
        .def_readwrite("time_nnrp", &ompl::geometric::MPN::time_nnrp)
        .def_readwrite("time_classical", &ompl::geometric::MPN::time_classical)
        .def_readwrite("time_simplify", &ompl::geometric::MPN::time_simplify)
        .def_readwrite("time_all", &ompl::geometric::MPN::time_all)
        .def_readwrite("forward_ori", &ompl::geometric::MPN::forward_ori)
        .def_readwrite("forward_nnrep", &ompl::geometric::MPN::forward_nnrep)
        .def_readwrite("invalid_o", &ompl::geometric::MPN::invalid_o)
        .def_readwrite("invalid_nnrep", &ompl::geometric::MPN::invalid_nnrep)
        .def_readwrite("colli_o", &ompl::geometric::MPN::colli_o)
        .def_readwrite("colli_nnrep", &ompl::geometric::MPN::colli_nnrep);


    class_<ompl::geometric::MPMDN, bases<ompl::base::Planner>>("MPMDN", init<const ompl::base::SpaceInformationPtr>())
        .def("test", &ompl::geometric::MPMDN::test)
        .def_readwrite("env_index", &ompl::geometric::MPMDN::env_index)
        .def_readwrite("nn_rep_cnt_lim", &ompl::geometric::MPMDN::nn_rep_cnt_lim)
        .def_readwrite("iter_cnt_lim", &ompl::geometric::MPMDN::iter_cnt_lim)
        .def_readwrite("state_type", &ompl::geometric::MPMDN::state_type)
        .def_readwrite("use_orcle", &ompl::geometric::MPMDN::use_orcle)
        .def_readwrite("orcle_time_lim", &ompl::geometric::MPMDN::orcle_time_lim)
        .def_readwrite("ori_simplify", &ompl::geometric::MPMDN::ori_simplify)
        .def_readwrite("time_o", &ompl::geometric::MPMDN::time_o)
        .def_readwrite("time_nnrp", &ompl::geometric::MPMDN::time_nnrp)
        .def_readwrite("time_classical", &ompl::geometric::MPMDN::time_classical)
        .def_readwrite("time_simplify", &ompl::geometric::MPMDN::time_simplify)
        .def_readwrite("time_all", &ompl::geometric::MPMDN::time_all)
        .def_readwrite("forward_ori", &ompl::geometric::MPMDN::forward_ori)
        .def_readwrite("forward_nnrep", &ompl::geometric::MPMDN::forward_nnrep)
        .def_readwrite("invalid_o", &ompl::geometric::MPMDN::invalid_o)
        .def_readwrite("invalid_nnrep", &ompl::geometric::MPMDN::invalid_nnrep)
        .def_readwrite("colli_o", &ompl::geometric::MPMDN::colli_o)
        .def_readwrite("colli_nnrep", &ompl::geometric::MPMDN::colli_nnrep);



}
