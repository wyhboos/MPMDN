#pragma once
#include <ompl/base/Planner.h>

// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <torch/script.h> // One-stop header.
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>

// #include <ompl/geometric/planners/mpn/mpn.h>
namespace ompl
{
    namespace geometric
    {
        class MPPN : public base::Planner
        {
        public:
            double time_o;
            double time_nnrp;
            double time_classical;
            int nn_rep_cnt_lim = 10;
            int iter_cnt_lim = 100;
            int forward_ori;
            int forward_nnrep;
            int invalid_o;
            int invalid_nnrep;
            int env_index = 0;
            bool failed = false;
            bool rep_flg = false; //help count the rep forward
            std::string env_file = "/home/wyh/Code/MPMDN/Data/S2D/obs_cloud_110.npy";
            std::string state_type = "Rigidbody_2D";
            std::string Enet_file = "/home/wyh/Code/MPMDN/Data/Model_structure/Encoder_S2D.pt";
            std::string Pnet_file = "/home/wyh/Code/MPMDN/Data/Model_structure/MPN_Pnet_S2D_RB.pt";
            torch::jit::script::Module Pnet;
            torch::jit::script::Module Enet;
            at::Tensor Env_encoding;
            float *obs_clouds;
            ompl::geometric::SimpleSetup *replan_ss;
            MPPN(const base::SpaceInformationPtr &si) : base::Planner(si, "MPPN")
            {
                // the specifications of this planner (ompl::base::PlannerSpecs)
                // specs_.approximateSolutions = true;
                // specs_.recognizedGoal = ...;
                load_Enet_Pnet(Enet_file, Pnet_file);
                load_obs_cloud(env_file);
                replan_ss = setup_orcle_planner();
            }

            MPPN(std::string si_info) : base::Planner(si_info, "MPPN")
            {
                // the specifications of this planner (ompl::base::PlannerSpecs)
                specs_.approximateSolutions = true;
                // specs_.recognizedGoal = ...;
            }
            // MPPN(const MPPN&) = default;
            // MPPN &operator=(const MPPN&) = default;

            virtual ~MPPN(void)
            {
                // free any allocated memory
            }

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> bidirectional_plan(ompl::base::ScopedState<ompl::base::CompoundStateSpace> *start, ompl::base::ScopedState<ompl::base::CompoundStateSpace> *goal);
            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> orcle_plan(ompl::base::ScopedState<ompl::base::CompoundStateSpace> *start, ompl::base::ScopedState<ompl::base::CompoundStateSpace> *goal);
            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> replan(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> path_ori, bool orcle);
            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> simplify_path(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> path_ori);
            at::Tensor get_state_tensor_from_state(ompl::base::ScopedState<ompl::base::CompoundStateSpace> *state);
            ompl::base::ScopedState<ompl::base::CompoundStateSpace> *get_state_ompl_from_tensor(at::Tensor state_t);
            void load_Enet_Pnet(std::string Enet_file, std::string Pnet_file);
            void load_obs_cloud(std::string cloud_file);
            at::Tensor get_env_encoding(int index);
            ompl::geometric::SimpleSetup *setup_orcle_planner();
            bool is_feasible(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> path);
            bool is_in_bounds(ompl::base::ScopedState<ompl::base::CompoundStateSpace>, ompl::base::RealVectorBounds);
            void test()
            {
                std::cout << "This is a test which test the py-binding for new function!" << std::endl;
            }
            // void get_env_encoding();

            virtual void clear(void)
            {
                Planner::clear();
                // clear the data structures here
            }

            // optional, if additional setup/configuration is needed, the setup() method can be implemented
            virtual void setup(void)
            {
                Planner::setup();
            }
        };

    }
}