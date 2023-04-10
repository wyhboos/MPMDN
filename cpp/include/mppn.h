#pragma once
#include <ompl/base/Planner.h>

// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <torch/script.h> // One-stop header.

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
            int invalid_o;
            int invalid_nnrp;
            int env_index=0;
            std::string env_file = "/home/wyh/Code/MPMDN/Data/S2D/obs_cloud_100.npy";
            std::string state_type = "Rigidbody_2D";
            std::string Enet_file = "/home/wyh/Code/MPMDN/Data/Model_structure/Encoder_S2D.pt";
            std::string Pnet_file = "/home/wyh/Code/MPMDN/Data/Model_structure/MPN_Pnet_S2D_RB.pt";
            torch::jit::script::Module Pnet;
            torch::jit::script::Module Enet;
            at::Tensor Env_encoding;
            float *obs_clouds;
            std::
            MPPN(const base::SpaceInformationPtr &si) : base::Planner(si, "MPPN")
            {
                // the specifications of this planner (ompl::base::PlannerSpecs)
                // specs_.approximateSolutions = true;
                // specs_.recognizedGoal = ...;
                load_Enet_Pnet(Enet_file, Pnet_file);
                load_obs_cloud(env_file);
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

            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> bidirectional_plan(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* start, ompl::base::ScopedState<ompl::base::CompoundStateSpace>* goal);
            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> replan_with_nn(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_ori);
            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> replan_with_orcle(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_ori);
            at::Tensor get_state_tensor_from_state(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* state);
            ompl::base::ScopedState<ompl::base::CompoundStateSpace> *get_state_ompl_from_tensor(at::Tensor state_t);
            void load_Enet_Pnet(std::string Enet_file, std::string Pnet_file);
            void load_obs_cloud(std::string cloud_file);
            at::Tensor get_env_encoding(int index);
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