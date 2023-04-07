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
            std::string env_file;
            std::string state_type = "Rigidbody_2D";
            torch::jit::script::Module Pnet;
            torch::jit::script::Module Enet;
            at::Tensor Env_encoding;
            MPPN(const base::SpaceInformationPtr &si) : base::Planner(si, "MPPN")
            {
                // the specifications of this planner (ompl::base::PlannerSpecs)
                // specs_.approximateSolutions = true;
                // specs_.recognizedGoal = ...;
                // load_Enet_Pnet("test", "test");
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

            // std::vector<ompl::base::ScopedState<>*> bidirectional_plan(ompl::base::ScopedState<>* start, ompl::base::ScopedState<>* goal);
            // std::vector<ompl::base::ScopedState<>*> replan_with_nn(std::vector<ompl::base::ScopedState<>*> path_ori);
            // std::vector<ompl::base::ScopedState<>*> replan_with_orcle(std::vector<ompl::base::ScopedState<>*> path_ori);
            // at::Tensor get_state_tensor_from_state(ompl::base::ScopedState<>* state);
            // ompl::base::ScopedState<> *get_state_ompl_from_tensor(at::Tensor state_t);
            // void load_Enet_Pnet(std::string Enet_file, std::string Pnet_file);
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