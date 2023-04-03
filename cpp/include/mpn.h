#pragma once
#include <ompl/base/Planner.h>
 
// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>


namespace ompl
{
    namespace geometric
    {
        class MPN : public base::Planner
        {
        public:
            double time_o = 100;
            double time_nnrp;
            double time_classical;
            int invalid_o;
            int invalid_nnrp;
            MPN(const base::SpaceInformationPtr &si) : base::Planner(si, "MPN")
            {
                // the specifications of this planner (ompl::base::PlannerSpecs)
                specs_.approximateSolutions = true;
                // specs_.recognizedGoal = ...;
            }
    
            virtual ~MPN(void)
            {
                // free any allocated memory
            }
    
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

    
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