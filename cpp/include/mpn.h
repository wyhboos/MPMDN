#include <ompl/base/Planner.h>
 
// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#ifndef OMPL_GEOMETRIC_PLANNERS_MPNET_MPNET_
#define OMPL_GEOMETRIC_PLANNERS_MPNET_MPNET_

namespace ompl
{
    namespace geometric
    {
        class MPN : public base::Planner
        {
        public:
            MPN(const base::SpaceInformationPtr &si) : base::Planner(si, "MPN")
            {
                // the specifications of this planner (ompl::base::PlannerSpecs)
                specs_.approximateSolutions = true;
                // specs_.recognizedGoal = ...;
            }
    
            virtual ~myNewPlanner(void)
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