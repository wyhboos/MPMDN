#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>

// #include <torch/script.h> // One-stop header.
#include <mppn.h>

ompl::base::PlannerStatus ompl::geometric::MPPN::solve(const base::PlannerTerminationCondition &ptc)
{

    // get the problem definition
    base::ProblemDefinitionPtr pdef = getProblemDefinition();
    base::StateSpacePtr space = si_->getStateSpace();

    // get the start and goal states
    const base::State *start_state = pdef->getStartState(0);
    const base::State *goal_state = pdef->getGoal()->as<ompl::base::GoalState>()->getState();

    // get the state dimension
    unsigned int dim = space->getDimension();

    // create a new path
    auto path(std::make_shared<PathGeometric>(si_));

    // create the start state
    ompl::base::ScopedState<> start(space);
    start = start_state;

    // create the goal state
    ompl::base::ScopedState<> goal(space);
    goal = goal_state;

    if(!si_->isValid(start.get())) std::cout<<"start invalid"<<std::endl;


    // // check if the connection between start and goal states is valid
    bool connection_valid = si_->checkMotion(start_state, goal_state);

    if(connection_valid)
    {
        std::cout << "The connection between start and goal states is valid." << std::endl;
    }
    else
    {
        std::cout << "The connection between start and goal states is invalid." << std::endl;
    }

    // add the start state to the path
    path->append(start.get());

    // add the goal state to the path
    path->append(goal.get());
    
    // add the path to the problem definition
    pdef->addSolutionPath(path);

    // this->time_o = 50;

    std::cout<<"before"<<this->time_o<<std::endl;
    this->time_o = 50;
    std::cout<<"after"<<this->time_o<<std::endl;
    std::cout<<"This is the MPPN, Finished!"<<std::endl;

    // return the status
    return base::PlannerStatus::EXACT_SOLUTION;
}


// std::vector<ompl::base::ScopedState<>>* ompl::geometric::MPPN::bidirectional_plan(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal)
// {
//     int iter_cnt = 0;
//     int iter_cnt_lim = 500;
//     int turn = 0;
//     std::vector<ompl::base::ScopedState<>>* path1;
//     std::vector<ompl::base::ScopedState<>>* path2;
//     path1.pushback(start);
//     path2.pushback(goal);
//     bool connect;
//     while(iter_cnt<iter_cnt_lim)
//     {   
//         iter_cnt += 1;
//         // from start to goal, path1
//         if (turn==0)
//         {
//             ompl::base::ScopedState<>* start_now = path1.back();
//             ompl::base::ScopedState<>* goal_now = path2.back();
//             std::vector<torch::jit::IValue> inputs;
//             at::Tensor env_encoding = torch::ones({1,28});
//             at::Tensor start_t = get_state_tensor_from_state(start_now);
//             at::Tensor goal_t = get_state_tensor_from_state(goal_nows);
//             input.pushback(env_encoding);
//             input.pushback(start_t);
//             input.pushback(goal_t);
//             at::Tensor output = P_net.forward(input).toTensor();
//             ompl::base::ScopedState<>* next_state = get_state_ompl_from_tensor(at::Tensor output);
//             path1.pushback(next_state);
//             turn = 1;
//         }
//         elif (turn==1)
//         {
//             ompl::base::ScopedState<>* start_now = path2.back();
//             ompl::base::ScopedState<>* goal_now = path1.back();
//             std::vector<torch::jit::IValue> inputs;
//             at::Tensor env_encoding = torch::ones({1,28});
//             at::Tensor start_t = get_state_tensor_from_state(start_now);
//             at::Tensor goal_t = get_state_tensor_from_state(goal_nows);
//             input.pushback(env_encoding);
//             input.pushback(start_t);
//             input.pushback(goal_t);
//             at::Tensor output = P_net.forward(input).toTensor();
//             ompl::base::ScopedState<>* next_state = get_state_ompl_from_tensor(at::Tensor output);
//             path2.pushback(next_state);
//             turn = 1;
//         }

//         //check if path1 and path2 can connect
//         connect = si_->checkMotion(path1.back(), path2.back());
//         if (connect)
//         {
//             //connect the two paths
//             int l = path1.size();
//             for(int i=0; i<l;i++)
//             {
//                 path1.pushback(path2[l-1-i]);
//                 std::cout<<"Find solution in bidirectional planning!"<<std::endl;
//                 return path1;
//             } 
//         }
//         else
//         [
//             std::cout<<"Not find solution in bidirectional planning!"<<std::endl;
//             return path1;
//         ]
//     }
// }


// at::Tensor ompl::geometric::MPPN::get_state_tensor_from_state(std::vector<ompl::base::ScopedState<>>* state)
// {
//     if (state_type=="Rigidbody_2D")
//     {
//         state_t = torch::ones({1, 3})
//         float x = state->getX();
//         float y = state->getY();
//         float yaw = state->getYaw();
//         state_t[0] = x;
//         state_t[1] = y;
//         state_t[2] = yaw;
//     }
//     if (state_type=="Two_Link_2D")
//     {
//         state_t = torch::ones({1, 4})
//         float x = state[0]->getX();
//         float y = state[0]->getY();
//         float yaw1 = state[1]->value;
//         float yaw2 = state[2]->value;
//         state_t[0] = x;
//         state_t[1] = y;
//         state_t[2] = yaw1;
//         state_t[3] = yaw2;
//     }
//     return state_t;
// }

// ompl::base::ScopedState<>* ompl::geometric::MPPN::get_state_ompl_from_tensor(at::Tensor state_t)
//  {
//     base::StateSpacePtr space = si_->getStateSpace();
//     ompl::base::ScopedState<> state(space);
//     if (state_type=="Rigidbody_2D")
//     {
//         state->setX(state_t[0]);
//         state->setY(state_t[1]);
//         state->setYaw(state_t[2]);
//     }
//     if (state_type=="Two_Link_2D")
//     {
//         state[0]->setX(state_t[0]);
//         state[0]->setY(state_t[1]);
//         state[2]->value = state_t[2];
//         state[3]->value = state_t[3];
//     }
//     return state.get();
//  }

// void ompl::geometric::MPPN::load_Enet_Pnet(string Enet_file, string Pnet_file)
// {
//     Pnet = torch::jit::load(Enet_file);
//     Enet = torch::jit::load(Pnet_file);
// }