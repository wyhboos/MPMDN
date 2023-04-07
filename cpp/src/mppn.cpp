#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <string>
#include <iostream>


// #include <torch/script.h> // One-stop header.
#include <mppn.h>

ompl::base::PlannerStatus ompl::geometric::MPPN::solve(const base::PlannerTerminationCondition &ptc)
{

    // get the problem definition
    base::ProblemDefinitionPtr pdef = getProblemDefinition();
    base::StateSpacePtr space = si_->getStateSpace();

    // // get the start and goal states
    const base::State *start_state = pdef->getStartState(0);
    const base::State *goal_state = pdef->getGoal()->as<ompl::base::GoalState>()->getState();

    // // get the state dimension
    // unsigned int dim = space->getDimension();

    // // create a new path
    auto path(std::make_shared<PathGeometric>(si_));

    // // create the start state
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
    ompl::base::ScopedState<> start_test(space);
    auto a = start.get();
    base::CompoundState* b = start_test.get();
    base::CompoundState* c = start.get();
    // auto b = *a;
    std::cout<<typeid(*a).name()<<std::endl;
    std::cout<<typeid(*b).name()<<std::endl;
    auto y = (*b).as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
    auto x = a->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
    std::cout<<"x"<<x<<std::endl;
    std::cout<<"y"<<x<<std::endl;

    start = start_state;

    // // create the goal state
    ompl::base::ScopedState<> goal(space);
    goal = goal_state;

    // if(!si_->isValid(start.get())) std::cout<<"start invalid"<<std::endl;


    // // // check if the connection between start and goal states is valid
    // bool connection_valid = si_->checkMotion(start_state, goal_state);

    // if(connection_valid)
    // {
    //     std::cout << "The connection between start and goal states is valid." << std::endl;
    // }
    // else
    // {
    //     std::cout << "The connection between start and goal states is invalid." << std::endl;
    // }

    // // add the start state to the path
    // path->append(start.get());

    // // add the goal state to the path
    // path->append(goal.get());
    
    // // add the path to the problem definition
    // pdef->addSolutionPath(path);

    // // this->time_o = 50;

    // std::cout<<"before"<<this->time_o<<std::endl;
    // this->time_o = 50;
    // std::cout<<"after"<<this->time_o<<std::endl;
    std::cout<<"This is the MPPN, Finished!"<<std::endl;

    // return the status

    // std::vector<ompl::base::ScopedState<>*> path_ = bidirectional_plan(&start, &goal);
    // int l = path_.size();
    // for (size_t i = 0; i < l; i++)
    // {
    //     path->append(path_[i]->get());
    // }
    // pdef->addSolutionPath(path);
    return base::PlannerStatus::EXACT_SOLUTION;
}


// std::vector<ompl::base::ScopedState<>*> ompl::geometric::MPPN::bidirectional_plan(ompl::base::ScopedState<>* start, ompl::base::ScopedState<>* goal)
// {
//     int iter_cnt = 0;
//     int iter_cnt_lim = 500;
//     int turn = 0;
//     std::vector<ompl::base::ScopedState<>*> path1;
//     std::vector<ompl::base::ScopedState<>*> path2;
//     path1.push_back(start);
//     path2.push_back(goal);
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
//             at::Tensor goal_t = get_state_tensor_from_state(goal_now);
//             inputs.push_back(env_encoding);
//             inputs.push_back(start_t);
//             inputs.push_back(goal_t);
//             at::Tensor output = Pnet.forward(inputs).toTensor();
//             ompl::base::ScopedState<>* next_state = get_state_ompl_from_tensor(output);
//             path1.push_back(next_state);
//             turn = 1;
//         }
//         else
//         {
//             ompl::base::ScopedState<>* start_now = path2.back();
//             ompl::base::ScopedState<>* goal_now = path1.back();
//             std::vector<torch::jit::IValue> inputs;
//             at::Tensor env_encoding = torch::ones({1,28});
//             at::Tensor start_t = get_state_tensor_from_state(start_now);
//             at::Tensor goal_t = get_state_tensor_from_state(goal_now);
//             inputs.push_back(env_encoding);
//             inputs.push_back(start_t);
//             inputs.push_back(goal_t);
//             at::Tensor output = Pnet.forward(inputs).toTensor();
//             ompl::base::ScopedState<>* next_state = get_state_ompl_from_tensor(output);
//             path2.push_back(next_state);
//             turn = 1;
//         }

//         //check if path1 and path2 can connect
//         connect = si_->checkMotion(path1.back()->get(), path2.back()->get());
//         if (connect)
//         {
//             //connect the two paths
//             int l = path1.size();
//             for(int i=0; i<l;i++)
//             {
//                 path1.push_back(path2[l-1-i]);
//                 std::cout<<"Find solution in bidirectional planning!"<<std::endl;
//                 return path1;
//             } 
//         }
//         else
//         {
//             // std::cout<<"Not find solution in bidirectional planning!"<<std::endl;
//             int l = path1.size();
//             for(int i=0; i<l;i++)
//             {
//                 path1.push_back(path2[l-1-i]);
//                 std::cout<<"Find solution in bidirectional planning!"<<std::endl;
//                 return path1;
//             } 
//             return path1;
//         }
//     }
// }

// std::vector<ompl::base::ScopedState<>*> ompl::geometric::MPPN::replan_with_nn(std::vector<ompl::base::ScopedState<>*> path_ori)
// {
//     int l = path_ori.size();
//     int l_r;
//     std::vector<ompl::base::ScopedState<>*> path_new;
//     for (int i = 0; i < (l-1); i++)
//     {
//         if(!si_->isValid(path_ori[i]->get())) continue;

//         if(si_->checkMotion(path_ori[i]->get(), path_ori[i+1]->get()))
//         {
//             path_new.push_back(path_ori[i]);
//             path_new.push_back(path_ori[i+1]);
//         }
//         else
//         {
//             auto rep_path = bidirectional_plan(path_ori[i], path_ori[i+1]);
//             l_r = rep_path.size();
//             for (int i = 0; i < l_r; i++)
//             {
//                 path_new.push_back(rep_path[i]);
//             }
            
//         }
//     }
//     return path_new;
    
// }

// at::Tensor ompl::geometric::MPPN::get_state_tensor_from_state(ompl::base::ScopedState<>* state)
// {
//     if (state_type=="Rigidbody_2D")
//     {
//         at::Tensor state_t = torch::ones({1, 3});
//         const auto *se3state = state->get()->as<ompl::base::SE2StateSpace::StateType>();
//         // const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
//         // const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
//         float x = se3state->getX();
//         float y = se3state->getY();
//         float yaw = se3state->getYaw();
//         state_t[0] = x;
//         state_t[1] = y;
//         state_t[2] = yaw;
//     }
//     if (state_type=="Two_Link_2D")
//     {
//         at::Tensor state_t = torch::ones({1, 4});
//         float x = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
//         float y = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
//         float yaw1 = state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value;
//         float yaw2 = state->get()->as<ompl::base::SO2StateSpace::StateType>(2)->value;
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
//     ompl::base::ScopedState<>* = new state(space);
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
//     return state;
//  }

// void ompl::geometric::MPPN::load_Enet_Pnet(std::string Enet_file, std::string Pnet_file)
// {
//     Pnet = torch::jit::load(Enet_file);
//     Enet = torch::jit::load(Pnet_file);
// }