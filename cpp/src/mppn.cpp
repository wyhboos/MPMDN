#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <string>
#include <iostream>
#include "cnpy.h"

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
    // ompl::base::ScopedState<> start_test(space);
    // auto a = start.get();
    // base::CompoundState* b = start_test.get();
    // base::CompoundState* c = start.get();
    // // auto b = *a;
    // std::cout<<typeid(*a).name()<<std::endl;
    // std::cout<<typeid(*b).name()<<std::endl;
    // auto y = (*b).as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
    // auto x = a->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
    // std::cout<<"x"<<x<<std::endl;
    // std::cout<<"y"<<x<<std::endl;

    start = start_state;

    // // create the goal state
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);
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
    std::cout<<"This is the MPPN, Start to plan!"<<std::endl;

    // return the status
    Env_encoding = get_env_encoding(env_index);
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_b = bidirectional_plan(&start, &goal);
    int l = path_b.size();
    // std::cout<<"lasdasdasdasdasd"<<l<<std::endl;
    for (int i = 0; i < l; i++)
    {
        // auto state = path_b[i];
        // // std::cout<<"ADD PATH"<<i<<std::endl;
        // float x = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        // float y = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        // float yaw = state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        path->append(path_b[i]->get());
        // std::cout<<"x"<<x<<std::endl;
        // std::cout<<"y"<<y<<std::endl;
        // std::cout<<"yaw"<<yaw<<std::endl;
    }
    pdef->addSolutionPath(path);
    return base::PlannerStatus::EXACT_SOLUTION;
}


std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> ompl::geometric::MPPN::bidirectional_plan(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* start, ompl::base::ScopedState<ompl::base::CompoundStateSpace>* goal)
{
    int iter_cnt = 0;
    int iter_cnt_lim = 100;
    int turn = 0;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path1;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path2;
    path1.push_back(start);
    path2.push_back(goal);
    bool connect;
    while(true)
    {   
        iter_cnt += 1;
        // from start to goal, path1
        if (turn==0)
        {
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* start_now = path1.back();
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* goal_now = path2.back();
            std::vector<torch::jit::IValue> inputs;
            at::Tensor env_encoding = Env_encoding;
            at::Tensor start_t = get_state_tensor_from_state(start_now);
            at::Tensor goal_t = get_state_tensor_from_state(goal_now);
            inputs.push_back(env_encoding);
            inputs.push_back(start_t);
            inputs.push_back(goal_t);
            at::Tensor output = Pnet.forward(inputs).toTensor();
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* next_state = get_state_ompl_from_tensor(output);
            path1.push_back(next_state);
            turn = 1;
        }
        else
        {
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* start_now = path2.back();
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* goal_now = path1.back();
            std::vector<torch::jit::IValue> inputs;
            at::Tensor env_encoding = torch::ones({1,28});
            at::Tensor start_t = get_state_tensor_from_state(start_now);
            at::Tensor goal_t = get_state_tensor_from_state(goal_now);
            inputs.push_back(env_encoding);
            inputs.push_back(start_t);
            inputs.push_back(goal_t);
            at::Tensor output = Pnet.forward(inputs).toTensor();
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* next_state = get_state_ompl_from_tensor(output);
            path2.push_back(next_state);
            turn = 1;
        }

        //check if path1 and path2 can connect
        connect = si_->checkMotion(path1.back()->get(), path2.back()->get());
        if (connect)
        {
            //connect the two paths
            int l = path2.size();
            for(int i=0; i<l;i++)
            {
                path1.push_back(path2[l-1-i]);
            } 
            std::cout<<"Find solution in bidirectional planning!"<<std::endl;
            return path1;
        }
        else if(iter_cnt>iter_cnt_lim)
        {
            // std::cout<<"Not find solution in bidirectional planning!"<<std::endl;
            int l = path2.size();
            for(int i=0; i<l;i++)
            {
                path1.push_back(path2[l-1-i]);
            } 
            std::cout<<"Not find solution in bidirectional planning!"<<std::endl;
            return path1;
        }
    }
}

std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> ompl::geometric::MPPN::replan(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_ori, bool orcle)
{
    int l = path_ori.size();
    int l_r;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_new;
    for (int i = 0; i < (l-1); i++)
    {
        if(!si_->isValid(path_ori[i]->get())) continue;

        if(si_->checkMotion(path_ori[i]->get(), path_ori[i+1]->get()))
        {
            path_new.push_back(path_ori[i]);
            path_new.push_back(path_ori[i+1]);
        }
        else
        {
            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> rep_path;
            if (orcle)
            {
                rep_path = orcle_plan(path_ori[i], path_ori[i+1]);
            }
            else
            {
                rep_path = bidirectional_plan(path_ori[i], path_ori[i+1]);
            }

            l_r = rep_path.size();
            for (int i = 0; i < l_r; i++)
            {
                path_new.push_back(rep_path[i]);
            }
            
        }
    }
    return path_new;
    
}

std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> ompl::geometric::MPPN::orcle_plan(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* start, ompl::base::ScopedState<ompl::base::CompoundStateSpace>* goal)
{
    base::StateSpacePtr space = si_->getStateSpace();
    replan_ss->setStartAndGoalStates(*start, *goal);
    ompl::base::PlannerStatus solved = replan_ss->solve(1.0);
    if (solved)
    {   
        std::cout << "Orcle solution found!" << std::endl;
        std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_;
        // std::vector<ompl::base::CompoundStateSpace*> states = replan_ss->getSolutionPath().getStates();
        std::vector<ompl::base::State*> states = replan_ss->getSolutionPath().getStates();
        int l = replan_ss->getSolutionPath().getStateCount();
        for (int i = 0; i < l; i++)
        {
            auto sco_state_tmp = (states[i]->as<ompl::base::CompoundState>());
            // auto sco_state = sco_state_tmp->as<ompl::base::ScopedState<ompl::base::CompoundStateSpace>>();
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* sco_s = new ompl::base::ScopedState<ompl::base::CompoundStateSpace>(space);
            *sco_s = sco_state_tmp;
            path_.push_back(sco_s);
        }
        
        return path_;
    }
    else
        std::cout << "No solution found" << std::endl;
}


at::Tensor ompl::geometric::MPPN::get_state_tensor_from_state(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* state)
{
    if (state_type=="Rigidbody_2D")
    {
        at::Tensor state_t = torch::ones({1, 3});
        float x = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        float y = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        float yaw = state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        state_t[0][0] = x;
        state_t[0][1] = y;
        state_t[0][2] = yaw;
        return state_t;
    }
    if (state_type=="Two_Link_2D")
    {
        at::Tensor state_t = torch::ones({1, 4});
        float x = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        float y = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        float yaw1 = state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        float yaw2 = state->get()->as<ompl::base::SO2StateSpace::StateType>(2)->value;
        state_t[0][0] = x;
        state_t[0][1] = y;
        state_t[0][2] = yaw1;
        state_t[0][3] = yaw2;
        return state_t;
    }
}

ompl::base::ScopedState<ompl::base::CompoundStateSpace>* ompl::geometric::MPPN::get_state_ompl_from_tensor(at::Tensor state_t)
 {
    base::StateSpacePtr space = si_->getStateSpace();
    ompl::base::ScopedState<ompl::base::CompoundStateSpace>* state =new ompl::base::ScopedState<ompl::base::CompoundStateSpace>(space);
    float *float_ptr = state_t.data_ptr<float>();
    if (state_type=="Rigidbody_2D")
    {   
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0]=float_ptr[0];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1]=float_ptr[1];
        state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value=float_ptr[2];
    }
    if (state_type=="Two_Link_2D")
    {
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0]= float_ptr[0];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1]= float_ptr[1];
        state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value = float_ptr[2];
        state->get()->as<ompl::base::SO2StateSpace::StateType>(2)->value = float_ptr[3];
    }
    return state;
 }

void ompl::geometric::MPPN::load_Enet_Pnet(std::string Enet_file, std::string Pnet_file)
{
    // Enet = torch::jit::load(Pnet_file);
    try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
        Pnet = torch::jit::load(Pnet_file);
        Enet = torch::jit::load(Enet_file);
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
    }
    std::cout<<"Load Model Suc!"<<std::endl;
}

void ompl::geometric::MPPN::load_obs_cloud(std::string cloud_file)
{
    cnpy::NpyArray arr = cnpy::npy_load(cloud_file);
    obs_clouds = arr.data<float>();
    std::cout<<"Size0"<<arr.shape[0]<<std::endl;
    std::cout<<"Size1"<<arr.shape[1]<<std::endl;
    std::cout<<"Load obs clouds Suc!"<<std::endl;
}

at::Tensor ompl::geometric::MPPN::get_env_encoding(int index)
{
    float *cloud_start = obs_clouds + index*2800;
    at::Tensor obs_cloud = torch::from_blob(cloud_start, {1,2800});
    // at::Tensor obs_cloud1 = torch::ones({1, 2800});
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs_cloud);
    at::Tensor output = Enet.forward(inputs).toTensor();
    return output;
}

ompl::geometric::SimpleSetup* ompl::geometric::MPPN::setup_orcle_planner()
{
    //We need to reconstruct space information
    if (state_type == "Rigidbody_2D")
    {   
        base::StateSpacePtr space_ = si_->getStateSpace();
        // ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        ompl::base::RealVectorStateSpace* vector_space =new ompl::base::RealVectorStateSpace(2);
        ompl::base::SO2StateSpace* angle_space1 = new ompl::base::SO2StateSpace;
        ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        vector_space->setBounds(bounds);
        ompl::base::StateSpacePtr vector_space_ptr(vector_space);
        ompl::base::StateSpacePtr angle_space1_ptr(angle_space1);
        auto cs(std::make_shared<ompl::base::CompoundStateSpace>());
        cs->addSubspace(vector_space_ptr, 1.0);
        cs->addSubspace(angle_space1_ptr, 0.5);

        ompl::geometric::SimpleSetup* ss=new ompl::geometric::SimpleSetup(space_);
        ss->setStateValidityChecker(si_->getStateValidityChecker());
        ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation()));
        return ss;
    }
    if (state_type == "Two_Link_2D")
    {   
        base::StateSpacePtr space_ = si_->getStateSpace();
        // ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        ompl::base::RealVectorStateSpace* vector_space =new ompl::base::RealVectorStateSpace(2);
        ompl::base::SO2StateSpace* angle_space1 = new ompl::base::SO2StateSpace;
        ompl::base::SO2StateSpace* angle_space2 = new ompl::base::SO2StateSpace;
        ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        vector_space->setBounds(bounds);
        ompl::base::StateSpacePtr vector_space_ptr(vector_space);
        ompl::base::StateSpacePtr angle_space1_ptr(angle_space1);
        ompl::base::StateSpacePtr angle_space2_ptr(angle_space2);

        auto cs(std::make_shared<ompl::base::CompoundStateSpace>());
        cs->addSubspace(vector_space_ptr, 1.0);
        cs->addSubspace(angle_space1_ptr, 0.5);
        cs->addSubspace(angle_space2_ptr, 0.5);

        ompl::geometric::SimpleSetup* ss=new ompl::geometric::SimpleSetup(space_);
        ss->setStateValidityChecker(si_->getStateValidityChecker());
        ss->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss->getSpaceInformation()));
        return ss;
    }


}