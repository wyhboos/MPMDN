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
#include <ompl/geometric/planners/rrt/RRT.h>

#include <ompl/base/PlannerTerminationCondition.h>
#include <string>
#include <iostream>
#include "cnpy.h"
#include <chrono>
// #include <torch/script.h> // One-stop header.
#include <mpn.h>

ompl::base::PlannerStatus ompl::geometric::MPN::solve(const base::PlannerTerminationCondition &ptc)
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
    std::cout<<"----------This is the MPN, Start to plan!----------"<<std::endl;

    //init statistics
    time_o = 0;
    time_nnrp = 0;
    time_classical = 0;
    time_simplify = 0;
    time_all = 0;
    forward_ori = 0;
    forward_nnrep = 0;
    invalid_o = 0;
    invalid_nnrep = 0;
    colli_o = 0;
    colli_nnrep = 0;
    failed = false;
    rep_flg = false;
    Env_encoding = get_env_encoding(env_index);

    auto start_o = std::chrono::high_resolution_clock::now();
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_b = bidirectional_plan(&start, &goal);
    if (failed) return base::PlannerStatus::TIMEOUT;
    auto end_o = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_o = end_o - start_o;
    time_o = elapsed_o.count();

    if(ori_simplify) path_b = simplify_path(path_b);
    if (is_feasible(path_b))
    {
        std::cout<<"Origin plan is feasible!"<<std::endl;
        int l1 = path_b.size();
        for (int i = 0; i < l1; i++)
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
    }
    else
    {
        rep_flg = true;
        std::cout<<"Origin plan is not feasible! Turn to replan"<<std::endl;
        // int nn_rep_cnt_lim = 2;
        int nn_rep_cnt_cnt = 0;
        bool orcle = false;
        while (!is_feasible(path_b))
        {
            if(nn_rep_cnt_cnt>=nn_rep_cnt_lim)
            {
                if (use_orcle)
                {
                    orcle = true;
                }
                else
                {
                    failed = true;
                    break;
                }
            }
            auto start_rp = std::chrono::high_resolution_clock::now();
            std::cout<<"Replan cnt:"<<nn_rep_cnt_cnt<<std::endl;
            std::cout<<"Use orcle:"<<orcle<<std::endl;
            path_b = replan(path_b, orcle);
            if (failed) break;
            auto end_rp = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_rp = end_rp - start_rp;
            if (orcle) 
            {
                time_classical += elapsed_rp.count();
            }
            else 
            {
                time_nnrp+= elapsed_rp.count();
            }
            auto start_sp = std::chrono::high_resolution_clock::now();
            path_b = simplify_path(path_b);
            auto end_sp = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_sp = end_sp - start_sp;
            time_simplify += elapsed_sp.count();
            nn_rep_cnt_cnt += 1;
        }
        int l2 = path_b.size();
        for (int i = 0; i < l2; i++)
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
        
    }

    pdef->addSolutionPath(path);
    std::cout<<"---------Statitics----------"<<std::endl;
    std::cout<<"time_o:"<<time_o<<std::endl;
    std::cout<<"time_nnrp:"<<time_nnrp<<std::endl;
    std::cout<<"time_classical:"<<time_classical<<std::endl;
    std::cout<<"forward_ori:"<<forward_ori<<std::endl;
    std::cout<<"forward_nnrep:"<<forward_nnrep<<std::endl;
    std::cout<<"invalid_o:"<<invalid_o<<std::endl;
    std::cout<<"invalid_nnrep:"<<invalid_nnrep<<std::endl;
    std::cout<<"----------End plan!----------"<<std::endl;

    auto end_all = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_all = end_all - start_o;
    time_all = elapsed_all.count();

    // int l3 = path_b.size();
    // for (int i = 0; i < l3; i++)
    // {
    //     path->append(path_b[i]->get());
    // }  

    if (failed) return base::PlannerStatus::APPROXIMATE_SOLUTION;
    return base::PlannerStatus::EXACT_SOLUTION;
}

std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> ompl::geometric::MPN::bidirectional_plan(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* start, ompl::base::ScopedState<ompl::base::CompoundStateSpace>* goal)
{
    int iter_cnt = 0;
    // int iter_cnt_lim = 10;
    int turn = 0;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path1;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path2;
    path1.push_back(start);
    path2.push_back(goal);
    bool connect;
    bool isvalid=1;
    bool is_colli=1;
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
            at::Tensor start_t = get_state_tensor_from_state(start_now).to(at::kCUDA);
            at::Tensor goal_t = get_state_tensor_from_state(goal_now).to(at::kCUDA);
            inputs.push_back(env_encoding);
            inputs.push_back(start_t);
            inputs.push_back(goal_t);
            // std::cout<<"env"<<env_encoding<<std::endl;
            // std::cout<<"start_t"<<start_t<<std::endl;
            // std::cout<<"goal_t"<<goal_t<<std::endl;

            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* next_state;
            at::Tensor output = Pnet.forward(inputs).toTensor();
            next_state = get_state_ompl_from_tensor(output.to(at::kCPU));
            // valid check 
            for (int i = 0; i < valid_ck_cnt; i++)
            {
                if(si_->isValid(next_state->get())) break;
                at::Tensor output = Pnet.forward(inputs).toTensor();
                next_state = get_state_ompl_from_tensor(output.to(at::kCPU));
            }
            // colli check
            for (int i = 0; i < colli_ck_cnt; i++)
            {
                if(si_->checkMotion(start_now->get(), next_state->get())) break;
                at::Tensor output = Pnet.forward(inputs).toTensor();
                next_state = get_state_ompl_from_tensor(output.to(at::kCPU));
            }
            isvalid = si_->isValid(next_state->get());
            is_colli = !si_->checkMotion(start_now->get(), next_state->get());
            path1.push_back(next_state);
            turn = 1;
            if(rep_flg)
            {
                forward_nnrep += 1;
                if (!isvalid) invalid_nnrep += 1;
                if (is_colli) colli_nnrep += 1;
            }
            else
            {
                forward_ori += 1;
                if (!isvalid) invalid_o += 1;
                if (is_colli) colli_o += 1;
            }
        }
        else
        {
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* start_now = path2.back();
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* goal_now = path1.back();
            std::vector<torch::jit::IValue> inputs;
            at::Tensor env_encoding = Env_encoding;
            at::Tensor start_t = get_state_tensor_from_state(start_now).to(at::kCUDA);
            at::Tensor goal_t = get_state_tensor_from_state(goal_now).to(at::kCUDA);
            inputs.push_back(env_encoding);
            inputs.push_back(start_t);
            inputs.push_back(goal_t);
            ompl::base::ScopedState<ompl::base::CompoundStateSpace>* next_state;
            at::Tensor output = Pnet.forward(inputs).toTensor();
            next_state = get_state_ompl_from_tensor(output.to(at::kCPU));
            // valid check 
            for (int i = 0; i < valid_ck_cnt; i++)
            {
                if(si_->isValid(next_state->get())) break;
                at::Tensor output = Pnet.forward(inputs).toTensor();
                next_state = get_state_ompl_from_tensor(output.to(at::kCPU));
            }
            // colli check
            for (int i = 0; i < colli_ck_cnt; i++)
            {
                if(si_->checkMotion(start_now->get(), next_state->get())) break;
                at::Tensor output = Pnet.forward(inputs).toTensor();
                next_state = get_state_ompl_from_tensor(output.to(at::kCPU));
            }
            isvalid = si_->isValid(next_state->get());
            is_colli = !si_->checkMotion(start_now->get(), next_state->get());
            path2.push_back(next_state);
            turn = 0;
            if(rep_flg)
            {
                forward_nnrep += 1;
                if (!isvalid) invalid_nnrep += 1;
                if (is_colli) colli_nnrep += 1;
            }
            else
            {
                forward_ori += 1;
                if (!isvalid) invalid_o += 1;
                if (is_colli) colli_o += 1;
            }
        }

        //check if path1 and path2 can connect
        connect = si_->checkMotion(path1.back()->get(), path2.back()->get());
        if (connect)
        {
            //connect the two paths
            int l = path2.size();
            for(int i=0; i<l;i++)
            {
                path1.push_back(path2[l-i-1]);
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
                path1.push_back(path2[l-i-1]);
            } 
            std::cout<<"Not find solution in bidirectional planning!"<<std::endl;
            failed = false;
            return path1;
        }
    }
}

std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> ompl::geometric::MPN::replan(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_ori, bool orcle)
{
    int l = path_ori.size();
    int l_r;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_valid;
    for (int i = 0; i <l; i++)
    {   
        if(!si_->isValid(path_ori[i]->get())) continue;
        else
        {
            path_valid.push_back(path_ori[i]);
        }
    }
    int l_v = path_valid.size();
    std::cout<<"valid length"<<l_v<<std::endl;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_new;
    for (int i = 0; i < (l_v-1); i++)
    {   
        if(!si_->isValid(path_valid[i]->get())) continue;

        if(!si_->isValid(path_valid[i+1]->get())) std::cout<<"FUCK"<<std::endl;

        if(si_->checkMotion(path_valid[i]->get(), path_valid[i+1]->get()))
        {
            path_new.push_back(path_valid[i]);
            path_new.push_back(path_valid[i+1]);
        }
        else
        {
            std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> rep_path;
            if (orcle)
            {
                rep_path = orcle_plan(path_valid[i], path_valid[i+1]);
            }
            else
            {
                rep_path = bidirectional_plan(path_valid[i], path_valid[i+1]);
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

std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> ompl::geometric::MPN::orcle_plan(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* start, ompl::base::ScopedState<ompl::base::CompoundStateSpace>* goal)
{
    base::StateSpacePtr space = si_->getStateSpace();
    replan_ss->clear(); //clear the planner!
    replan_ss->setStartAndGoalStates(*start, *goal);
    // ompl::base::PlannerStatus solved = replan_ss->solve(ompl::base::exactSolnPlannerTerminationCondition(replan_ss->getProblemDefinition()));
    ompl::base::PlannerStatus solved = replan_ss->solve(
        ompl::base::plannerOrTerminationCondition(
            ompl::base::exactSolnPlannerTerminationCondition(replan_ss->getProblemDefinition()), 
            ompl::base::timedPlannerTerminationCondition(orcle_time_lim)));
    // ompl::base::plannerOrTerminationCondition(ompl::base::exactSolnPlannerTerminationCondition(replan_ss->getProblemDefinition()), ompl::base::timedPlannerTerminationCondition(10))
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
    if (solved && solved.asString() == "Exact solution")
    {
        std::cout << "Orcle solution found!" << std::endl;
    }
    else
    {
        failed = true;
        std::cout << "No solution found" << std::endl;
    }
    return path_;
}

std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> ompl::geometric::MPN::simplify_path(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace> *> path_ori)
{
    int l = path_ori.size();
    base::StateSpacePtr space_ = si_->getStateSpace();
    std::cout<<"before simplify:"<<l<<std::endl;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_valid;
    ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
    for (int i = 0; i < l; i++)
    {
        // double x = path_ori[i]->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        // double y = path_ori[i]->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        if (si_->isValid(path_ori[i]->get()) && is_in_bounds(path_ori[i], bounds)) 
        {
            path_valid.push_back(path_ori[i]);
        }
    }
    std::cout<<"valid simplify:"<<path_valid.size()<<std::endl;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> simpify_path;
    simpify_path.push_back(path_valid[0]);
    l = path_valid.size();
    int left = 0;
    int right = l - 1;
    int p_index = 0;
    while (true)
    {
        //termination condition
        if (left >= l - 1) break;

        //the most right point is left's right point
        if(left >= right - 1)
        {
            simpify_path.push_back(path_valid[right]);
            left = right;
            right = l - 1;
            continue;
        }
        //if collision happens, move the right point one step to the left
        if(!si_->checkMotion(path_valid[left]->get(), path_valid[right]->get()))
        {
            right -= 1;
        }
        else
        {
            simpify_path.push_back(path_valid[right]);
            left = right;
            right = l - 1;
        }
    }
    int ll = simpify_path.size();
    std::cout<<"after simplify:"<<ll<<std::endl;
    return simpify_path;
}

at::Tensor ompl::geometric::MPN::get_state_tensor_from_state(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* state)
{
    if (state_type=="Point_2D")
    {
        at::Tensor state_t = torch::ones({1, 2});
        float x = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        float y = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        state_t[0][0] = x;
        state_t[0][1] = y;
        return state_t;
    }
    if (state_type=="Point_3D")
    {
        at::Tensor state_t = torch::ones({1, 3});
        float x = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        float y = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        float z = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2];
        state_t[0][0] = x;
        state_t[0][1] = y;
        state_t[0][2] = z;
        return state_t;
    }
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
    if (state_type=="Three_Link_2D")
    {
        at::Tensor state_t = torch::ones({1, 5});
        float x = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        float y = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        float yaw1 = state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        float yaw2 = state->get()->as<ompl::base::SO2StateSpace::StateType>(2)->value;
        float yaw3 = state->get()->as<ompl::base::SO2StateSpace::StateType>(3)->value;
        state_t[0][0] = x;
        state_t[0][1] = y;
        state_t[0][2] = yaw1;
        state_t[0][3] = yaw2;
        state_t[0][4] = yaw3;
        return state_t;
    }
    if (state_type=="panda_arm")
    {
        at::Tensor state_t = torch::ones({1, 7});
        float j1 = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];
        float j2 = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1];
        float j3 = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2];
        float j4 = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[3];
        float j5 = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[4];
        float j6 = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[5];
        float j7 = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[6];
        state_t[0][0] = j1;
        state_t[0][1] = j2;
        state_t[0][2] = j3;
        state_t[0][3] = j4;
        state_t[0][4] = j5;
        state_t[0][5] = j6;
        state_t[0][6] = j7;
        return state_t;
    }
}

ompl::base::ScopedState<ompl::base::CompoundStateSpace>* ompl::geometric::MPN::get_state_ompl_from_tensor(at::Tensor state_t)
 {
    base::StateSpacePtr space = si_->getStateSpace();
    ompl::base::ScopedState<ompl::base::CompoundStateSpace>* state =new ompl::base::ScopedState<ompl::base::CompoundStateSpace>(space);
    float *float_ptr = state_t.data_ptr<float>();
    if (state_type=="Point_2D")
    {   
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0]=float_ptr[0];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1]=float_ptr[1];
    }
    if (state_type=="Point_3D")
    {   
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0]=float_ptr[0];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1]=float_ptr[1];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2]=float_ptr[2];
    }
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
    if (state_type=="Three_Link_2D")
    {
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0]= float_ptr[0];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1]= float_ptr[1];
        state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value = float_ptr[2];
        state->get()->as<ompl::base::SO2StateSpace::StateType>(2)->value = float_ptr[3];
        state->get()->as<ompl::base::SO2StateSpace::StateType>(3)->value = float_ptr[4];
    }
    if (state_type=="panda_arm")
    {
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0]=float_ptr[0];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1]=float_ptr[1];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2]=float_ptr[2];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[3]=float_ptr[3];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[4]=float_ptr[4];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[5]=float_ptr[5];
        state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[6]=float_ptr[6];
    }
    return state;
 }

void ompl::geometric::MPN::load_Enet_Pnet(std::string Enet_file, std::string Pnet_file)
{
    // Enet = torch::jit::load(Pnet_file);
    try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
        Pnet = torch::jit::load(Pnet_file);
        if (Pnet_train)
        {
            Pnet.train();
        }
        else
        {
            Pnet.eval();
        }
        Pnet.to(at::kCUDA);
        Enet = torch::jit::load(Enet_file);
        Enet.eval();
        Enet.to(at::kCUDA);
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
    }
    std::cout<<"Load Model Suc!"<<std::endl;
}

void ompl::geometric::MPN::load_obs_cloud(std::string cloud_file)
{
    obs_clouds = cnpy::npy_load(cloud_file);
    std::cout<<"Size0"<<obs_clouds.shape[0]<<std::endl;
    std::cout<<"Size1"<<obs_clouds.shape[1]<<std::endl;
    std::cout<<"Load obs clouds Suc!"<<std::endl;
}

at::Tensor ompl::geometric::MPN::get_env_encoding(int index)
{
    float *cloud_start = obs_clouds.data<float>();
    cloud_start += index*2800;
    at::Tensor obs_cloud = torch::from_blob(cloud_start, {1,2800}).to(at::kCUDA);
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs_cloud);
    at::Tensor output = Enet.forward(inputs).toTensor().to(at::kCUDA);
    std::cout<<"env index:"<<index<<std::endl;
    // std::cout<<output<<index<<std::endl;
    return output;
}

ompl::geometric::SimpleSetup* ompl::geometric::MPN::setup_orcle_planner()
{
    //We need to reconstruct space information
    if (state_type == "Point_2D")
    {   
        base::StateSpacePtr space_ = si_->getStateSpace();
        // ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        ompl::base::RealVectorStateSpace* vector_space =new ompl::base::RealVectorStateSpace(2);
        ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        vector_space->setBounds(bounds);
        ompl::base::StateSpacePtr vector_space_ptr(vector_space);
        auto cs(std::make_shared<ompl::base::CompoundStateSpace>());
        cs->addSubspace(vector_space_ptr, 1.0);

        ompl::geometric::SimpleSetup* ss=new ompl::geometric::SimpleSetup(space_);
        ss->setStateValidityChecker(si_->getStateValidityChecker());
        ss->setPlanner(std::make_shared<ompl::geometric::RRT>(ss->getSpaceInformation()));//RRTstar seems to fail to terminate when finding initial solution, so chose RRT
        return ss;
    }
    if (state_type == "Point_3D")
    {   
        base::StateSpacePtr space_ = si_->getStateSpace();
        // ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        ompl::base::RealVectorStateSpace* vector_space =new ompl::base::RealVectorStateSpace(3);
        ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        vector_space->setBounds(bounds);
        ompl::base::StateSpacePtr vector_space_ptr(vector_space);
        auto cs(std::make_shared<ompl::base::CompoundStateSpace>());
        cs->addSubspace(vector_space_ptr, 1.0);

        ompl::geometric::SimpleSetup* ss=new ompl::geometric::SimpleSetup(space_);
        ss->setStateValidityChecker(si_->getStateValidityChecker());
        ss->setPlanner(std::make_shared<ompl::geometric::RRT>(ss->getSpaceInformation()));//RRTstar seems to fail to terminate when finding initial solution, so chose RRT
        return ss;
    }
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
        ss->setPlanner(std::make_shared<ompl::geometric::RRT>(ss->getSpaceInformation()));//RRTstar seems to fail to terminate when finding initial solution, so chose RRT
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
        ss->setPlanner(std::make_shared<ompl::geometric::RRT>(ss->getSpaceInformation()));
        return ss;
    }
    if (state_type == "Three_Link_2D")
    {   
        base::StateSpacePtr space_ = si_->getStateSpace();
        // ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        ompl::base::RealVectorStateSpace* vector_space =new ompl::base::RealVectorStateSpace(2);
        ompl::base::SO2StateSpace* angle_space1 = new ompl::base::SO2StateSpace;
        ompl::base::SO2StateSpace* angle_space2 = new ompl::base::SO2StateSpace;
        ompl::base::SO2StateSpace* angle_space3 = new ompl::base::SO2StateSpace;
        ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        vector_space->setBounds(bounds);
        ompl::base::StateSpacePtr vector_space_ptr(vector_space);
        ompl::base::StateSpacePtr angle_space1_ptr(angle_space1);
        ompl::base::StateSpacePtr angle_space2_ptr(angle_space2);
        ompl::base::StateSpacePtr angle_space3_ptr(angle_space3);

        auto cs(std::make_shared<ompl::base::CompoundStateSpace>());
        cs->addSubspace(vector_space_ptr, 1.0);
        cs->addSubspace(angle_space1_ptr, 0.5);
        cs->addSubspace(angle_space2_ptr, 0.5);
        cs->addSubspace(angle_space3_ptr, 0.5);

        ompl::geometric::SimpleSetup* ss=new ompl::geometric::SimpleSetup(space_);
        ss->setStateValidityChecker(si_->getStateValidityChecker());
        ss->setPlanner(std::make_shared<ompl::geometric::RRT>(ss->getSpaceInformation()));
        return ss;
    }
    if (state_type == "panda_arm")
    {   
        base::StateSpacePtr space_ = si_->getStateSpace();
        // ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        ompl::base::RealVectorStateSpace* vector_space =new ompl::base::RealVectorStateSpace(7);
        ompl::base::RealVectorBounds bounds = space_->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->as<ompl::base::RealVectorStateSpace>()->getBounds();
        vector_space->setBounds(bounds);
        ompl::base::StateSpacePtr vector_space_ptr(vector_space);
        auto cs(std::make_shared<ompl::base::CompoundStateSpace>());
        cs->addSubspace(vector_space_ptr, 1.0);

        ompl::geometric::SimpleSetup* ss=new ompl::geometric::SimpleSetup(space_);
        ss->setStateValidityChecker(si_->getStateValidityChecker());
        ss->setPlanner(std::make_shared<ompl::geometric::RRT>(ss->getSpaceInformation()));//RRTstar seems to fail to terminate when finding initial solution, so chose RRT
        return ss;
    }




}

bool ompl::geometric::MPN::is_in_bounds(ompl::base::ScopedState<ompl::base::CompoundStateSpace>* state, ompl::base::RealVectorBounds bounds)
{
    int dim;
    dim = bounds.high.size();
    double value;
    double high;
    double low;

    for (int i = 0; i < dim; i++)
    {
        value = state->get()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[i];
        high = bounds.high[i];
        low = bounds.low[i];
        if(value>high || value<low) return false;
    }
    if (state_type == "Point_2D")
    {
        return true;
    }
    if (state_type == "Point_3D")
    {
        return true;
    }
    if (state_type == "panda_arm")
    {
        return true;
    }
    if (state_type == "Rigidbody_2D")
    {
        double angle = state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        if(angle>3.14 || angle<-3.14) return false;
        return true;
    }
    if (state_type == "Two_Link_2D")
    {
        double angle1 = state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        double angle2 = state->get()->as<ompl::base::SO2StateSpace::StateType>(2)->value;
        if(angle1>3.14 || angle1<-3.14) return false;
        if(angle2>3.14 || angle2<-3.14) return false;
        return true;
    }
    if (state_type == "Three_Link_2D")
    {
        double angle1 = state->get()->as<ompl::base::SO2StateSpace::StateType>(1)->value;
        double angle2 = state->get()->as<ompl::base::SO2StateSpace::StateType>(2)->value;
        double angle3 = state->get()->as<ompl::base::SO2StateSpace::StateType>(3)->value;
        if(angle1>3.14 || angle1<-3.14) return false;
        if(angle2>3.14 || angle2<-3.14) return false;
        if(angle3>3.14 || angle3<-3.14) return false;
        return true;
    }
}

bool ompl::geometric::MPN::is_feasible(std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_ori)
{
    int l = path_ori.size();
    // std::cout<<"bool"<<std::endl;
    std::vector<ompl::base::ScopedState<ompl::base::CompoundStateSpace>*> path_new;
    for (int i = 0; i < (l-1); i++)
    {
        if(!si_->isValid(path_ori[i]->get())) return false;

        if(!si_->checkMotion(path_ori[i]->get(), path_ori[i+1]->get())) return false;
    }
    return true;

}