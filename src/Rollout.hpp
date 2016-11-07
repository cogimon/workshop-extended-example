/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/OperationCaller.hpp>
#include <string>


class Rollout: public RTT::TaskContext  {//: public RolloutIF
public:
    Rollout(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    bool init();
    bool addAdditionalComponent(std::string & componentName);
    void startRobotGazebo();
    void stopRobotGazebo();
    void configureAdditonalComponents();
    void startAdditonalComponents();
    void stopAdditonalComponents();
    void cleanupAdditonalComponents();
    void resetWorldCaller();
    void toggleDynamicsSimulationCaller(bool activate);
    bool testMultipleRolloutExecutions(unsigned int numRollouts, bool toggleDynamics);
    bool sleeping();


private:
    TaskContext* gazebo_task_ptr;
    TaskContext* robotgazebo_task_ptr;
    RTT::OperationCaller<bool(void)> resetWorldGazebo;
    RTT::OperationCaller<bool(const bool)> toggleDynamicsSimulationGazebo;
    std::vector<TaskContext*> additonalComponent_task_ptrs;
    std::vector<std::string> additonalComponent_names;
    bool initialized;
    bool test;
};

