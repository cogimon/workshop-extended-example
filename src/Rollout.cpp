/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#include "Rollout.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


Rollout::Rollout(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("init", &Rollout::init, this).doc("init");
    addOperation("addAdditionalComponent", &Rollout::addAdditionalComponent, this).doc("add additional component");
    addOperation("startRobotGazebo", &Rollout::startRobotGazebo, this, RTT::ClientThread).doc("start robot gazebo");
    addOperation("stopRobotGazebo", &Rollout::stopRobotGazebo, this, RTT::ClientThread).doc("stop robot gazebo");
    addOperation("configureAdditonalCompoents", &Rollout::configureAdditonalCompoents, this, RTT::ClientThread).doc("configure additonal compoents");
    addOperation("startAdditonalCompoents", &Rollout::startAdditonalCompoents, this, RTT::ClientThread).doc("start additonal compoents");
    addOperation("stopAdditonalCompoents", &Rollout::stopAdditonalCompoents, this, RTT::ClientThread).doc("stop additonal compoents");
    addOperation("cleanupAdditonalCompoents", &Rollout::cleanupAdditonalCompoents, this, RTT::ClientThread).doc("cleanup additonal compoents");
    addOperation("resetWorldCaller", &Rollout::resetWorldCaller, this, RTT::ClientThread).doc("call gazebo reset_world()");
    addOperation("toggleDynamicsSimulationCaller", &Rollout::toggleDynamicsSimulationCaller, this, RTT::ClientThread).doc("call gazebo toggleDynamicsSimulation()");
    addOperation("testMultipleRolloutExecutions", &Rollout::testMultipleRolloutExecutions, this, RTT::ClientThread).doc("test multiple rollout executions");

    initialized = false;
}

bool Rollout::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    return false;
}

bool Rollout::startHook() {
    // this method starts the component
    return false;
}

void Rollout::updateHook() {
    // this is the actual body of a component. it is called on each cycle
}

void Rollout::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void Rollout::cleanupHook() {
    // cleaning the component data
}

bool Rollout::init() {
    std::string componentName;
    componentName = "robot_gazebo";
    robotgazebo_task_ptr = getPeer(componentName);
    if (robotgazebo_task_ptr == NULL) {
        std::cout << "null pointer for " << componentName << std::endl;
        return false;
    }

    componentName = "gazebo";
    gazebo_task_ptr = getPeer(componentName);
    if (gazebo_task_ptr == NULL) {
        std::cout << "null pointer for " << componentName << std::endl;
        return false;
    }

    resetWorldGazebo = gazebo_task_ptr->getOperation("reset_world");

    toggleDynamicsSimulationGazebo = gazebo_task_ptr->getOperation("toggleDynamicsSimulation");

    initialized = true;
    return true;
}

bool Rollout::addAdditionalComponent(std::string & componentName) {
    if (initialized){
        TaskContext* tmp_task_ptr = getPeer(componentName);
        if (tmp_task_ptr == NULL) {
            std::cout << "null pointer for " << componentName << std::endl;
            return false;
        }
        additonalComponent_task_ptrs.push_back(tmp_task_ptr);
        additonalComponent_names.push_back(componentName);
        return true;
    }
    else {
        return false;
    }
}

void Rollout::startRobotGazebo() {
//    std::cout << "### configure robotgazebo_task_ptr" << std::endl;
    test = robotgazebo_task_ptr->configure();
//    std::cout << "### result " << test << std::endl;

    //this is a pure virtual method...
//    std::cout << "### start robotgazebo_task_ptr" << std::endl;
//    test = robotgazebo_task_ptr->start();
//    std::cout << "### result " << test << std::endl;
}

void Rollout::stopRobotGazebo() {
//    std::cout << "### stop robotgazebo_task_ptr" << std::endl;
    test = robotgazebo_task_ptr->stop(); //stopps automatically when gazebo is stopped
//    std::cout << "### result " << test << std::endl;

//    std::cout << "### cleanup robotgazebo_task_ptr" << std::endl;
    test = robotgazebo_task_ptr->cleanup();
//    std::cout << "### result " << test << std::endl;
}

void Rollout::configureAdditonalCompoents() {
    for(unsigned int i=0; i<additonalComponent_task_ptrs.size(); i++){
//        std::cout << "### configure " << i << " -> " << additonalComponent_names[i] << std::endl;
        test = additonalComponent_task_ptrs[i]->configure();
//        std::cout << "### result " << test << std::endl;
    }
}

void Rollout::startAdditonalCompoents() {
    for(unsigned int i=0; i<additonalComponent_task_ptrs.size(); i++){
//        std::cout << "### start " << i << " -> " << additonalComponent_names[i] << std::endl;
        test = additonalComponent_task_ptrs[i]->start();
//        std::cout << "### result " << test << std::endl;
    }
}

void Rollout::stopAdditonalCompoents() {
    for(unsigned int i=0; i<additonalComponent_task_ptrs.size(); i++){
//        std::cout << "### stop " << i << " -> " << additonalComponent_names[i] << std::endl;
        additonalComponent_task_ptrs[i]->stop();
//        std::cout << "### result " << test << std::endl;
    }
}

void Rollout::cleanupAdditonalCompoents() {
    for(unsigned int i=0; i<additonalComponent_task_ptrs.size(); i++){
//        std::cout << "### cleanup " << i << " -> " << additonalComponent_names[i] << std::endl;
        additonalComponent_task_ptrs[i]->cleanup();
//        std::cout << "### result " << test << std::endl;
    }
}

void Rollout::resetWorldCaller() {
//    std::cout << "### call resetWorld() " << std::endl;
    test = this->resetWorldGazebo(); //call operation-function
//    std::cout << "### result " << test << std::endl;
    assert(test == true);
}

void Rollout::toggleDynamicsSimulationCaller(bool activate) {
//    std::cout << "### call toggleDynamicsSimulation(" << activate << ")" << std::endl;
    test = this->toggleDynamicsSimulationGazebo(activate); //call operation-function
//    std::cout << "### result " << test << std::endl;
    assert(test == true);
}

bool Rollout::testMultipleRolloutExecutions(unsigned int numRollouts) {
    if (initialized){
        this->sleeping();

        for(unsigned int i=0; i<numRollouts; i++){
            std::cout << "starting... " << i << std::endl;
            this->configureAdditonalCompoents();
            this->startAdditonalCompoents();
            this->toggleDynamicsSimulationCaller(true);
            std::cout << "started! " << i << std::endl;

            this->sleeping();

            std::cout << "stopping... " << i << std::endl;
            this->stopAdditonalCompoents();
            this->toggleDynamicsSimulationCaller(false);
            this->cleanupAdditonalCompoents();
            std::cout << "stopped! " << i << std::endl;

            std::cout << "resetting... " << i << std::endl;
            this->resetWorldCaller();
            std::cout << "resetted! " << i << std::endl;

            this->sleeping();
        }
        return true;
    }
    else {
        return false;
    }
}


bool Rollout::sleeping() {
    double tmp=1;
    for(unsigned int i=1; i<999999999; i++){
        tmp = tmp * double(i);
        if ( i % 100000000 == 0) {
            std::cout << "i = " << i << std::endl;
        }
    }
    return true;
}

//this macro should appear only once per library
ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(Rollout)
