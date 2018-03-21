/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#include "extended-rtt-component.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>


ExampleRightArm::ExampleRightArm(std::string const & name) : RTT::TaskContext(name), exampleAttribute("exampleAttribute",42) {
    // constructor:
    // bind attribute
    this->attributes()->addAttribute( exampleAttribute );
}

bool ExampleRightArm::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    // check for port connectivity
    return true;
}

bool ExampleRightArm::startHook() {
    // this method starts the component
    return true;
}

void ExampleRightArm::updateHook() {
    // we must handle data flow state

    // actual controller!
    
    // write it to port
    
}

void ExampleRightArm::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void ExampleRightArm::cleanupHook() {
    // cleaning the component data
}

double ExampleRightArm::getSimulationTime() {
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}


// This macro, as you can see, creates the component. Every component should have this!
// talk about the way it differs from the first component
ORO_LIST_COMPONENT_TYPE(ExampleRightArm)
