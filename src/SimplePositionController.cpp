/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#include "SimplePositionController.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


SimplePositionController::SimplePositionController(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsizeAndGains", &SimplePositionController::setDOFsizeAndGains, this).doc("set DOF size and Gains");
    addOperation("setGains", &SimplePositionController::setGains, this).doc("set Gains");
    addOperation("setDesiredJointAngles", &SimplePositionController::setDesiredJointAngles, this).doc("set desired joint angles");
    addOperation("computeJointAngles", &SimplePositionController::computeJointAngles, this).doc("compute joint angles");
    addOperation("printCurrentState", &SimplePositionController::printCurrentState, this).doc("print current state");

    //other stuff
    portsArePrepared = false;
}

bool SimplePositionController::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run

    //check conncetion
    if (!in_robotstatus_port.connected() || !out_angles_port.connected())
        return false;
    else
        return true;
}

bool SimplePositionController::startHook() {
    // this method starts the component
    return true;
}

void SimplePositionController::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_robotstatus_port.connected()) {
        // read data and save state of data into "Flow", which can be "NewData", "OldData" or "NoData".
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        // handle the situation
    }


    // you can handle cases when there is no new data.
    if ((in_robotstatus_flow == RTT::NewData || in_robotstatus_flow == RTT::OldData)) {
        this->computeJointAngles(in_robotstatus_var, desJointAngles, out_angles_var);
    } else if ((in_robotstatus_flow == RTT::NoData)) {
        out_angles_var.angles.setZero();
    } else {
        // there should be something really wrong!
    }

    out_angles_port.write(out_angles_var);
}

void SimplePositionController::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void SimplePositionController::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void SimplePositionController::setDOFsizeAndGains(unsigned int DOFsize, float gainP){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
    this->desJointAngles = rstrt::kinematics::JointAngles(DOFsize);
    this->desJointAngles.angles.setZero();
    this->setGains(gainP);
    this->preparePorts();
}

void SimplePositionController::setGains(float gainP){
    assert(gainP > 0);
    this->gainP = gainP;
}

bool SimplePositionController::setDesiredJointAngles(rstrt::kinematics::JointAngles & desJointAngles){
    if (portsArePrepared){
        if(desJointAngles.angles.size() != DOFsize){
            return false;
        }
        this->desJointAngles.angles = desJointAngles.angles;
        return true;
    }
    else{
        return false;
    }
}


void SimplePositionController::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("out_angles_port");
    }

    //prepare input
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;


    //prepare output
    out_angles_var = rstrt::kinematics::JointAngles(DOFsize);
    out_angles_var.angles.setZero();
    out_angles_port.setName("out_angles_port");
    out_angles_port.doc("Output port for sending angle values");
    out_angles_port.setDataSample(out_angles_var);
    ports()->addPort(out_angles_port);

    portsArePrepared = true;
}

void SimplePositionController::computeJointAngles(rstrt::robot::JointState const & jointState,
                                            rstrt::kinematics::JointAngles const & desJointAngles,
                                            rstrt::kinematics::JointAngles & jointAngles) {
    jointAngles.angles = jointState.angles + gainP * (desJointAngles.angles - jointState.angles);
}

void SimplePositionController::printCurrentState(){
    std::cout << "############## SimplePositionController State begin " << std::endl;
    std::cout << " angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " velocities " << in_robotstatus_var.velocities << std::endl;
    std::cout << " desJointAngles " << desJointAngles.angles << std::endl;
    std::cout << " jointAngles " << out_angles_var.angles << std::endl;
    std::cout << "############## SimplePositionController State end " << std::endl;
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(SimplePositionController)
