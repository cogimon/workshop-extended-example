/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#ifndef EXTENDEDRTTCOMPONENT_HPP
#define EXTENDEDRTTCOMPONENT_HPP

// RTT header files. Might missing some or some be unused
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#define COMAN_RIGHT_ARM_DOF_SIZE 7

class ExampleRightArm: public RTT::TaskContext {
public:
    ExampleRightArm(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    // Declare ports and their datatypes
    // this time an input port is needed as well

    // Data flow:
    // input ports need dataflow

    // Actuall joint command to be sent over port:
    
    // helpers:
    double getSimulationTime();

    // operations:
    // are class methods that can be called from deployer

    // properties:
    RTT::Attribute<int> exampleAttribute;

};

#endif // SIMPLERTTCOMPONENT_HPP
