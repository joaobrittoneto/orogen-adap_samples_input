/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Seabotix.hpp"

using namespace adap_samples_input;

Seabotix::Seabotix(std::string const& name, TaskCore::TaskState initial_state)
    : SeabotixBase(name, initial_state)
{
}

Seabotix::Seabotix(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : SeabotixBase(name, engine, initial_state)
{
}

Seabotix::~Seabotix()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Seabotix.hpp for more detailed
// documentation about them.

bool Seabotix::configureHook()
{
    if (! SeabotixBase::configureHook())
        return false;
    else
       {
       	samplesInput = new SamplesInput();

          	return true;
       }
}
bool Seabotix::startHook()
{
    if (! SeabotixBase::startHook())
        return false;
    return true;
}
void Seabotix::updateHook()
{
    SeabotixBase::updateHook();

    base::samples::RigidBodyState						position_sample;
    base::samples::Joints 								forces_sample;

    static base::samples::RigidBodyState 				actual_RBS;
    static base::samples::RigidBodyAcceleration 		actual_RBA;

    static base::samples::Joints 						forces_output;

    adap_samples_input::DynamicAUV						dynamic;

    static std::queue<base::samples::RigidBodyState>	queueOfRBS;
    static std::queue<base::samples::Joints> 			queueOfForces;


    static base::samples::Joints 										last_forces_sample;
    static base::samples::RigidBodyState								last_position_sample;
    static base::Time 													beginMov;
    static base::Time 													beginForce;
    static bool															delay = true;
    static bool															update = false;

    static bool first_time = true;
    if(first_time)
    {
    	actual_RBS.position = Eigen::VectorXd::Zero(3);
    	actual_RBS.velocity = Eigen::VectorXd::Zero(3);
    	actual_RBA.acceleration = Eigen::VectorXd::Zero(3);
    	forces_output.elements.resize(6);
    	for(int i=0; i<6; i++)
    	{
    		forces_output.elements[i].raw = 0;
    		forces_output.elements[i].effort = 0;
    	}

    	if(!queueOfRBS.empty())
    		std::cout << "queueOfRBS not empty at the begin: "<< std::endl;
    	if(!queueOfForces.empty())
    		std::cout << "queueOfForces not empty at the begin: "<< std::endl;

   /* 	forces_sample.elements.resize(4);
    	last_forces_sample.elements.resize(4);
    	last_forces_sample.elements[1].effort = 0;
    	last_position_sample.position[0] = -2.2;
	*/
    	first_time = false;
    }

    // size of the queue 2*m+1. Important for the filter in the position and in compensating the delay
    const double m = 157; // 315 ≃ number of samples in a period of 0.2 rad/s w/ sample time of 0.05s (pi/0.2/0.05)
    const double size = 2*m+1;
    static bool doIt = false;

    if (_forces_samples.read(forces_sample) == RTT::NewData)
    {
    	samplesInput->Update_Force_Seabotix(forces_sample, queueOfForces, size, forces_output);
    }


    if (_position_samples.read(position_sample) == RTT::NewData)
    {
    	doIt = samplesInput->Update_Velocity_Seabotix(position_sample, queueOfRBS, size, actual_RBS, actual_RBA);
    }


    if(fmod(queueOfRBS.size(),2) == 1 && queueOfRBS.size() == size && queueOfForces.size() > 1 && doIt)
    {
    	bool aligned = samplesInput->Delay_Compensate(actual_RBS, queueOfForces, forces_output);

    	doIt = false;

    	if(aligned)
    	{
    		samplesInput->Agglomerate(forces_output,actual_RBS, actual_RBA, dynamic);
    		_dynamic.write(dynamic);
    		_velocity.write(actual_RBS);
    		_acceleration.write(actual_RBA);
    		_forces.write(forces_output);


    		//static int number = 0;
    		//number++;
    		//std::cout<< std::endl << "number of samples: "<< number << std::endl;

    	}
    }

    ////////////////////
    // Delay estimation
    ///////////////////
/*    if (forces_sample.elements[1].effort != last_forces_sample.elements[1].effort)
    {
    	beginForce = forces_sample.time;
    }
    last_forces_sample.elements[1].effort = forces_sample.elements[1].effort;

    if (fabs((position_sample.position[0]-last_position_sample.position[0])/position_sample.position[0]) >= 0.01 && delay && update)
    {
    	beginMov = position_sample.time;
    	delay = false;
    }
    last_position_sample.position[0] = position_sample.position[0];
    update = true;

    if (!delay)
    	std::cout<< std::endl << "delay: "<< (beginMov-beginForce) << std::endl;
*/

}
void Seabotix::errorHook()
{
    SeabotixBase::errorHook();
}
void Seabotix::stopHook()
{
    SeabotixBase::stopHook();
}
void Seabotix::cleanupHook()
{
    SeabotixBase::cleanupHook();
}
