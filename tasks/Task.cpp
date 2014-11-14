/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

/******************************************************************************/
/*  Preparing data from Avalon to apply the adaptive parameters identification
/*
/*
/* PURPOSE --- Filter the position signal, derived it to get the velocity and
/* 				compute the forces applied in the auv
/*
/*  João Da Costa Britto Neto
/*  joao.neto@dfki.de
/*  DFKI - BREMEN 2014
/*****************************************************************************/


#include "Task.hpp"

using namespace adap_samples_input;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
	delete samplesInput;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    else
    {
    	samplesInput = new SamplesInput();

       	return true;
    }

}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    base::samples::LaserScan 							position_sample;
    base::samples::Joints 								forces_sample;

	static base::samples::RigidBodyState 				actual_RBS;
	static base::samples::RigidBodyAcceleration 		actual_RBA;

	static base::samples::Joints 						forces_output;

	static std::queue<base::samples::RigidBodyState>	queueOfRBS;
	static std::queue<base::samples::Joints> 			queueOfForces;

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
		first_time = false;
	}

	// size of the queue 2*m+1. Important for the filter in the position and in compensating the delay
	const double m = 100;
	const double size = 2*m+1;

    if (_position_samples.read(position_sample) == RTT::NewData)
    {
    	samplesInput->Update_Velocity(position_sample, queueOfRBS, size, actual_RBS, actual_RBA);
    }


    if (_forces_samples.read(forces_sample) == RTT::NewData)
       {
       	samplesInput->Update_Force(forces_sample, queueOfForces, m, forces_output);


       	//std::cout << "forces_input.time: "<< forces_output.time.toString() << std::endl;
       	//std::cout << "forces_output.surge: "<< forces_output.elements[0].effort << std::endl;
       	//std::cout << "forces_output.sway: "<< forces_output.elements[1].effort << std::endl;
       	//std::cout << "forces_output.heave: "<< forces_output.elements[2].effort << std::endl;
       	//std::cout << "forces_output.roll: "<< forces_output.elements[3].effort << std::endl;
       	//std::cout << "forces_output.pitch: "<< forces_output.elements[4].effort << std::endl;
       	//std::cout << "forces_output.yaw: "<< forces_output.elements[5].effort << std::endl;
       }


    //if(queueOfRBS.size() == size || queueOfForces.size() == m)
    if(fmod(queueOfRBS.size(),2) == 1 && queueOfForces.size() > 1 )
    {
    	samplesInput->Delay_Compensate(actual_RBS, queueOfForces, forces_output);

    	std::cout << "velocity.time: "<< actual_RBS.time.toString() << std::endl;
    	_velocity.write(actual_RBS);
    	_acceleration.write(actual_RBA);

    	//std::cout << "forces_output.surge: "<< forces_output.elements[0].effort << std::endl;
    	//std::cout << "forces_output.sway: "<< forces_output.elements[1].effort << std::endl;
    	//std::cout << "forces_output.heave: "<< forces_output.elements[2].effort << std::endl;
    	//std::cout << "forces_output.roll: "<< forces_output.elements[3].effort << std::endl;
    	//std::cout << "forces_output.pitch: "<< forces_output.elements[4].effort << std::endl;
    	//std::cout << "forces_output.yaw: "<< forces_output.elements[5].effort << std::endl;

    	std::cout << "forces_output.time: "<< forces_output.time.toString() << std::endl<< std::endl;
    	_forces.write(forces_output);

    }



}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
