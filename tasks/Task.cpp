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

	adap_samples_input::DynamicAUV						dynamic;

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
	const double m = 79;
	const double size = 2*m+1;
	static bool doit = false;


	if (_forces_samples.read(forces_sample) == RTT::NewData)
	{
	   	samplesInput->Update_Force(forces_sample, queueOfForces, size, forces_output);
	}


    if (_position_samples.read(position_sample) == RTT::NewData)
    {
    	doit = samplesInput->Update_Velocity(position_sample, queueOfRBS, size, actual_RBS, actual_RBA);
    }


    if(fmod(queueOfRBS.size(),2) == 1 && queueOfRBS.size() == size && queueOfForces.size() > 1 && doit)
    {
       	bool aligned = samplesInput->Delay_Compensate(actual_RBS, queueOfForces, forces_output);

       	doit = false;

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
