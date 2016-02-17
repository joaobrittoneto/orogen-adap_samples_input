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
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
	delete inputAdap;
}

void Task::pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{

	base::samples::RigidBodyState transformed_rbs = pose_samples_sample;
	double step;

	// Convert velocities to body-frame
    if(transformed_rbs.sourceFrame == "body" && transformed_rbs.targetFrame != "body")
	{
    	transformed_rbs.velocity = transformed_rbs.orientation.inverse() * transformed_rbs.velocity;
	}

    if(handleMeasurement(transformed_rbs, step))
	{
    	inputAdap->Queue(gsize, transformed_rbs, queueOfRBS);
    	computeMeanStep(queueOfStep, step);
    	new_sample = true;
	}

}

void Task::forces_samplesCallback(const base::Time &ts, const ::base::samples::Joints &forces_samples_sample)
{

	base::samples::Joints forces = forces_samples_sample;

	if(handleMeasurement(forces))
	{
		if(_delay.get() > 0)
			forces.time = forces.time + base::Time::fromSeconds(_delay.get()); // Add delay
		//inputAdap->Queue(30*gsize, forces, queueOfForces);
		queueOfForces.push(forces);
	}

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
    	double step = _step.get();

    	// Number of samples should be a odd value
    	gsize = _number_samples.get();
    	if(fmod(gsize,2)==0)
    		gsize =+ 1;

    	double poly = _poly.get();
    	double pos_filter = _pos_filter.get();
    	bool smooth = _smooth.get();

    	inputAdap = new InputAdap();
    	inputAdap->UpdateSavGol(step, poly, pos_filter, smooth);

    	first_time = true;
    	meanStep = step;

    	new_sample = false;


		while(!queueOfRBS.empty())
			queueOfRBS.pop();
		while(!queueOfStep.empty())
			queueOfStep.pop();
		while(!queueOfForces.empty())
			queueOfForces.pop();

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

	base::samples::RigidBodyState 				actual_RBS;
	base::samples::RigidBodyAcceleration 		actual_LinRBA;
	base::samples::RigidBodyAcceleration 		actual_AngRBA;
	base::samples::Joints 						forces_output;
	forces_output.elements.resize(6);

    adap_samples_input::DynamicAUV dynamic;

    bool doIt = false;

    if(!queueOfRBS.empty() && !queueOfForces.empty())
    {

    	if(_filter_sav_gol.get() && new_sample)
    	{
    		if(_velocity_avalible.get())
    		{
    			doIt = inputAdap->calcAcceleration(queueOfRBS, actual_RBS, actual_LinRBA, actual_AngRBA);
    		}
    		else
			{
    			doIt = inputAdap->calcVelAcc(queueOfRBS, actual_RBS, actual_LinRBA, actual_AngRBA);
			}
    		new_sample = false;
    	}

    	if(doIt)
    	{
    		int back_queue = 0;
    		if(inputAdap->Delay_Compensate(actual_RBS, queueOfForces, forces_output, back_queue))
    		{
    			//std::cout << "queueOfForces.size() before: " << queueOfForces.size() << std::endl;
    			while(back_queue > 0)
    			{
    				queueOfForces.pop();
    				back_queue--;
    			}
    			//std::cout << "queueOfForces.size() after: " << queueOfForces.size() << std::endl;

				inputAdap->Agglomerate(forces_output,actual_RBS, actual_LinRBA, actual_AngRBA, dynamic);
				_dynamic.write(dynamic);
				_velocities.write(actual_RBS);
				_forces.write(forces_output);

    		}
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

bool Task::handleMeasurement(const base::samples::Joints &force_sample)
{
	for (int j=0; j < force_sample.elements.size(); j++)
	{
		if( isnan((double)force_sample.elements[j].effort))
			return false;
	}

	return true;

}

bool Task::handleMeasurement(const base::samples::RigidBodyState &rbs_sample, double &step)
{
	for (int j=0; j<3; j++)
	{
		if(_velocity_avalible.get() && (isnan((double)rbs_sample.velocity[j]) || isnan((double)rbs_sample.angular_velocity[j])) )
			{
				std::cout << "VEl is nan "<< std::endl;
				return false;
			}
		else if(!_velocity_avalible.get() && (isnan((double)rbs_sample.position[j]) || isnan((double)base::getEuler(rbs_sample.orientation)[j])))
				return false;

	}

	if(first_time)
	{
		lastPoseSample = rbs_sample;
		first_time = false;
	}

	if(lastPoseSample.time > rbs_sample.time)
		{
			//std::cout << "pose.time is Bigger  "<< std::endl;
			return false;
		}
	if(lastPoseSample.time == rbs_sample.time)
		{
			//std::cout << "pose.time is Equal  "<< std::endl;
			return false;
		}



	step = (rbs_sample.time - lastPoseSample.time).toSeconds();

	lastPoseSample = rbs_sample;

	return true;
}

void Task::updateStep(double estimatedstep)
{
	double step = inputAdap->getStep();
	double error = fabs(step-estimatedstep)/estimatedstep;

	if(error > 0.05)
	{
		inputAdap->setStep(estimatedstep);
	}
}

void Task::computeMeanStep(std::queue<double>	&queueOfStep, double step)
{
	if(!queueOfStep.empty() && queueOfStep.size()< gsize)
		meanStep = (meanStep*(queueOfStep.size()) + step)/(queueOfStep.size()+1);

	else if(!queueOfStep.empty() && queueOfStep.size()== gsize)
	{
		double auxstep = meanStep*queueOfStep.size() - queueOfStep.front();
		meanStep = (auxstep + step) / (queueOfStep.size());
	}

	inputAdap->Queue(gsize, step, queueOfStep);
	updateStep(meanStep);
}
