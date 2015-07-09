/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GetPoseForcePeriodic.hpp"

using namespace adap_samples_input;

GetPoseForcePeriodic::GetPoseForcePeriodic(std::string const& name)
    : GetPoseForcePeriodicBase(name)
{
}

GetPoseForcePeriodic::GetPoseForcePeriodic(std::string const& name, RTT::ExecutionEngine* engine)
    : GetPoseForcePeriodicBase(name, engine)
{
}

GetPoseForcePeriodic::~GetPoseForcePeriodic()
{
}


void GetPoseForcePeriodic::pose_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{

	base::samples::RigidBodyState transformed_rbs = pose_samples_sample;

	// Convert velocities to body-frame
    if(transformed_rbs.sourceFrame == "body" && transformed_rbs.targetFrame != "body")
	{
    	transformed_rbs.velocity = transformed_rbs.orientation.inverse() * transformed_rbs.velocity;
	}

    if(handleMeasurement(transformed_rbs, sampleTime))
	{
    	dataModel->EnqueueRBS(transformed_rbs);
	}

}

void GetPoseForcePeriodic::forces_samplesCallback(const base::Time &ts, const ::base::samples::Joints &forces_samples_sample)
{

	base::samples::Joints forces = forces_samples_sample;

	if(handleMeasurement(forces, sampleTime))
	{
		dataModel->EnqueueForce(forces);
	}

}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GetPoseForcePeriodic.hpp for more detailed
// documentation about them.

bool GetPoseForcePeriodic::configureHook()
{
    if (! GetPoseForcePeriodicBase::configureHook())
        return false;

    sampleTime			= _step.get();
    double polynomial	= _poly.get();
    double halfSize		= _halfSize.get();
    double position		= _posFilter.get();


    //adap_method		= _adap_method.get();

    dataModel = new DataModel(sampleTime, polynomial, halfSize, position);

    return true;
}
bool GetPoseForcePeriodic::startHook()
{
    if (! GetPoseForcePeriodicBase::startHook())
        return false;
    return true;
}
void GetPoseForcePeriodic::updateHook()
{
    GetPoseForcePeriodicBase::updateHook();

    base::samples::RigidBodyAcceleration 	lin_RBA;
    base::samples::RigidBodyAcceleration 	ang_RBA;
    base::samples::RigidBodyState 			output_RBS;
    base::samples::Joints 					output_force;
    adap_samples_input::DynamicAUV			output_dyn;

    static base::samples::RigidBodyAcceleration 	last_lin_RBA;
	static base::samples::RigidBodyAcceleration 	last_ang_RBA;
	static base::samples::RigidBodyState 			last_output_RBS;
	static base::samples::Joints 					last_output_force;
	static adap_samples_input::DynamicAUV			last_output_dyn;
	static bool first_time = true;

    static base::samples::RigidBodyState lastSample;
    static base::Time actualTime = base::Time::fromSeconds(0);
    bool newSample;

    actualTime = actualTime + base::Time::fromSeconds(TaskContext::getPeriod());

    if(!dataModel->queueOfRBS.empty() && !dataModel->queueOfForces.empty())
    {
//    	if(adap_method)
//		{
//
//			output_RBS		= dataModel->queueOfRBS.back();
//			output_force	= dataModel->queueOfForces.back();
//
////			if(fabs((output_RBS.time - output_force.time).toSeconds()) > 0.05)
////			{
////				dataModel->Delay_Compensate(output_RBS, output_force);
////				std::cout << "Diff time: "<< (output_RBS.time - output_force.time).toSeconds() << std::endl;
////			}
//
//			dataModel->Agglomerate(output_force, output_RBS, lin_RBA, ang_RBA, output_dyn);
//
//			_dynamic.write(output_dyn);
//		}


		if(dataModel->calcAcceleration(output_RBS, lin_RBA, ang_RBA))
		{
			if(dataModel->Delay_Compensate(output_RBS, output_force))
			{
				dataModel->Agglomerate(output_force, output_RBS, lin_RBA, ang_RBA, output_dyn);

				std::cout << "step pose "<< (output_RBS.time - lastSample.time).toSeconds()  << std::endl;
				lastSample = output_RBS;

				if(actualTime >= actualStep)
				{
					std::cout << "actualStep "<< (actualStep).toSeconds()  << std::endl;

				    last_lin_RBA		= lin_RBA;
					last_ang_RBA		= ang_RBA;
					last_output_RBS		= output_RBS;
					last_output_force	= output_force;
					last_output_dyn		= output_dyn;
					actualTime = base::Time::fromSeconds(0);

					_velocity.write(output_RBS);
					_effort.write(output_force);
					_dynamic.write(output_dyn);
				}
//				else if(!first_time)
//				{
//					_velocity.write(last_output_RBS);
//					_effort.write(last_output_force);
//					_dynamic.write(last_output_dyn);
//					first_time = false;
//					//std::cout << "step pose "<< (output_RBS.time - lastSample.time).toSeconds()  << std::endl;
//				}
			}


		}
	}


}
void GetPoseForcePeriodic::errorHook()
{
    GetPoseForcePeriodicBase::errorHook();
}
void GetPoseForcePeriodic::stopHook()
{
    GetPoseForcePeriodicBase::stopHook();
}
void GetPoseForcePeriodic::cleanupHook()
{
    GetPoseForcePeriodicBase::cleanupHook();
}

bool GetPoseForcePeriodic::handleMeasurement(base::samples::RigidBodyState &rbs_sample, double &step)
{

	base::samples::RigidBodyState periodic_pose;
	static bool first_time = true;

	for (int j=0; j<3; j++)
	{
		if( isnan((double)rbs_sample.velocity[j]) || isnan((double)rbs_sample.angular_velocity[j]) )
			{
				std::cout << "measured VEl is nan "<< std::endl;
				return false;
			}
	}

	if(lastPoseSample.time > rbs_sample.time)
		{
			//std::cout << "measured pose.time is Bigger  "<< std::endl;
			return false;
		}
	if(lastPoseSample.time == rbs_sample.time)
		{
			//std::cout << "measured pose.time is Equal  "<< std::endl;
			return false;
		}

	if((rbs_sample.time - lastPoseSample.time).toSeconds() < step)
		{
			//std::cout << "dropped pose. step:  " << (rbs_sample.time - lastPoseSample.time).toSeconds()<< std::endl;
			return false;
		}
	else
	{
		if(first_time)
		{
			lastPoseSample = rbs_sample;
			first_time = false;
			return true;
		}

		periodic_pose = getPeriodicPose(lastPoseSample, rbs_sample);

		rbs_sample = periodic_pose;
		actualStep = rbs_sample.time - lastPoseSample.time;
		//std::cout << "step pose "<< (rbs_sample.time - lastPoseSample.time).toSeconds()  << std::endl;
		lastPoseSample = rbs_sample;

		return true;
	}


}

bool GetPoseForcePeriodic::handleMeasurement(const base::samples::Joints &force_sample, double &step)
{
	for (int j=0; j < force_sample.elements.size(); j++)
	{
		if( isnan((double)force_sample.elements[j].effort))
			return false;
	}


	if(lastForceSample.time > force_sample.time)
		{
			//std::cout << "model pose.time is Bigger  "<< std::endl;
			return false;
		}
	if(lastForceSample.time == force_sample.time)
		{
			//std::cout << "model pose.time is Equal  "<< std::endl;
			return false;
		}

//	if((force_sample.time - lastForceSample.time).toSeconds() < step)
//		{
//			std::cout << "dropped force. step:  "<< (force_sample.time - lastForceSample.time).toSeconds() << std::endl;
//			return false;
//		}

	//std::cout << "step force "<< (force_sample.time - lastForceSample.time).toSeconds() << std::endl;
	lastForceSample = force_sample;

	return true;
}

base::samples::RigidBodyState GetPoseForcePeriodic::getPeriodicPose(const base::samples::RigidBodyState &old_sample, const base::samples::RigidBodyState &new_sample)
{
	base::samples::RigidBodyState output;
	float deltaLin;
	float deltaAng;
	// Force's interpolation
	for(int j=0; j<3; j++)
	{
		deltaLin = (new_sample.velocity[j] - old_sample.velocity[j]) * sampleTime
						/ ((new_sample.time - old_sample.time).toSeconds());

		deltaAng = (new_sample.angular_velocity[j] - old_sample.angular_velocity[j]) * sampleTime
						/ ((new_sample.time - old_sample.time).toSeconds());

		output.velocity[j] = deltaLin + old_sample.velocity[j];
		output.angular_velocity[j] = deltaAng + old_sample.angular_velocity[j];
	}
	output.time = old_sample.time + base::Time::fromSeconds(sampleTime);
	return output;
}
