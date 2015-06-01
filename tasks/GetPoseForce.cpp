/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GetPoseForce.hpp"

using namespace adap_samples_input;

GetPoseForce::GetPoseForce(std::string const& name)
    : GetPoseForceBase(name)
{
}

GetPoseForce::GetPoseForce(std::string const& name, RTT::ExecutionEngine* engine)
    : GetPoseForceBase(name, engine)
{
}

GetPoseForce::~GetPoseForce()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GetPoseForce.hpp for more detailed
// documentation about them.

bool GetPoseForce::configureHook()
{
    if (! GetPoseForceBase::configureHook())
        return false;

    double sampleTime	= _step.get();
    aligned_data		= _aligned_data.get();

    inputAdap = new InputAdap(sampleTime);

    return true;
}
bool GetPoseForce::startHook()
{
    if (! GetPoseForceBase::startHook())
        return false;
    return true;
}
void GetPoseForce::updateHook()
{
    GetPoseForceBase::updateHook();

    static std::queue<base::samples::RigidBodyState>	queueOfRBS;
	static std::queue<base::samples::Joints> 			queueOfForces;

	static base::samples::RigidBodyState 				actual_RBS;
	static base::samples::RigidBodyAcceleration 		actual_LinRBA;
	static base::samples::RigidBodyAcceleration 		actual_AngRBA;

	static base::samples::Joints 						forces_sample;
	static base::samples::RigidBodyState				valid_rbs_sample;

//	static base::samples::RigidBodyState				last_rbs_sample;

	base::samples::RigidBodyState						rbs_sample;
	adap_samples_input::DynamicAUV						dynamic;

	// size of the queue 2*m+1. Important for calculating the acceleration and in compensating the delay
	const int m = 50; //
	const int size = 2*m+1;
	static bool doIt = false;

	 if(treatOutput && newForceSample)
	 {
		 inputAdap->Queue(5*size, forcesOut, queueOfForces);
		 //inputAdap->Update_Force(size, forcesOut, queueOfForces, queueForFilterForces);
		 forces_sample = forcesOut;
		 newForceSample = false;
	 }



	 if(_pose_samples.read(rbs_sample) == RTT::NewData)
	 {
		 // Data received as world-frame. Convert to body-frame
		if(rbs_sample.sourceFrame == "body" && rbs_sample.targetFrame != "body")
		{
			rbs_sample.velocity = rbs_sample.getTransform().rotation().inverse() * rbs_sample.velocity;
			//rbs_sample.angular_velocity = rbs_sample.getTransform().rotation().inverse() * rbs_sample.angular_velocity;
		}

		// Verify if las
		bool isvelnan = false;
		for (int j=0; j<3; j++)
		{
			if( isnan((double)valid_rbs_sample.velocity[j]) || isnan((double)valid_rbs_sample.angular_velocity[j]) )
				isvelnan = true;
		}
		// Get the newest data w/ the same timestamp and put in last_rbs_sample
		if (isvelnan || (rbs_sample.time - valid_rbs_sample.time).toSeconds() == 0)
		{
			valid_rbs_sample = rbs_sample;
		}
		// When the sample change timestamp, use the last update sample in last_rbs_sample
		else if((rbs_sample.time - valid_rbs_sample.time).toSeconds() > 0)
		{
			doIt = inputAdap->calcAcceleration(valid_rbs_sample, queueOfRBS, size, actual_RBS, actual_LinRBA, actual_AngRBA);

		//	std::cout << "sample step: "<< (valid_rbs_sample.time-last_rbs_sample.time).toSeconds() << std::endl;
		//	last_rbs_sample = valid_rbs_sample;
			valid_rbs_sample = rbs_sample;
		}
		// If the acceleration and alignment of forces and pose are not important
		if(!aligned_data)
				actual_RBS = rbs_sample;
	 }


	 if(fmod(queueOfRBS.size(),2) == 1 && queueOfRBS.size() == size && queueOfForces.size() >= size && doIt && aligned_data && treatOutput)
	{
		bool aligned = inputAdap->Delay_Compensate(actual_RBS, queueOfForces, forces_sample);

		doIt = false;

		if(aligned)
		{
			inputAdap->Agglomerate(forces_sample,actual_RBS, actual_LinRBA, actual_AngRBA, dynamic);
			_dynamic.write(dynamic);
			_velocity.write(actual_RBS);
			_lin_acceleration.write(actual_LinRBA);
			_ang_acceleration.write(actual_AngRBA);
			_forces.write(forces_sample);
		}
	}

	if(!aligned_data && treatOutput && !queueOfForces.empty())
	{
		base::samples::RigidBodyAcceleration 		nan_RBA;
		inputAdap->Agglomerate(queueOfForces.back(),actual_RBS, nan_RBA, nan_RBA, dynamic);
		_dynamic.write(dynamic);
		_velocity.write(actual_RBS);
		_forces.write(queueOfForces.back());
	}

	if(!aligned_data && !treatOutput)
	{
		base::samples::RigidBodyAcceleration 		nan_RBA;
		inputAdap->Agglomerate(forcesOut,actual_RBS, nan_RBA, nan_RBA, dynamic);
		_dynamic.write(dynamic);
		_velocity.write(actual_RBS);
	}



}
void GetPoseForce::errorHook()
{
    GetPoseForceBase::errorHook();
}
void GetPoseForce::stopHook()
{
    GetPoseForceBase::stopHook();
}
void GetPoseForce::cleanupHook()
{
    GetPoseForceBase::cleanupHook();
}
