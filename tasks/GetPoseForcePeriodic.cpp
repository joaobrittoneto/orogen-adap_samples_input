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


void GetPoseForcePeriodic::pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_sample)
{
//	base::samples::RigidBodyState body2world;
//	    std::cout << "pose_samples "<< std::endl;
//	    if (!_body2world.get(ts, body2world))
//	        {
//	    	std::cout << "skip, have no " << _body2world.getSourceFrame() << "2" << _body2world.getTargetFrame() << " transformation sample!" << std::endl;
//	    		return;
//	        }
	base::samples::RigidBodyState transformed_rbs = pose_samples_sample;

//    std::cout << "pose_Vx: "<< transformed_rbs.velocity[0] << std::endl;

    if(transformed_rbs.sourceFrame == "body" && transformed_rbs.targetFrame != "body")
    	{
    		transformed_rbs.velocity = transformed_rbs.orientation.inverse() * transformed_rbs.velocity;
    	}

//    std::cout << "2_pose_Vx: "<< transformed_rbs.velocity[0] << std::endl;

    bool isvelnan = false;
	for (int j=0; j<3; j++)
	{
		if( isnan((double)transformed_rbs.velocity[j]) || isnan((double)transformed_rbs.angular_velocity[j]) )
			isvelnan = true;
	}
	if(!isvelnan)
	{
		dataModel->EnqueueRBS(transformed_rbs);
	}

}

void GetPoseForcePeriodic::forces_samplesTransformerCallback(const base::Time &ts, const ::base::samples::Joints &forces_samples_sample)
{

	base::samples::Joints forces = forces_samples_sample;
	bool isvelnan = false;
	for (int j=0; j < forces.elements.size(); j++)
		{
			if( isnan((double)forces.elements[j].effort))
				isvelnan = true;
		}
	if(!isvelnan)
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

    double sampleTime	= _step.get();
    double polynomial	= _poly.get();
    double halfSize		= _halfSize.get();
    double position		= _posFilter.get();


    adap_method		= _adap_method.get();

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

    if(!dataModel->queueOfRBS.empty() && !dataModel->queueOfForces.empty())
    {
    	if(adap_method)
		{

			output_RBS		= dataModel->queueOfRBS.back();
			output_force	= dataModel->queueOfForces.back();

//			if(fabs((output_RBS.time - output_force.time).toSeconds()) > 0.05)
//			{
//				dataModel->Delay_Compensate(output_RBS, output_force);
//				std::cout << "Diff time: "<< (output_RBS.time - output_force.time).toSeconds() << std::endl;
//			}

			dataModel->Agglomerate(output_force, output_RBS, lin_RBA, ang_RBA, output_dyn);

			_dynamic.write(output_dyn);
		}

		else if(dataModel->calcAcceleration(output_RBS, lin_RBA, ang_RBA))
		{
			if(dataModel->Delay_Compensate(output_RBS, output_force))
			{
				dataModel->Agglomerate(output_force, output_RBS, lin_RBA, ang_RBA, output_dyn);
				_dynamic.write(output_dyn);
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
