/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Dagon.hpp"

using namespace adap_samples_input;

Dagon::Dagon(std::string const& name, TaskCore::TaskState initial_state)
    : DagonBase(name, initial_state)
{
}

Dagon::Dagon(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : DagonBase(name, engine, initial_state)
{
}

Dagon::~Dagon()
{
	delete samplesInputDagon;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Dagon.hpp for more detailed
// documentation about them.

bool Dagon::configureHook()
{
    if (! DagonBase::configureHook())
        return false;
    else
    {
    	double step		= _step.get();
       	double X_pos_Cv	= _Cv.get()[0];
    	double X_neg_Cv = _Cv.get()[1];
		double Y_pos_Cv = _Cv.get()[2];
		double Y_neg_Cv = _Cv.get()[3];
		Eigen::MatrixXd TCM = _TCM.get();

		 aligned_data			= _aligned_data.get();
		 pose_estimator_data	= _pose_estimator_data.get();

		samplesInputDagon = new SamplesInputDagon(step, X_pos_Cv, X_neg_Cv, Y_pos_Cv, Y_neg_Cv, TCM);

		std::cout << "TCM: "<< std::endl << TCM<< std::endl;

		return true;
    }
}
bool Dagon::startHook()
{
    if (! DagonBase::startHook())
        return false;
    return true;
}
void Dagon::updateHook()
{
    DagonBase::updateHook();

    base::samples::RigidBodyState						rbs_sample;
    base::samples::RigidBodyState						dvl_sample;
    base::samples::RigidBodyState						orientation_sample;
   	base::samples::Joints 								forces_sample;

	static base::samples::RigidBodyState 				actual_RBS;
	static base::samples::RigidBodyAcceleration 		actual_LinRBA;
	static base::samples::RigidBodyAcceleration 		actual_AngRBA;

	static base::samples::Joints 						forces_output;

	adap_samples_input::DynamicAUV						dynamic;

	static std::queue<base::samples::RigidBodyState>	queueOfRBS;
	static std::queue<base::samples::Joints> 			queueOfForces;

	static base::samples::RigidBodyState				last_rbs_sample;
	static base::samples::Joints 						last_forces_sample;


	static bool first_time = true;
	if(first_time)
	{
		actual_RBS.position = Eigen::VectorXd::Zero(3);
		actual_RBS.velocity = Eigen::VectorXd::Zero(3);
		actual_LinRBA.acceleration = Eigen::VectorXd::Zero(3);
		actual_AngRBA.acceleration = Eigen::VectorXd::Zero(3);
		forces_output.elements.resize(6);
		for(int i=0; i<6; i++)
		{
			forces_output.elements[i].raw = 0;
			forces_output.elements[i].speed = 0;
			forces_output.elements[i].effort = 0;
		}

		if(!queueOfRBS.empty())
			std::cout << "queueOfRBS not empty at the begin: "<< std::endl;
		if(!queueOfForces.empty())
			std::cout << "queueOfForces not empty at the begin: "<< std::endl;
		first_time = false;
	}


	// size of the queue 2*m+1. Important for the filter in the position and in compensating the delay
	const double m = 10; //
	const double size = 2*m+1;
	static bool doIt = false;


	if (_forces_samples.read(forces_sample) == RTT::NewData)
	{
		samplesInputDagon->Update_Force_Dagon(forces_sample, queueOfForces, 5*size, forces_output);

		//std::cout << "sample_time_force: "<< (forces_sample.time-last_forces_sample.time).toSeconds() << std::endl;
		//std::cout << "force_interval_time: "<< queueOfForces.back().time << "   "<< queueOfForces.front().time <<std::endl;
		//std::cout << "velX: "<< (forces_sample.elements[3].speed) << std::endl << std::endl;

		last_forces_sample = forces_sample ;

	}

	if (pose_estimator_data)
	{
		if (_position_samples.read(rbs_sample) == RTT::NewData)
		{
			// Data received as world-frame. Convert to body-frame
			if(rbs_sample.sourceFrame == "body" && rbs_sample.targetFrame != "body")
			{
				rbs_sample.velocity = rbs_sample.getTransform().rotation().inverse() * rbs_sample.velocity;
				//rbs_sample.angular_velocity = rbs_sample.getTransform().rotation().inverse() * rbs_sample.angular_velocity;
			}


			bool isvelnan = false;
			for (int j=0; j<3; j++)
			{
				if( isnan((double)last_rbs_sample.velocity[j]) || isnan((double)last_rbs_sample.angular_velocity[j]) )
					isvelnan = true;
			}
			// Get the newest data w/ the same timestamp and put in last_rbs_sample
			if (isvelnan || (rbs_sample.time - last_rbs_sample.time).toSeconds() == 0)
			{
				last_rbs_sample = rbs_sample;
			}
			// When the sample change timestamp, use the last update sample in last_rbs_sample
			else if((rbs_sample.time - last_rbs_sample.time).toSeconds() > 0)
			{
				doIt = samplesInputDagon->Update_Velocity_Dagon(last_rbs_sample, queueOfRBS, size, actual_RBS, actual_LinRBA, actual_AngRBA);

				//std::cout << "sample step: "<< (rbs_sample.time-last_rbs_sample.time).toSeconds() << std::endl;
				last_rbs_sample = rbs_sample;
			}

			if(!aligned_data)
					actual_RBS = rbs_sample;
		}
	}

	if(!pose_estimator_data)
	{
		// dvl sensor mounting at 45 degree in the robot
		if (_dvl_samples.read(dvl_sample) == RTT::NewData)
		{
			actual_RBS.velocity[0] = dvl_sample.velocity[0]*cos(M_PI/4) + dvl_sample.velocity[1]*cos(3*M_PI/4);
			actual_RBS.velocity[1] = dvl_sample.velocity[0]*cos(3*M_PI/4) + dvl_sample.velocity[1]*cos(5*M_PI/4);
			actual_RBS.velocity[2] = dvl_sample.velocity[2];
			if (actual_RBS.time	< dvl_sample.time)
				actual_RBS.time	= dvl_sample.time;
		}
		if (_orientation_samples.read(orientation_sample) == RTT::NewData)
		{
			actual_RBS.angular_velocity[0] = orientation_sample.angular_velocity[0];
			actual_RBS.angular_velocity[1] = orientation_sample.angular_velocity[0];
			actual_RBS.angular_velocity[2] = orientation_sample.angular_velocity[2];
			if (actual_RBS.time	< orientation_sample.time)
				actual_RBS.time	= orientation_sample.time;
		}
	}



	if(fmod(queueOfRBS.size(),2) == 1 && queueOfRBS.size() == size && queueOfForces.size() > 1 && doIt && aligned_data)
	{
		bool aligned = samplesInputDagon->Delay_Compensate(actual_RBS, queueOfForces, forces_output);

		doIt = false;

		if(aligned)
		{
			samplesInputDagon->Agglomerate(forces_output,actual_RBS, actual_LinRBA, actual_AngRBA, dynamic);
			_dynamic.write(dynamic);
			_velocity.write(actual_RBS);
			_lin_acceleration.write(actual_LinRBA);
			_ang_acceleration.write(actual_AngRBA);
			_forces.write(forces_output);
		}
	}

	if(!aligned_data)
	{
		base::samples::RigidBodyAcceleration 		nan_RBA;
		samplesInputDagon->Agglomerate(forces_output,actual_RBS, nan_RBA, nan_RBA, dynamic);
		_dynamic.write(dynamic);
		_velocity.write(actual_RBS);
		_forces.write(forces_output);
	}

}
void Dagon::errorHook()
{
    DagonBase::errorHook();
}
void Dagon::stopHook()
{
    DagonBase::stopHook();
}
void Dagon::cleanupHook()
{
    DagonBase::cleanupHook();
}
