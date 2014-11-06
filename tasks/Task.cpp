/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

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

    	last_sample.position[0] = 0;
    	last_sample.time = base::Time::now();
    	actual_sample.position[0] = 0;
    	actual_sample.initUnknown();

    	fist_sample.time = base::Time::now();
    	end_sample.time = base::Time::now();
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

	double delta_t;
	double position;
	double last_position;

	static std::queue<double> queueOfPosition;
	static std::queue<double> queueOfsamples;

     if (_position_samples.read(sample) == RTT::NewData)
    	{	// Position from [mm] to [m]
    	 if (sample.start_angle <= 0.100){
    	 	double angle = sample.start_angle;

    		actual_sample.position[0] = sample.ranges[0]*cos(angle);
    		actual_sample.position[0] = actual_sample.position[0]/1000;
    		actual_sample.time = sample.time;


    		//TODO Be careful with the outlier (initial case) and the amplitude.  Verify with sample 12645 (18:03:40.383727)
    		if ( (!queueOfPosition.empty()) && (queueOfPosition.back() != 0) &&
    				((actual_sample.position[0]/queueOfPosition.back()) < 0.05 ||
    				(actual_sample.position[0]/queueOfPosition.back() > 20)) )
    		    {
    			//std::cout << std::endl << "actual_sample.position[0]_before: "<< actual_sample.position[0] << std::endl;
    			actual_sample.position[0] = queueOfPosition.back();
    			//std::cout << "actual_sample.posititon[0]_after: "<< actual_sample.position[0] << std::endl<< std::endl;
    		    }

    		static bool endQueue = false;
    		if (sample.start_angle <= -0.09 && endQueue)
    			{fist_sample.time = sample.time;
    			 endQueue = false;
    			}
    		else if (sample.start_angle <= -0.09 && !endQueue)
    			{end_sample.time = sample.time;
    			 endQueue = true;
    			}

    	//	if (sample.start_angle <= -0.9)
    	//	{
    	//	double interval = (fist_sample.time.toSeconds() - end_sample.time.toSeconds());
    	//	std::cout << std::endl << "interval: "<< interval << std::endl;
    	//	std::cout << std::endl << "fist_sample.time: "<< fist_sample.time << std::endl;
    	//	std::cout << std::endl << "end_sample.time: "<< end_sample.time << std::endl;
    	//	std::cout << std::endl << "angle: "<< sample.start_angle << std::endl << std::endl;
    	//	}

    		 //////////////////////////////////////////

    	/*	 int sQueue = 10;
    		 static bool endQueue = false;
    		 if (angle == 0.100645 && endQueue == false)
    			 endQueue = true;

    		 if (endQueue)
    			 {
    			 double sample_position = actual_sample.position[0];
    			 samplesInput->Queue(sQueue, sample_position, queueOfsamples);
    			 }
	*/

    		 /////////////////////////////////////////


    		//filter value
    		double m = 100;
    		double size = 2*m+1;
    		position = actual_sample.position[0];
    		samplesInput->Queue(size, position, queueOfPosition);
    		//actual_sample.position[0] = samplesInput->Filter_1(queueOfPosition);
    		actual_sample.position[0] = samplesInput->Filter_2(queueOfPosition, size, 3, 0);
    		//actual_sample.velocity[0] = samplesInput->Filter_2(queueOfPosition, size, 3, 1);

/*
    		std::cout << std::endl << "Queue: "<< queueOfPosition.size() << std::endl;
    		std::cout << std::endl << "Queue Back: "<< queueOfPosition.back() << std::endl;
    		std::cout << std::endl << "Queue Front: "<< queueOfPosition.front() << std::endl;

*/
    		//position
    		position = actual_sample.position[0];
    		last_position = last_sample.position[0];
    		delta_t = (actual_sample.time.toSeconds() - last_sample.time.toSeconds());
    		delta_t = 0.065;


    		//std::cout << std::endl << "delta_t: "<< delta_t << std::endl;
    		//std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1) << std::endl << "actual_sample.time: "<< actual_sample.time.toSeconds() << std::endl;
    		//std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1) << std::endl << "last_sample.time: "<< last_sample.time.toSeconds() << std::endl;
    		std::cout << std::endl << "actual_sample.position[0]: "<< actual_sample.position[0] << std::endl;
    		std::cout << std::endl << "last_sample.position[0]: "<< last_sample.position[0] << std::endl;


    		actual_sample.velocity[0] = samplesInput->Deriv(position, last_position, delta_t);

  //  		std::cout << std::endl << "actual_sample.velocity[0]: "<< actual_sample.velocity[0] << std::endl;
 //   		std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1)<< std::endl << "actual_sample.time: "<< actual_sample.time.toSeconds() << std::endl << std::endl;

    		last_sample = actual_sample;
    		//samplesInput->Filter_2(queueOfPosition);
    	 }
    	}

    //if (actual_sample.isValidValue(actual_sample.position))
    	_velocity.write(actual_sample);
   // else _velocity.write(last_sample);

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
