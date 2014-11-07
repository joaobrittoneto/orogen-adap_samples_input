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

    	//last_sample.position[0] = 0;
    	//last_sample.time = base::Time::now();
    	//actual_sample.position[0] = 0;
    	//actual_sample.initUnknown();

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

	static base::samples::RigidBodyState actual_sample;
	static base::samples::RigidBodyState last_sample;
	static base::samples::LaserScan sample;

	static base::samples::RigidBodyState fist_sample;
	static base::samples::RigidBodyState end_sample;

	static std::queue<double> queueOfPosition;
	static std::queue<base::samples::RigidBodyState> queueOfsamples;

     if (_position_samples.read(sample) == RTT::NewData)
    	{
    	// if (/*sample.start_angle <= 0.100 || */isnan(sample.ranges[0]==0)){

    		// size of the queue 2*m+1
    		double m = 100;
    		double size = 2*m+1;


    		actual_sample = samplesInput->Convert(sample);

    		samplesInput->Remove_Outlier(queueOfsamples, actual_sample);

    		samplesInput->Queue(size, actual_sample, queueOfsamples);
    		if (fmod(queueOfsamples.size(),2) == 1){
/*
    		static bool endQueue = false;
    		//if (sample.start_angle <= -0.09 && endQueue)
    		if ( endQueue)
    			{fist_sample.time = sample.time;
    			 endQueue = false;
    			}
    		//else if (sample.start_angle <= -0.09 && !endQueue)
    		else if (!endQueue)
    			{end_sample.time = sample.time;
    			 endQueue = true;
    			}

    		//if (sample.start_angle <= -0.09)
    		//{
    		double interval = fabs(fist_sample.time.toSeconds() - end_sample.time.toSeconds());
    		std::cout << std::endl << "interval: "<< interval << std::endl;
    		std::cout << std::endl << "fist_sample.time: "<< fist_sample.time << std::endl;
    		std::cout << std::endl << "end_sample.time: "<< end_sample.time << std::endl;
    		std::cout << std::endl << "angle: "<< sample.start_angle << std::endl << std::endl;
    		//}
*/
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

    		//position = actual_sample.position[0];
    		//samplesInput->Queue(size, position, queueOfPosition);
    		//actual_sample.position[0] = samplesInput->Filter_1(queueOfPosition);
    		//actual_sample.position[0] = samplesInput->Filter_2(queueOfPosition, size, 3, 0);
    		// Position with will be take in account when applying the filter. May vary from -(queue.size-1)/2 to (queue.size-1)/2 where 0 is at the center of the queue.
    		double t;
    		t = 0;
    		//t = (queueOfPosition.size()-1)/2;
    		//actual_sample.position[0] = samplesInput->Filter_3(queueOfPosition, t, 3, 0);
    		//actual_sample.velocity[0] = samplesInput->Filter_2(queueOfPosition, size, 2, 1);
    		actual_sample = samplesInput->Filter_4(queueOfsamples, t, 3, 0);
/*
    		std::cout << std::endl << "Queue: "<< queueOfPosition.size() << std::endl;
    		std::cout << std::endl << "Queue Back: "<< queueOfPosition.back() << std::endl;
    		std::cout << std::endl << "Queue Front: "<< queueOfPosition.front() << std::endl;


*/
    		// avoid peak in velocity
    		if (queueOfsamples.size() == 5)
    			{last_sample = actual_sample;
    			 last_sample.time = base::Time::now()-actual_sample.time;
    			}
    		//position
    		position = actual_sample.position[0];
    		last_position = last_sample.position[0];
    		//delta_t = (actual_sample.time.toSeconds() - last_sample.time.toSeconds());
    		// Time step is not constant. Use a medium value from observed steps.
    		delta_t = 0.066;


    		std::cout << std::endl << "delta_t: "<< delta_t << std::endl;
    		std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1) << std::endl << "actual_sample.time: "<< actual_sample.time.toSeconds() << std::endl;
    		std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1) << std::endl << "last_sample.time: "<< last_sample.time.toSeconds() << std::endl;
    		std::cout << std::endl << "TASK actual_sample.position[0]: "<< actual_sample.position[0] << std::endl;
    		std::cout << std::endl << "TASK last_sample.position[0]: "<< last_sample.position[0] << std::endl;
    		std::cout << std::endl << "queueOfsamples.size(): "<< queueOfsamples.size() << std::endl;

    		actual_sample.velocity[0] = samplesInput->Deriv(position, last_position, delta_t);

    		//remove the interference of the initial values of the queue.
    		if (queueOfsamples.size()>=m && (actual_sample.velocity[0]/last_sample.velocity[0] > 5 || actual_sample.velocity[0]/last_sample.velocity[0] < 0.2))
    			last_sample.velocity[0] = actual_sample.velocity[0];

    		std::cout << std::endl << "TASK actual_sample.velocity[0]: "<< actual_sample.velocity[0] << std::endl;
 //   		std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1)<< std::endl << "actual_sample.time: "<< actual_sample.time.toSeconds() << std::endl << std::endl;

    		last_sample = actual_sample;
    		//samplesInput->Filter_2(queueOfPosition);
    	 }
    	}

    //if (actual_sample.isValidValue(actual_sample.position))
    if(fmod(queueOfsamples.size(),2) == 1)
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
