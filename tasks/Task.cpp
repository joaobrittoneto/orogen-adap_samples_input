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

	static std::queue<base::samples::RigidBodyState> queueOfsamples;

     if (_position_samples.read(sample) == RTT::NewData)
    	{
    		// size of the queue 2*m+1
    		double m = 100;
    		double size = 2*m+1;


    		actual_sample = samplesInput->Convert(sample);

    		samplesInput->Remove_Outlier(queueOfsamples, actual_sample);

    		samplesInput->Queue(size, actual_sample, queueOfsamples);
    		// Filter works for odd values of queue.size
    		if (fmod(queueOfsamples.size(),2) == 1)
    		{

    			// Position that will be take in account when applying the filter. May vary from -(queue.size-1)/2 to (queue.size-1)/2 where 0 is at the center of the queue.
    			double t;
    			t = 0;
    			// Savitzky-Golay filter. Filter(queue, t, n, s). queue: RBS. t_th position in queue to be take in account. Converge for a polynomial n_th order. s_th derivative
    			actual_sample = samplesInput->Filter_4(queueOfsamples, t, 3, 0);

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

    			actual_sample.velocity[0] = samplesInput->Deriv(position, last_position, delta_t);

    			//remove the interference of the initial values of the queue.
    			if (queueOfsamples.size() > m &&
    					(((fabs(actual_sample.velocity[0]/last_sample.velocity[0]) > 2 || fabs(actual_sample.velocity[0]/last_sample.velocity[0]) < 0.5)
    							&& fabs(last_sample.velocity[0]) > 0.1)	||
    							(fabs(last_sample.velocity[0])<= 0.1 && fabs(actual_sample.velocity[0])>= 0.2)
    						))
    				{
    				actual_sample.velocity[0] = last_sample.velocity[0];
    				}

    			last_sample = actual_sample;

    		}
    	}


    if(fmod(queueOfsamples.size(),2) == 1)
    	_velocity.write(actual_sample);


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
