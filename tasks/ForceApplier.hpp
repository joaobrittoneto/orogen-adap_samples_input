/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ADAP_SAMPLES_INPUT_FORCEAPPLIER_TASK_HPP
#define ADAP_SAMPLES_INPUT_FORCEAPPLIER_TASK_HPP

#include "adap_samples_input/ForceApplierBase.hpp"
#include <base/JointState.hpp>

namespace adap_samples_input {

    /*! \class ForceApplier 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','adap_samples_input::ForceApplier')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class ForceApplier : public ForceApplierBase
    {
	friend class ForceApplierBase;
    protected:


	int numberOfThrusters;
	base::VectorXd CoeffPos;
    base::VectorXd CoeffNeg;
    base::MatrixXd TCM;
    std::vector<base::JointState::MODE> controlModes;
    std::vector<std::string> thrusterNames;
    double thrusterVoltage;
	// If false, will not write the output. The forces may be treated in other task if ForceApplier is used as subtask
    bool treatOutput;
    // In case the ForceApplier is used as a subtask, newForceSample as true means a new sample. Set as false in the main task after use the new data.
    bool newForceSample;

    base::samples::Joints forcesOut;
    base::samples::Joints thrusterForces;

    public:

    	void calcThrustersForces(base::samples::Joints &forcesThruster, base::VectorXd &forces);
    	void calcOutput(base::samples::Joints &out_cmd, base::VectorXd &forces);
    	bool checkControlInput(base::samples::Joints &forcesThruster);


        /** TaskContext constructor for ForceApplier
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        ForceApplier(std::string const& name = "adap_samples_input::ForceApplier");

        /** TaskContext constructor for ForceApplier 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        ForceApplier(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of ForceApplier
         */
	~ForceApplier();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

