#include "pid.h"
using namespace robot::pid;

Pid::Pid(double input, double output, double setPoint,
         double kp, double ki, double kd, controllerDirection controllerDirection)
        :input_(input), output_(output), setPoint_(setPoint),
        kp_(kp), ki_(ki), kd_(kd), controllerDirection_(controllerDirection)
{
    inAuto_ = false;
    sampleTime_ = 100.0;
    Pid::setOutput(0, 255);

    Pid::setControllerDirection( controllerDirection_);
    Pid::setTunings(kp_, ki_, kd_);
    auto now = chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto current_time = duration.count();
    lastTime_ = current_time - sampleTime_;
}

bool Pid::compute()
{
    if(!inAuto_)
    {
        return false;
    }
    auto now = chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto current_time = duration.count();
    auto time_change = current_time - lastTime_;
    if(time_change >= sampleTime_)
    {
        auto error = setPoint_ - input_;
        auto dinput = input_ -lastInput_;
        outputSum_ += ki_*error;

        /*Add proportional on measurement, if p_on_m is specified*/
        if(!pOnE_)
        {
            outputSum_ -= kp_ * dinput;
        }

        if(outputSum_ > outMax_)
        {
            outputSum_ = outMax_;
        }
        else if(outputSum_ < outMin_)
        {
            outputSum_ = outMin_;
        }

        /*Add Proportional on Error, if P_ON_E is specified*/
        if(pOnE_)
        {
            output_ = kp_ * error;
        }
        else
        {
            output_ = 0;
        }

        /*Rest of PID output */
        output_ =outputSum_ - kd_ *dinput;
        if(outputSum_ > outMax_)
        {
            outputSum_ = outMax_;
        }
        else if(outputSum_ < outMin_)
        {
            outputSum_ = outMin_;
        }

        /*store old variables */
        lastInput_ = input_;
        lastTime_ =current_time;
        return true;

    }
    else
    {
        return false;
    }

}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/

void Pid::setTunings(double kp, double ki , double kd)
{
    if(kp < 0 || ki < 0 || kd < 0) return;

    auto sample_time_insec = sampleTime_/1000;
    kp_ =kp;
    ki_ = ki * sample_time_insec;
    kd_ = kd / sample_time_insec;
    if(controllerDirection_ == controllerDirection::Reverse)
    {
        kp_ = (0 - kp);
        ki_ = (0 - ki);
        kd_ = (0 - kd);
    }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void Pid::setSampleTime(long long  new_sample_time)
{
    if(new_sample_time > 0)
    {
        auto ratio = new_sample_time / sampleTime_;
        ki_ *= ratio;
        kd_ /= ratio;
        sampleTime_ = new_sample_time;
    }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void Pid::setOutput(double min, double max)
{
    if(min >= max) return;
    outMin_ = min;
    outMax_ = max;

    if(inAuto_)
    {
        if(output_ > outMax_) output_ = outMax_;
        else if(output_ < outMin_) output_ = outMin_;

        if(outputSum_ > outMax_) outputSum_ = outMax_;
        else if(outputSum_ < outMin_) outputSum_ = outMin_;
    }
}
/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void Pid::setMode(pidMode Mode)
{
    bool newAuto = (Mode == pidMode::Automatic);
    if(newAuto && !inAuto_)
    {  /*we just went from manual to auto*/
        Pid::initialize();
    }
    inAuto_ = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void Pid::initialize()
{
    outputSum_ = output_;
    lastInput_ = input_;
    if(outputSum_ > outMax_)
    {
        outputSum_ = outMax_;
    }
    else if(outputSum_ < outMin_)
    {
        outputSum_ = outMin_;
    }
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void Pid::setControllerDirection(controllerDirection direction)
{
    if(inAuto_ && direction !=controllerDirection_)
    {
        kp_ = (0 - kp_);
        ki_ = (0 - ki_);
        kd_ = (0 - kd_);
    }
    controllerDirection_ = direction;
}