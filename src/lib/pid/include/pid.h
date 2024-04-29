#ifndef ROBOT_PID_H
#define ROBOT_PID_H

#include <iostream>
#include <chrono>
using namespace std;
namespace robot::pid
{
    enum pidMode
    {
        Automatic = 0,
        Manual,
    };

    enum controllerDirection
    {
        Direct = 1,
        Reverse = -1
    };

    class Pid
    {
    public:
        Pid(double input, double output, double setPoint,
            double kp, double ki, double kd, controllerDirection controllerDirection);

        void setMode(pidMode mode);
        bool compute();
        void setOutput(double min, double max);

        void setTunings(double kp, double ki , double kd);

        void setControllerDirection(controllerDirection direction );
        void setSampleTime(long long new_sample_time);

        /* Display Functions */
        double getKp();
        double getKi();
        double getKd();
        double getMode();
        double getDirection();

    private:

        void initialize();


        double kp_;
        double ki_;
        double kd_;

        double input_;
        double output_;
        double setPoint_;

        controllerDirection controllerDirection_;
        int pOn_;

        long long lastTime_;
        double outputSum_, lastInput_;

        long long sampleTime_;
        double outMin_, outMax_;
        bool pOnE_;
        bool inAuto_ ;

    };

}




#endif //ROBOT_PID_H
