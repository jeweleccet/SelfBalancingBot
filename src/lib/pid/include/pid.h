#ifndef ROBOT_PID_H
#define ROBOT_PID_H

#include <iostream>
#include <chrono>
using namespace std;
namespace robot::pid
{
    class Pid
    {
    public:
        Pid(double input, double output, double setPoint,
            double kp, double ki, double kd, int pOn, int controllerDirection);

        void setMode(int mode);
        bool compute();
        void setOutput(double min, double max);

        void setTunings(double kp, double ki , double kd, int pOn);
        void setTunings(double kp, double ki , double kd);

        void setControllerDirection(int );
        void setSampleTime(long long new_sample_time);

        /* Display Functions */
        double getKp();
        double getKi();
        double getKd();
        double getMode();
        double getDirection();

    private:

        void initialize();
        enum pidCategory_
        {
            Automatic,
            Manual,
            Direct,
            Reverse,
            POnM,
            PonE
        };

        double kp_;
        double ki_;
        double kd_;

        double input_;
        double output_;
        double setPoint_;

        int controllerDirection_;
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
