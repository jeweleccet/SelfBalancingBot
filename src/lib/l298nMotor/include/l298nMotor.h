#ifndef SELFBALANCINGBOT_L298NMOTOR_H
#define SELFBALANCINGBOT_L298NMOTOR_H
#include <chrono>
#include <thread>
#include "JetsonGPIO.h"

namespace robot::l298n
{
    class L298n
    {
    private:
        int _ena, _in1, _in2, _enb, _in3, _in4;
        int _currentSpeed;
        double _motorAConst, _motorBConst;
        long map(long x, long in_min, long in_max, long out_min, long out_max);

    public:
        L298n(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst);
        void move(int leftSpeed, int rightSpeed, int minAbsSpeed);
        void move(int speed);
        void move(int speed, int minAbsSpeed);
        void turnLeft(int speed, bool kick);
        void turnRight(int speed, bool kick);
        void stopMoving();

    };
}

#endif //SELFBALANCINGBOT_L298NMOTOR_H
