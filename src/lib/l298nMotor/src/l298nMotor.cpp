#include "l298nMotor.h"
#include <algorithm>
#include <map>
using namespace robot::l298n;
using namespace std;
L298n::L298n(int ena, int in1, int in2, int enb, int in3, int in4, double motorAConst, double motorBConst)
{
    _motorAConst = motorAConst;
    _motorBConst = motorBConst;

    _ena = ena;
    _in1 = in1;
    _in2 = in2;
    _enb = enb;
    _in3 = in3;
    _in4 = in4;
    GPIO::setmode(GPIO::BOARD);

    GPIO::setup(_ena, GPIO::OUT);
    GPIO::setup(_in1, GPIO::OUT);
    GPIO::setup(_in2, GPIO::OUT);

    GPIO::setup(_enb, GPIO::OUT);
    GPIO::setup(_in3, GPIO::OUT);
    GPIO::setup(_in4, GPIO::OUT);
}


void L298n::move(int leftSpeed, int rightSpeed, int minAbsSpeed)
{
    if (rightSpeed < 0)
    {
        rightSpeed = min(rightSpeed, -1*minAbsSpeed);
        rightSpeed = max(rightSpeed, -255);
    }
    else if (rightSpeed > 0)
    {
        rightSpeed = max(rightSpeed, minAbsSpeed);
        rightSpeed = min(rightSpeed, 255);
    }

    int realRightSpeed = map(abs(rightSpeed), 0, 255, minAbsSpeed, 255);

    if (leftSpeed < 0)
    {
        leftSpeed = min(leftSpeed, -1*minAbsSpeed);
        leftSpeed = max(leftSpeed, -255);
    }
    else if (leftSpeed > 0)
    {
        leftSpeed = max(leftSpeed, minAbsSpeed);
        leftSpeed = min(leftSpeed, 255);
    }

    int realLeftSpeed = map(abs(leftSpeed), 0, 255, minAbsSpeed, 255);

    GPIO::output(_in3, rightSpeed > 0 ? GPIO::HIGH : GPIO::LOW);
    GPIO::output(_in4, rightSpeed > 0 ? GPIO::LOW : GPIO::HIGH);
    GPIO::output(_in1, leftSpeed > 0 ? GPIO::HIGH : GPIO::LOW);
    GPIO::output(_in2, leftSpeed > 0 ? GPIO::LOW : GPIO::HIGH);
    //analogWrite(_ena, realRightSpeed * _motorAConst);
    //analogWrite(_enb, realLeftSpeed * _motorBConst);
}


void L298n::move(int speed, int minAbsSpeed)
{
    int direction = 1;

    if (speed < 0)
    {
        direction = -1;

        speed = min(speed, -1*minAbsSpeed);
        speed = max(speed, -255);
    }
    else
    {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }

    if (speed == _currentSpeed) return;

    int realSpeed = max(minAbsSpeed, abs(speed));

    GPIO::output(_in1, speed > 0 ? GPIO::HIGH : GPIO::LOW);
    GPIO::output(_in2, speed > 0 ? GPIO::LOW : GPIO::HIGH);
    GPIO::output(_in3, speed > 0 ? GPIO::HIGH : GPIO::LOW);
    GPIO::output(_in4, speed > 0 ? GPIO::LOW : GPIO::HIGH);
    //analogWrite(_ena, realSpeed * _motorAConst);
    //analogWrite(_enb, realSpeed * _motorBConst);

    _currentSpeed = direction * realSpeed;
}


void L298n::move(int speed)
{
    if (speed == _currentSpeed) return;

    if (speed > 255) speed = 255;
    else if (speed < -255) speed = -255;

    GPIO::output(_in1, speed > 0 ? GPIO::HIGH : GPIO::LOW);
    GPIO::output(_in2, speed > 0 ? GPIO::LOW : GPIO::HIGH);
    GPIO::output(_in3, speed > 0 ? GPIO::HIGH : GPIO::LOW);
    GPIO::output(_in4, speed > 0 ? GPIO::LOW : GPIO::HIGH);
    //analogWrite(_ena, abs(speed) * _motorAConst);
    //analogWrite(_enb, abs(speed) * _motorBConst);

    _currentSpeed = speed;
}


void L298n::turnLeft(int speed, bool kick)
{
    GPIO::output(_in1, GPIO::HIGH);
    GPIO::output(_in2, GPIO::LOW);
    GPIO::output(_in3, GPIO::LOW);
    GPIO::output(_in4, GPIO::HIGH);

    if (kick)
    {
        //analogWrite(_ena, 255);
        //analogWrite(_enb, 255);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    //analogWrite(_ena, speed * _motorAConst);
    //analogWrite(_enb, speed * _motorBConst);
}


void L298n::turnRight(int speed, bool kick)
{
    GPIO::output(_in1, GPIO::LOW);
    GPIO::output(_in2, GPIO::HIGH);
    GPIO::output(_in3, GPIO::HIGH);
    GPIO::output(_in4, GPIO::LOW);

    if (kick)
    {
        //analogWrite(_ena, 255);
        //analogWrite(_enb, 255);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    //analogWrite(_ena, speed * _motorAConst);
    //analogWrite(_enb, speed * _motorBConst);
}


void L298n::stopMoving()
{
    GPIO::output(_in1, GPIO::LOW);
    GPIO::output(_in2, GPIO::LOW);
    GPIO::output(_in3, GPIO::LOW);
    GPIO::output(_in4, GPIO::LOW);
    GPIO::output(_ena, GPIO::HIGH);
    GPIO::output(_enb, GPIO::HIGH);

    _currentSpeed = 0;
}

long L298n::map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}