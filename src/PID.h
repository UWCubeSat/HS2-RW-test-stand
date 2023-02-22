#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
/* A standard PID controller class */
class PIDController {
private:
  double kp, ki, kd, lastErr, cumulErr;

protected:
  virtual double calcError(double setPoint, double current) {
    return setPoint - current;
  }
  
public:
  PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), lastErr(0), cumulErr(0) {}
  
  double compute(double setPoint, double current, long deltaT) {
    /*Compute all the working error variables*/
    double error = calcError(setPoint, current);
    double dErr = (deltaT==0) ? 0 : calcError(error, lastErr) / deltaT;

    /*Remember some variables for next time*/
    lastErr = error;
    cumulErr += error * deltaT;

    // we don't want derivative to cause the thing to ever go backwards
    double out = kp * error + ki * cumulErr + kd * dErr;
    // condition is sus
    out = (dErr > 0) ? max(0, out) : min(0, out);
    return out;
  }
  
  void reset() {
    lastErr=0;
    cumulErr=0;
  }

  double getError(){
    return lastErr;
  }
};

/* A PID controller to control angles*/
class PIDAngleController: public PIDController{
  
public:
  PIDAngleController(double kp, double ki, double kd) : PIDController(kp, ki, kd) {}

protected:
  virtual double calcError(double setPoint, double current){
    double distance = fmod(setPoint - current , 360.0);
    if (distance < -180)
        return distance + 360;
    else if (distance > 179)
        return distance - 360;
    return distance;
    // return setPoint - current;
  }
};
#endif