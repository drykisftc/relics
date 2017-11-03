package org.firstinspires.ftc.teamcode;

/**
 * Created by hfu on 10/17/17.
 */

public class PIDControl {

    double fKp;
    double fKi;
    double fKd;

    double fError;
    double fIntegral;
    double fDerivative;
    double fTime;
    double fPrevError;
    double fMaxIntegralError;

    public PIDControl () {
        reset();
        fKp = 1.0f;
        fKi = 1.0f;
        fKd = 1.0f;
        fMaxIntegralError = Double.MAX_VALUE;
    }

    public void setKp(double p) {
        fKp = p;
    }

    public void setKi(double i) {
        fKi = i;
    }

    public void setKd(double d){
        fKd = d;
    }

    public void setMaxIntegralError(double maxV) {
        fMaxIntegralError = Math.abs(maxV);
    }

    public double update ( double e, double time){
        if (time != 0.0) {
            fIntegral += e*time;
            // cap the max error
            if (fIntegral >0) {
                fIntegral = Math.min(fMaxIntegralError, fIntegral);
            } else {
                fIntegral = Math.max(-fMaxIntegralError, fIntegral);
            }
            fDerivative = (e-fError)/time;
            fError = e;
            return fKp*e+fKi*fIntegral+fKd*fDerivative;
        }
        return 0.0f;
    }

    public void reset (){
        fError = 0.0f;;
        fIntegral = 0.0f;
    }
}
