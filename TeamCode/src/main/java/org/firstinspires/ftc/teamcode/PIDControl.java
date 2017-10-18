package org.firstinspires.ftc.teamcode;

/**
 * Created by hfu on 10/17/17.
 */

public class PIDControl {

    float fKp;
    float fKi;
    float fKd;

    float fError;
    float fIntegral;
    float fDerivative;
    float fTime;
    float fPrevError;
    float fMaxIntegralError;

    public PIDControl () {
        reset();
        fKp = 1.0f;
        fKi = 1.0f;
        fKd = 1.0f;
        fMaxIntegralError = Float.MAX_VALUE;
    }

    public void setKp(float p) {
        fKp = p;
    }

    public void setKi(float i) {
        fKi = i;
    }

    public void setKd(float d){
        fKd = d;
    }

    public void setMaxIntegralError(float maxV) {
        fMaxIntegralError = Math.abs(maxV);
    }

    public float update ( float e, float time){
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
