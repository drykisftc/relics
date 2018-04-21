package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by robotgyms on 4/20/18.
 */

@Autonomous(name = "TestAutoSensors", group = "Test")

public class TestAutoSensors extends AutoHarvesterPlanARed {

    @Override
    public void start (){
        super.start ();
        navigation.pidControlDistance.setKp(-0.03f);//0.004
        navigation.pidControlDistance.setKi(-0.0001f);
        navigation.pidControlDistance.setKd(-0.00000001f);
        navigation.pidControlDistance.setMaxIntegralError(0.2f/navigation.pidControlDistance.fKi);
    }
    @Override
    public void loop () {
        switch (state) {
            case 0:
                if ( stopAtWallByRangeSensor(5, 1.5) <=0 ) {
                    state = 1;
                }
                break;
            case 1:

                break;
            default:

        }

        telemetry.addData("State: ", state);
        telemetry.update();
    }
}
