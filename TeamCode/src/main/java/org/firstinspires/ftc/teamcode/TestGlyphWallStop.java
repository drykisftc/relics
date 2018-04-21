package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by robotgyms on 4/20/18.
 */

@Autonomous(name = "TestGlyphWallStop", group = "Test")

public class TestGlyphWallStop extends AutoHarvesterPlanARed {

    double baselineDis [] = new double[10];
    int baselineIndex = 0;
    double baselineAvg = 0;

    @Override
    public void start (){
        super.start ();
        navigation.pidControlDistance.setKp(-0.03f);//0.004
        navigation.pidControlDistance.setKi(-0.0001f);
        navigation.pidControlDistance.setKd(-0.00000001f);
        navigation.pidControlDistance.setMaxIntegralError(0.2f/navigation.pidControlDistance.fKi);
        baselineIndex = 0;
    }

    @Override
    public void loop () {
        switch (state) {
            case 0:
                // collect some distance
                baselineDis[baselineIndex++] = robot.backDistanceSensor.getDistance(DistanceUnit.INCH);
                // if done, compute average, then go to next state
                if (baselineIndex >= 10) {
                    baselineIndex = 0;
                    baselineAvg = 0;
                    for (int i =0; i < baselineDis.length; i++) {
                        baselineAvg += baselineDis[i];
                    }
                    baselineAvg = baselineAvg / baselineDis.length;
                    state = 1;
                }

                break;
            case 1:
                // start side move
                if (0 != sideMoveByRangeSensor(baselineAvg, 2) ) {
                    moveAtPower(0.0);
                    state = 2;
                }
                // if distance changed by some % , stop

                break;
            default:

        }

        telemetry.addData("State: ", state);
        telemetry.addData("baselineAvg: ", baselineAvg);
        telemetry.update();
    }
}
