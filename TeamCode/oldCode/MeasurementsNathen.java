/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareVortex class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp: Nathen Measurement", group="Utilities")
@Disabled
public class MeasurementsNathen extends OpMode{

    /* Declare OpMode members. */
    protected HardwareNathen robot = new HardwareNathen();

    double [] wheelPowerLUT = {0.0f, 0.05f, 0.15f, 0.18f, 0.20f,
            0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
            0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
            0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f,
            1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f,
            1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f};

    DcMotor [] leftMotors;
    DcMotor [] rightMotors;

    Navigation navigation = null;

    int navigationState = 0;
    double kp = 1.0;
    double ki = 1.0;
    double kd = 1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.start();
        robot.gyro.calibrate();

        leftMotors = new DcMotor[1];
        leftMotors[0] = robot.motorLeftWheel;
        rightMotors = new DcMotor[1];
        rightMotors[0] = robot.motorRightWheel;

        navigation = new Navigation(telemetry);

        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (robot.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro is calibrating.  DO NOT start!!!!");
            telemetry.addData(">", "Wait! Wait! Wait! ");
        }
        else {
            telemetry.addData(">", "Press Start.");
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.jewelSensor.enableLed(true);
        kp = navigation.pidControlHeading.fKp;
        ki = navigation.pidControlHeading.fKi;
        kd = navigation.pidControlHeading.fKd;
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        measurements();
        testGyroTurn();
        adjustPID();
        telemetry.update();
    }

    public void measurements() {

        //jewel color detection
        int blue = robot.jewelSensor.blue();
        int red = robot.jewelSensor.red();
        int green = robot.jewelSensor.green();
        telemetry.addData("red   ", red);
        telemetry.addData("blue  ", blue);
        telemetry.addData("green ", green);


        if ( blue> red && blue > green) {
            telemetry.addData("jewelColor", "blue");
        } else if (red > blue && red > green) {
            telemetry.addData("jewelColor", "red");
        } else {
            telemetry.addData("jewelColor", "unknown");
        }

        //motors
        telemetry.addData("left wheel : ", robot.motorLeftWheel.getCurrentPosition());
        telemetry.addData("right wheel: ", robot.motorRightWheel.getCurrentPosition());

        telemetry.addData("lift wheel : ", robot.liftMotor.getCurrentPosition());

        telemetry.addData("Heading    : ", robot.gyro.getHeading());

        telemetry.addData("Navigation state    : ", navigationState);

        telemetry.addData("KP :", kp);
        telemetry.addData("KI :", ki);
        telemetry.addData("KD :", kd);
    }

    public void adjustPID () {
        if (Math.abs(gamepad1.right_stick_y) > 0.02) {
            kp += gamepad1.right_stick_y/100000;
            navigation.pidControlHeading.setKp(kp);
        }
        if (Math.abs(gamepad1.left_stick_y) > 0.02) {
            ki += gamepad1.left_stick_y/10000;
            navigation.pidControlHeading.setKi(ki);
        }
    }

    public void testGyroTurn () {
        if (gamepad1.y) {
            navigationState = 1;
        } else if (gamepad1.x) {
            navigationState = 2;
        } else if (gamepad1.b) {
            navigationState = 3;
        } else if (gamepad1.a) {
            navigationState = 4;
        }

        switch (navigationState) {
            case 1:
                if (0==navigation.turnByGyroCloseLoop(0.0, robot.gyro.getHeading(),
                    navigation.normalizeHeading(0), leftMotors, rightMotors)){
                    navigationState = 0;
                }
                break;
            case 2 :
                if (0==navigation.turnByGyroCloseLoop(0.0, robot.gyro.getHeading(),
                    navigation.normalizeHeading(90), leftMotors, rightMotors)){
                    navigationState = 0;
                }
                break;
            case 3:
               if (0==navigation.turnByGyroCloseLoop(0.0, robot.gyro.getHeading(),
                    navigation.normalizeHeading(-90), leftMotors, rightMotors)){
                    navigationState = 0;
                }
                break;
            case 4:
                if (0==navigation.turnByGyroCloseLoop(0.0, robot.gyro.getHeading(),
                    navigation.normalizeHeading(180), leftMotors, rightMotors)){
                    navigationState = 0;
                }
                break;
            default:
                navigation.resetTurn(leftMotors, rightMotors);
                break;

        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

}
