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

@TeleOp(name="TeleOp: Measurement Harvester", group="Measurement")
public class MeasurementHarvester extends OpMode{

    /* Declare OpMode members. */
    protected HardwareHarvester robot = new HardwareHarvester();

    protected int liftHeightLimit = 3200;
    protected int liftMotorPosition = 0;
    protected double liftMotorHolderPower = 0.3;

    private int state = 0;
    private boolean preBack = false;

    double [] wheelPowerLUT = {0.0f, 0.05f, 0.15f, 0.18f, 0.20f,
            0.22f, 0.24f, 0.26f, 0.28f, 0.30f, 0.32f, 0.34f, 0.36f,
            0.38f, 0.42f, 0.46f, 0.50f, 0.54f, 0.58f, 0.62f, 0.66f,
            0.70f, 0.74f, 0.78f, 0.82f, 0.86f, 0.90f, 0.94f, 0.98f,
            1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f,
            1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f};

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("TeleOp", "Hello Relic");    //
        updateTelemetry(telemetry);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.start();
        robot.initAllDevices();
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (state) {
            case 0:
                telemetry.addData("Mode: ", "Normal");

                glyphSensorReadValue();
                joystickWheelControl();
                glyphWheelControl();
                glyphDepositControl();
                glyphLiftControl();
                jewelArmControl();
                jewelSensorReadValue();
                imuReadings();
                break;
            case 1:
                telemetry.addData("Mode: ", "SecondControlSet");

                RKArmControl();
                break;
            default:
                state = 0;
                break;
        }

        if ((gamepad1.back || gamepad2.back) && !preBack) {
            state = ((state == 0)? 1:0);
        }

        preBack = (gamepad1.back || gamepad2.back);
        telemetry.update();
    }

    public void joystickWheelControl() {

        // Mecanum wheel driving system (note: The joystick goes negative when pushed forwards, so negate it)
        float throttle = -gamepad1.right_stick_y;
        float direction = gamepad1.right_stick_x;
        float parallel = -gamepad1.left_stick_x;
        double diagonal = gamepad1.left_stick_y;
        float right = throttle - direction;
        float left = throttle + direction;
        double diagonal1 = parallel + diagonal;
        double diagonal2 = -parallel + diagonal;

        if (Math.abs(parallel) > 0.05 || Math.abs(diagonal) > 0.05) {

            //parallel and diagonal movement
            diagonal1 = Range.clip(diagonal1, -1, 1);
            diagonal2 = Range.clip(diagonal2, -1, 1);
            robot.motorLeftBackWheel.setPower(-diagonal2*Math.abs(diagonal2));
            robot.motorLeftFrontWheel.setPower(-diagonal1*Math.abs(diagonal1));
            robot.motorRightBackWheel.setPower(-diagonal1*Math.abs(diagonal1));
            robot.motorRightFrontWheel.setPower(-diagonal2*Math.abs(diagonal2));

        } else {

            // clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);
            robot.motorLeftBackWheel.setPower(left);
            robot.motorLeftFrontWheel.setPower(left);
            robot.motorRightBackWheel.setPower(right);
            robot.motorRightFrontWheel.setPower(right);

        }

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }

    public void glyphWheelControl() {

        double lw = gamepad2.left_stick_y;
        double rw = gamepad2.right_stick_y;

        if (gamepad2.right_bumper || gamepad1.right_bumper) {
            robot.glyphWheelLoad();
        } else if (gamepad2.left_bumper || gamepad1.left_bumper) {
            robot.glyphWheelUnload();
        } else if (Math.abs(lw) > 0.05 || Math.abs(rw) > 0.05) {
            robot.leftLiftWheel.setPower(lw * -0.25);
            robot.rightLiftWheel.setPower(rw * 0.25);
        } else {
            robot.stopGlyphWheels();
        }
    }

    public void glyphDepositControl() {

        if (gamepad1.a || gamepad2.a) {
            robot.loadGlyph();
        }

        if (gamepad1.b || gamepad2.b ) {
            robot.levelGlyph();
        }

        if (gamepad1.y || gamepad2.y) {
            robot.dumpGlyph();
        }
    }

    public void glyphLiftControl () {

        liftMotorPosition = robot.liftMotor.getCurrentPosition();
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            if (liftMotorPosition < liftHeightLimit) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(robot.defaultGlyphLiftPower);
            } else {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(0);
            }
            robot.levelGlyph();
        } else if (gamepad1.dpad_down || gamepad2.dpad_down ) {
            if (liftMotorPosition > 0) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(-robot.defaultGlyphLiftPower);
            } else {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(0);
            }
        } else {
            // hold position
            VortexUtils.moveMotorByEncoder(robot.liftMotor, liftMotorPosition, liftMotorHolderPower);
        }

        telemetry.addData("lift arm pos ", "%6d", liftMotorPosition);
    }

    public void jewelArmControl() {

        if (gamepad1.dpad_left || gamepad2.dpad_left) {

            robot.jewelArm.setPosition(robot.jewelArm.getPosition() + 0.001);

        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {

            robot.jewelArm.setPosition(robot.jewelArm.getPosition() - 0.001);

        }

        telemetry.addData("jewel arm position ", robot.jewelArm.getPosition());
        telemetry.addData("jewel hitter position ", robot.jewelHitter.getPosition());
    }

    public void jewelSensorReadValue() {

        telemetry.addData("jewel sensor red     : ", robot.jewelSensor.red());
        telemetry.addData("jewel sensor blue    : ", robot.jewelSensor.blue());
        telemetry.addData("jewel sensor green   : ", robot.jewelSensor.green());
        telemetry.addData("jewel sensor alpha   : ", robot.jewelSensor.alpha());
        telemetry.addData("jewel sensor argb    : ", robot.jewelSensor.argb());

        telemetry.addData("jewel sensor distance: ", robot.jewelSensorDistance.getDistance(DistanceUnit.CM));

    }

    public void imuReadings() {

        telemetry.addData("First angle", robot.imu.getAngularOrientation().firstAngle);
        telemetry.addData("Second angle", robot.imu.getAngularOrientation().secondAngle);
        telemetry.addData("Third angle", robot.imu.getAngularOrientation().thirdAngle);

    }

    public void glyphSensorReadValue() {
        telemetry.addData("glyph sensor distance: ", robot.glyphDistance.getDistance(DistanceUnit.CM));
    }

    public void RKArmControl() {
        if (gamepad2.a) {
            robot.RKArm.setPosition(robot.RKArmExtendPosition); // extend
        } else if (gamepad1.b) {
            robot.RKArm.setPosition(robot.RKArmRetractPosition); // retract
        }

        telemetry.addData("RKSensorDistance", robot.RKSensor.getDistance(DistanceUnit.CM));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

}
