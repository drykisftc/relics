package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 24 inches have 1939 encoder counts that is 80.79encoder counts per inch
 Nathan has wheel base of 14.4 inches.
 */

public class HardwareGMNPrototype extends HardwareBase
{
    // DC Motors
    public DcMotor leftWheel = null;
    public DcMotor rightWheel =null;
    //public DcMotor liftMotor = null;

    public DcMotor leftBelt = null;
    public DcMotor rightBelt = null;

    //servos
//    public Servo jewelArm = null;
//    public Servo jewelHitter = null;

    //sensors
    public ColorSensor jewelSensor = null;
    public DistanceSensor jewelSensorDistance = null;

    // Orientation sensor
    BNO055IMU imu = null;
    Orientation angles = null;

    protected float axleDistance = 2200; //80.79 * 14;

    double pusherLoadPosition = 0.0;
    double pusherActPosition = 0.6;

    double blockerUnloadPosition = 0.00;
    double blockerLoadPosition = 0.40;

    double defaultGlyphWheelPower = 0.3;
    double defaultGlyphLiftPower = 0.8;

    /* Constructor */
    public HardwareGMNPrototype(){

    }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        leftWheel = hwMap.dcMotor.get("leftBackWheel");
        rightWheel = hwMap.dcMotor.get("rightBackWheel");
        leftWheel.setDirection(DcMotor.Direction.REVERSE);  // rev motor
        rightWheel.setDirection(DcMotor.Direction.FORWARD); // rev motor
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        liftMotor = hwMap.dcMotor.get("liftMotor");
//        liftMotor.setDirection(DcMotor.Direction.FORWARD);
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        jewelHitter = hwMap.servo.get("jewelHitter");
//        jewelArm = hwMap.servo.get("jewelArm");

//        jewelSensor = hwMap.get(ColorSensor.class, "jewelSensor");
//        jewelSensorDistance = hwMap.get(DistanceSensor.class, "jewelSensor");

        leftBelt = hwMap.dcMotor.get("leftBelt");
        leftBelt.setDirection(DcMotor.Direction.FORWARD);
        leftBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBelt = hwMap.dcMotor.get("rightBelt");
        rightBelt.setDirection(DcMotor.Direction.FORWARD);
        rightBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void start () {
        // wheels
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);

//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        liftMotor.setPower(0.0);

        leftBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBelt.setPower(0.0);

        rightBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBelt.setPower(0.0);

        //jewelArm.setPosition(0.8);
    }

    @Override
    public void stop() {

        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);

//        liftMotor.setPower(0.0);

        leftBelt.setPower(0.0);
        rightBelt.setPower(0.0);

        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /*void beltDepositGlyph() {
        lowerBeltServo1.setPower(1.0);
        lowerBeltServo2.setPower(1.0);
    }

    void beltSpitOutGlyph() {
        lowerBeltServo1.setPower(-1.0);
        lowerBeltServo2.setPower(-1.0);
    }

    void beltStop() {
        lowerBeltServo1.setPower(0);
        lowerBeltServo2.setPower(0);
    }

    void glyphWheelLoad(){
        leftLiftWheel.setPower(-defaultGlyphWheelPower);
        rightLiftWheel.setPower(defaultGlyphWheelPower);
    }

    void glyphWheelUnload() {
        leftLiftWheel.setPower(defaultGlyphWheelPower);
        rightLiftWheel.setPower(-defaultGlyphWheelPower);
        glyphPusher.setPosition(pusherLoadPosition);
    }

    void initAllDevices() {
        jewelArm.setPosition(0.55);
        jewelHitter.setPosition(1.0);
        glyphPusher.setPosition(0.05);
        smolL.setPosition(blockerLoadPosition);
    }*/

}
