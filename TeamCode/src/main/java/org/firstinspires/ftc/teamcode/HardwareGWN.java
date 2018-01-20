package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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

public class HardwareGWN extends HardwareBase
{
    // DC Motors
    public DcMotor motorLeftBackWheel = null;
    public DcMotor motorRightBackWheel =null;
    public DcMotor motorLeftFrontWheel = null;
    public DcMotor motorRightFrontWheel =null;
    public DcMotor liftMotor = null;

    public DcMotor leftTrack = null;
    public DcMotor rightTrack= null;

    //servos
    public Servo jewelArm = null;
    public Servo jewelHitter = null;

    public Servo leftFlipper = null;
    public Servo rightFlipper = null;

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

    double defaultGlyphWheelPower = 1.0;
    double defaultGlyphLiftPower = 0.9;

    double flipperDepositePosition = 0.0;
    double flipperLoadPosition = 0.7;

    /* Constructor */
    public HardwareGWN(){

    }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);

        motorLeftBackWheel = hwMap.dcMotor.get("leftBackWheel");
        motorRightBackWheel = hwMap.dcMotor.get("rightBackWheel");
        motorLeftBackWheel.setDirection(DcMotor.Direction.REVERSE);  // 40 to 1 andymark motor
        motorRightBackWheel.setDirection(DcMotor.Direction.FORWARD); // 40 to 1 andymark motor
        motorLeftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftFrontWheel = hwMap.dcMotor.get("leftFrontWheel");
        motorRightFrontWheel = hwMap.dcMotor.get("rightFrontWheel");
        motorLeftFrontWheel.setDirection(DcMotor.Direction.REVERSE);  // 40 to 1 andymark motor
        motorRightFrontWheel.setDirection(DcMotor.Direction.FORWARD); // 40 to 1 andymark motor
        motorLeftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor = hwMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jewelHitter = hwMap.servo.get("jewelHitter");
        jewelArm = hwMap.servo.get("jewelArm");

        jewelSensor = hwMap.get(ColorSensor.class, "jewelSensor");
        jewelSensorDistance = hwMap.get(DistanceSensor.class, "jewelSensor");

        leftTrack = hwMap.dcMotor.get("leftTrack");
        leftTrack.setDirection(DcMotor.Direction.FORWARD);
        leftTrack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTrack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightTrack = hwMap.dcMotor.get("rightTrack");
        rightTrack.setDirection(DcMotor.Direction.FORWARD);
        rightTrack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTrack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFlipper = hwMap.servo.get("leftFlipper");
        rightFlipper = hwMap.servo.get("rightFlipper");

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
        motorLeftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBackWheel.setPower(0.0);
        motorRightBackWheel.setPower(0.0);

        motorLeftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFrontWheel.setPower(0.0);
        motorRightFrontWheel.setPower(0.0);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(0.0);

        leftTrack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTrack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTrack.setPower(0.0);

        rightTrack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTrack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTrack.setPower(0.0);
        //jewelArm.setPosition(0.8);
    }

    @Override
    public void stop() {

        motorLeftBackWheel.setPower(0.0);
        motorRightBackWheel.setPower(0.0);
        motorLeftFrontWheel.setPower(0.0);
        motorRightFrontWheel.setPower(0.0);

        liftMotor.setPower(0.0);

        leftTrack.setPower(0.0);
        rightTrack.setPower(0.0);

        motorLeftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTrack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTrack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    }*/

    void glyphWheelLoad(){
        leftTrack.setPower(-defaultGlyphWheelPower);
        rightTrack.setPower(defaultGlyphWheelPower);
        setFlipperPositions(flipperLoadPosition);
    }

    void initAllDevices() {
        jewelArm.setPosition(0.55);
        jewelHitter.setPosition(1.0);
    }

    void setFlipperPositions(Double position) {
        leftFlipper.setPosition(1 - position);
        rightFlipper.setPosition(position);
    }

}