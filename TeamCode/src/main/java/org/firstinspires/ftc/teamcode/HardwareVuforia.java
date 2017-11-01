package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This is NOT an OpMode
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot, using Matrix Hardware.
 * See PushbotTeleopTank_Iterative for a usage examples.
 *
 * This is coded as an Extension of HardwarePushbot to illustrate that the only additional
 * action REQUIRED for a MATRIX controller is enabling the Servos.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Matrix Controller has been assigned the name:  "matrix controller"
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Manipulator drive motor:  "arm motor"
 * Servo channel:  Servo to open left claw:  "left claw"
 * Servo channel:  Servo to open right claw: "right claw"
 *
 * In addition, the Matrix Controller has been assigned the name:  "matrix controller"
 */
public class HardwareVuforia extends HardwareBase
{
    //Vuforia
    VuforiaLocalizer vuforia;

    protected VuforiaLocalizer.CameraDirection cameraDirection;
    protected int cameraMonitorViewId;
    protected VuforiaLocalizer.Parameters parameters;
    protected VuforiaTrackables relicTrackables;
    protected VuforiaTrackable relicTemplate;
     RelicRecoveryVuMark vuMark;
    String vumarkImage = "unknown";

    /* Constructor */
    public HardwareVuforia(VuforiaLocalizer.CameraDirection cameraD){
        cameraDirection = cameraD;
    }

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);
        // camera
        cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVw9AA7/////AAAAGR2dOk5hfEdLl+V9Doao7C5Xp0Wvb2cien7ybTAhAyUTB2iZRO/CMyxlXakNnP3+HqLEMe7nzV+fllHLVQLuSwWmLdDErkjexTZKcgCGQUIZ+Ts6O2m7l+zwVVBH5V5Ah5SJP3jd/P6lvuKJY+DUY0pThAitsP59uD6wkcukMQQXNN+xBPzEBEx/0kt7hS5GJ+qCYDLD1qgCO5KrDuWzYtWjZi3LaGHsO9msvrGiCXYaP9PDRX9ZoWB1tJiHky5HyG/p+ndycmiK6sY9lRymaaJ5fX556ZUKtQX2dOAF7tHgVqsPOhqCV3E3qN6kXnwEqy9KgZ1QQjKJnCR5eLRXmSOqAbKi8ArzrRc3737EpSzK";
        parameters.cameraDirection = cameraDirection;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

    }

    @Override
    public void start() {
        relicTrackables.activate();
    }

    @Override
    public void stop() {

    }

    public void identifyGlyphCrypto (){
        vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            vumarkImage = "left";
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            vumarkImage = "right";
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            vumarkImage = "center";
        }
    }

    public OpenGLMatrix getGlyphCryptoPosition() {
        return ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
    }
}