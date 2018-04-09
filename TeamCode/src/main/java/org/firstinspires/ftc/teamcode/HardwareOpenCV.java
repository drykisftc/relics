package org.firstinspires.ftc.teamcode;

/**
 * Created by robotgyms on 3/26/18.
 * The below steps for using Android OpenCV sdk in Android Studio. This is a simplified version of this(1) SO answer.

 Download latest OpenCV sdk for Android from OpenCV.org and decompress the zip file.
 Import OpenCV to Android Studio, From File -> New -> Import Module, choose sdk/java folder in the unzipped opencv archive.
 Update build.gradle under imported OpenCV module to update 4 fields to match your project build.gradle a) compileSdkVersion b) buildToolsVersion c) minSdkVersion and d) targetSdkVersion.
 Add module dependency by Application -> Module Settings, and select the Dependencies tab. Click + icon at bottom, choose Module Dependency and select the imported OpenCV module.
 For Android Studio v1.2.2, to access to Module Settings : in the project view, right-click the dependent module -> Open Module Settings
 Copy libs folder under sdk/native to Android Studio under app/src/main.
 In Android Studio, rename the copied libs directory to jniLibs and we are done.
 Step (6) is since Android studio expects native libs in app/src/main/jniLibs instead of older libs folder. For those new to Android OpenCV, don't miss below steps

 include static{ System.loadLibrary("opencv_java"); } (Note: for OpenCV version 3 at this step you should instead load the library opencv_java3.)
 For step(5), if you ignore any platform libs like x86, make sure your device/emulator is not on that platform.
 */


public class HardwareOpenCV extends HardwareBase {
    public HardwareOpenCV() {
        super();
    }

    /*
    VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
    long numImages = frame.getNumImages();

    for (int i = 0; i < numImages; i++) {
        if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
            rgb = frame.getImage(i);
            break;
        }
    }

    // rgb is now the Image object that weve used in the video
    Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
    bm.copyPixelsFromBuffer(rgb.getPixels());

    //put the image into a MAT for OpenCV
    Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
    Utils.bitmapToMat(bm, tmp);

    //close the frame, prevents memory leaks and crashing
    frame.close();
   */
}
