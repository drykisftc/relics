package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareBase
{

    HardwareMap hwMap           =  null;
    Navigation navigation = null;

    /* Constructor */
    public HardwareBase(){
        navigation = new Navigation();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
    }

    public void reset () {

    }
}
