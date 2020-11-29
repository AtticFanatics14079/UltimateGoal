package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class DriveObject {

    static ValueStorage vals;
    int partNum;

    public DriveObject(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum) {};
    void set(double value) {

    };
    int getPartNum();
    double[] get();
    void setHardware(double value);
    double[] getHardware();
    void endThreads();

}
