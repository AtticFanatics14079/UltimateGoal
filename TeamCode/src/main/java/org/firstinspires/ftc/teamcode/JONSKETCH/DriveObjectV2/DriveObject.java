package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public interface DriveObject extends SharedObjects {

    int getPartNum();
    double[] get();
    double[] getHardware();
    void endThreads();
}
