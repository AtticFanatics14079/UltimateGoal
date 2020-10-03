package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

public interface DriveObject {

    void set(double value);
    int getPartNum();
    double[] get();
    void setHardware(double value);
    double[] getHardware();
    void endThreads();

}
