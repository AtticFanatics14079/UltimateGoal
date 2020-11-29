package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface DriveObject {

    int getPartNum();
    double[] get();
    void setHardware(double value);
    double[] getHardware();
    void endThreads();

}
