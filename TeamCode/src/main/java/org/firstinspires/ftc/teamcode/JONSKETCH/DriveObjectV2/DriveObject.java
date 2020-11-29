package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface DriveObject {

    int getPartNum();
    double[] get();
    double[] getHardware();
    void endThreads();

}
