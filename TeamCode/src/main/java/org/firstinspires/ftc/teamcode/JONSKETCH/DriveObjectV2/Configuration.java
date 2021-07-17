package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public interface Configuration extends SharedObjects {

    void Configure(HardwareMap hwMap);
    void setBulkCachingManual(boolean manual);
    void clearBulkCache();
}
