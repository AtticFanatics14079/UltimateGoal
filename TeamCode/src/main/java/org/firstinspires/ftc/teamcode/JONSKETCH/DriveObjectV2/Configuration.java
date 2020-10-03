package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

interface Configuration {

    ArrayList<DriveObject> hardware = new ArrayList<>();

    void Configure(HardwareMap hwMap, ValueStorage vals);
    void setBulkCachingManual(boolean manual);
    void clearBulkCache();
}
