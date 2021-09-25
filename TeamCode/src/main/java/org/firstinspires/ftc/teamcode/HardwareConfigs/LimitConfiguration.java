package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Configuration;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DAnalogSensor;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DDigitalSensor;

import java.util.List;


@Disabled
public class LimitConfiguration implements Configuration {

    private List<LynxModule> allHubs;

    private DAnalogSensor potentiometer;

    public void Configure(HardwareMap ahwMap) {

        System.out.println("Step 1");

        hardware.clear();

        allHubs = ahwMap.getAll(LynxModule.class);

        setBulkCachingManual(true);

        potentiometer = new DAnalogSensor(ahwMap, "potent", (double voltage, double maxVoltage) -> voltage * 81.45);

        System.out.println("Step 2");
    }

    public void setBulkCachingManual(boolean manual){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(manual ? LynxModule.BulkCachingMode.MANUAL : LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            if(module.getBulkCachingMode() == LynxModule.BulkCachingMode.MANUAL) {
                System.out.println("Clearing");
                module.clearBulkCache();
                //module.getBulkData();
            }
        }
    }
}
