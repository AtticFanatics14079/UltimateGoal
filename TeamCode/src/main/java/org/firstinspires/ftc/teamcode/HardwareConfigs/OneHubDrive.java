package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Configuration;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DMotor;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DOdometryPod;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DServo;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DThreeWheelOdo;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Odometry;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ValueStorage;

import java.util.List;

public class OneHubDrive implements Configuration {

    private List<LynxModule> allHubs;

    public DMotor backLeft, frontLeft, frontRight, backRight;
    public DOdometryPod frontOdoPod, leftOdoPod, rightOdoPod;
    public DServo servo;
    public DMotor shooter;
    public Odometry odometry;

    public void Configure(HardwareMap hwMap, ValueStorage vals){
        //Add all hardware devices here.
        //Example: hardware.put("motor1", new DriveObject(DriveObject.type.DcMotorImplEx, "left_back_motor", DriveObject.classification.Drivetrain, hwMap));
        //In this example, "left_back_motor" is whatever your configuration says.
        int i = 0;
        hardware.clear();
        backLeft = new DMotor(hwMap, "back_left_motor");
        frontLeft = new DMotor(hwMap, "front_left_motor");
        frontRight = new DMotor(hwMap, "front_right_motor");
        backRight = new DMotor(hwMap, "back_right_motor");

        hardware.add(backLeft);
        hardware.add(frontLeft);
        hardware.add(frontRight);
        hardware.add(backRight);
        frontRight.reverse(true);
        backRight.reverse(true);
        //Adding more later

        //Below are other configuration activities that are necessary for writing to file.
        allHubs = hwMap.getAll(LynxModule.class);

        setBulkCachingManual(true);
    }

    public void setBulkCachingManual(boolean manual){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(manual ? LynxModule.BulkCachingMode.MANUAL : LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            if(module.getBulkCachingMode() == LynxModule.BulkCachingMode.MANUAL) {
                module.clearBulkCache();
                module.getBulkData();
            }
        }
    }
}
