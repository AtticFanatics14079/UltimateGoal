package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class SampleConfiguration implements Configuration {

    private List<LynxModule> allHubs;

    public DMotor backLeft, frontLeft, frontRight, backRight;
    public DOdometryPod frontOdoPod, leftOdoPod, rightOdoPod;
    public DServo servo;
    public DMotor motor;
    public Odometry odometry;

    public void Configure(HardwareMap hwMap, ValueStorage vals){
        //Add all hardware devices here.
        //Example: hardware.put("motor1", new DriveObject(DriveObject.type.DcMotorImplEx, "left_back_motor", DriveObject.classification.Drivetrain, hwMap));
        //In this example, "left_back_motor" is whatever your configuration says.
        int i = 0;
        hardware.clear();
        backLeft = new DMotor(vals, hwMap, "back_left_motor", i++);
        frontLeft = new DMotor(vals, hwMap, "front_left_motor", i++);
        frontRight = new DMotor(vals, hwMap, "front_right_motor", i++);
        backRight = new DMotor(vals, hwMap, "back_right_motor", i++);
        frontOdoPod = new DOdometryPod(vals, hwMap, "frontEncoder", i++);
        leftOdoPod = new DOdometryPod(vals, hwMap, "leftEncoder", i++);
        rightOdoPod = new DOdometryPod(vals, hwMap, "rightEncoder", i++);
        servo = new DServo(vals, hwMap, "servo", i++);
        motor = new DMotor(vals, hwMap, "motor", i++);
        odometry = new DThreeWheelOdo(0, 0, 0, vals, new DOdometryPod[]{leftOdoPod, rightOdoPod, frontOdoPod}, 1/274.29, 9.92, 0, 13.5);

        hardware.add(backLeft);
        hardware.add(frontLeft);
        hardware.add(frontRight);
        hardware.add(backRight);
        hardware.add(frontOdoPod);
        hardware.add(leftOdoPod);
        hardware.add(rightOdoPod);
        hardware.add(servo);
        hardware.add(motor);
        hardware.add(odometry);
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
