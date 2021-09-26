package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class SampleConfiguration implements Configuration {

    private List<LynxModule> allHubs;

    public DMotor backLeft, frontLeft, frontRight, backRight, slides;
    public DEncoderlessMotor ingest;
    //public DOdometryPod frontOdoPod, leftOdoPod, rightOdoPod;
    //public DServo servo;
    //public DMotor motor;
    //public Odometry odometry;
    public DAnalogSensor left, right, back1, back2;

    public DServo servo;

    public DDigitalSensor limit;

    public DIMU imu;

    public ValueStorage vals;

    public void Configure(HardwareMap hwMap){
        //Add all hardware devices here.
        //Example: hardware.put("motor1", new DriveObject(DriveObject.type.DcMotorImplEx, "left_back_motor", DriveObject.classification.Drivetrain, hwMap));
        //In this example, "left_back_motor" is whatever your configuration says.

        DAnalogSensor.InterpretVoltage distance = ((double voltage, double max) -> 87.4 * (voltage - 0.138));

        hardware.clear();
        frontLeft = new DMotor(hwMap, "front_left_motor");
        backLeft = new DMotor(hwMap, "back_left_motor");
        frontRight = new DMotor(hwMap, "front_right_motor");
        backRight = new DMotor(hwMap, "back_right_motor");
        ingest = new DEncoderlessMotor(hwMap, "ingest");
        limit = new DDigitalSensor(hwMap, "limit");
        slides = new DMotor(hwMap, "slides");
        servo = new DServo(hwMap, "servo");
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = new DIMU(hwMap);
        //left = new DAnalogSensor(hwMap, "left", distance);
        //right = new DAnalogSensor(hwMap, "right", distance);
        //back1 = new DAnalogSensor(hwMap, "back1", distance);
        //back2 = new DAnalogSensor(hwMap, "back2", distance);
        //imu = new DIMU(hwMap);

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
