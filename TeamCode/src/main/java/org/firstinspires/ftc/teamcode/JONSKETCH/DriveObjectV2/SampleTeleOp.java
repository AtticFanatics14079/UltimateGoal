package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@TeleOp
public class SampleTeleOp extends LinearOpMode {

    HardwareThread hardware;
    SampleConfiguration config;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            config = new SampleConfiguration();
            ValueStorage vals = new ValueStorage();
            hardware = new HardwareThread(hardwareMap, vals, config);
            //hardware.config.ExtendGripper.setPID(2, 0, 0); //Gonna need to mess with this one
            waitForStart();
            ElapsedTime time = new ElapsedTime();
            hardware.start();
            //config.odometry.beginTracking();
            while(!isStopRequested()){
                vals.waitForCycle();
                System.out.println("Finshed waiting, " + time.milliseconds());
                getInput();
            }
        } catch(Exception e) {
            System.out.println("Exception: " + e);
        } finally {
            config.odometry.endTracking();
            hardware.Stop();
        }
    }

    private void getInput(){
        setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //config.motor.setPower(1);

        if(gamepad1.a) config.servo.set(1);
        else if(gamepad1.b) config.servo.set(0);

        if(gamepad1.x) config.motor.setPosition(1000, 1, 50);
        else if(gamepad1.y) config.motor.setPosition(0, 1, 50);

        //if(gamepad1.start) {
        //}

        telemetry.addData("Values: ", Arrays.toString(config.backLeft.get()));

        System.out.println("After in runValues in op mode");

        telemetry.update();
    }

    private void setPower(double px, double py, double pa){
        double p1 = -px + py + pa;
        double p2 = px + py + pa;
        double p3 = -px + py - pa;
        double p4 = px + py - pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        config.backLeft.setPower(p1);
        config.frontLeft.setPower(p2);
        config.frontRight.setPower(p3);
        config.backRight.setPower(p4);
    }
}