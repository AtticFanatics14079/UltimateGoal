package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfigs.OneHubDrive;

import java.util.Arrays;

@TeleOp
public class SampleTeleOp extends LinearOpMode {

    HardwareThread hardware;
    OneHubDrive config;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            config = new OneHubDrive();
            ValueStorage vals = new ValueStorage();
            hardware = new HardwareThread(hardwareMap, vals, config);
            //hardware.config.ExtendGripper.setPID(2, 0, 0); //Gonna need to mess with this one
            waitForStart();
            ElapsedTime time = new ElapsedTime();
            hardware.start();
            while(!isStopRequested()){
                vals.waitForCycle();
                System.out.println("Finshed waiting, " + time.milliseconds());
                getInput();
            }
        } catch(Exception e) {
            System.out.println("Exception: " + e);
        } finally {
            hardware.Stop();
        }
    }

    private void getInput(){
        setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

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