package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfigs.CVConfig;
import org.firstinspires.ftc.teamcode.HardwareConfigs.LimitConfiguration;

import static org.firstinspires.ftc.teamcode.TeleOp.UltimateGoalTeleOpV1.sensorSideOffset;
import static org.firstinspires.ftc.teamcode.TeleOp.UltimateGoalTeleOpV1.sensorStrightOffset;

@TeleOp
public class SampleTeleOp extends LinearOpMode {

    HardwareThread hardware;
    //SampleConfiguration config;
    LimitConfiguration config;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            config = new LimitConfiguration();
            hardware = new HardwareThread(hardwareMap, config);
            //config.imu.gettingInput = true;
            hardware.start();
            System.out.println("Step 3");
            /*Sequence seq = new Sequence(() -> {
                while(Math.abs(config.backLeft.get()[1]) < 1000) {
                    setPower(0, 0, 0.5);
                    HardwareThread.waitForCycle();
                }
                setPower(0, 0, 0);
            }, () -> {
                while(Math.abs(config.backLeft.get()[1]) < 200) {
                    HardwareThread.waitForCycle();
                };
            }, null);
            Sequence seq2 = new Sequence(() -> {
                while(Math.abs(config.backLeft.get()[1]) < 1000) {
                    HardwareThread.waitForCycle();
                    System.out.println("Current position: " + config.backLeft.get()[1]);
                }

            }, seq);
            System.out.println("Here");

             */
            waitForStart();
            System.out.println("There");
            ElapsedTime time = new ElapsedTime();
            //Thread t = new Thread(seq2);
            //t.start();
            while(!isStopRequested()){
                hardware.waitForCycle();
                getInput();
            }
        } catch(Exception e) {
            System.out.println("Exception: " + e);
        } finally {
            hardware.Stop();
        }
    }

    private void getInput(){
        //setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        //double imuHeading = config.imu.get()[0];
        //double imuDegrees = Math.toDegrees(Math.abs(imuHeading));
        //imuDegrees = imuDegrees > 90 ? 180 - imuDegrees : imuDegrees;
        //imuDegrees = Math.abs(imuDegrees - 90) < 25 ? Math.abs(imuDegrees - 90) : imuDegrees;
        //double hypotenuse = config.back2.get()[0] * (1 - 0.00000706 * imuDegrees + 0.000114 * Math.pow(imuDegrees, 2));
        //telemetry.addData("Hypot: ", hypotenuse);
        //telemetry.addData("Wall: ", hypotenuse * Math.cos(Math.toRadians(imuDegrees)));
        //telemetry.addData("IMU: ", imuDegrees);
        System.out.println("Exited input loop.");
        telemetry.addData("Degree: ", hardware.hardware.get(0).get()[0]);
        telemetry.update();
    }

    /*private void setPower(double px, double py, double pa){
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

     */
}