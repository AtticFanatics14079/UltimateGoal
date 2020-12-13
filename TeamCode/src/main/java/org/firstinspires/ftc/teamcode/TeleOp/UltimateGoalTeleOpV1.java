package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.HardwareConfigs.UltimateGoalConfig;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Configuration;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ConfigurationRR;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Sequence;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ValueStorage;

@TeleOp
public class UltimateGoalTeleOpV1 extends LinearOpMode {

    ConfigurationRR config;
    ValueStorage vals = new ValueStorage();
    HardwareThread hardware;
    Thread returnToShoot, powerShots;
    public static double wobbleUp = 0.22, wobbleDown = 0.65, wobbleMid = 0.45, gripperOpen = 0, gripperClosed = 1, load = 0.6, reload = 0.15;

    private Pose2d highGoalShoot = new Pose2d(-16.0, -32.0, 0);
    private Pose2d powerShotShoot = new Pose2d(-24.0, -18.0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            config = new ConfigurationRR(hardwareMap);
            config.Configure(hardwareMap, vals);
            hardware = new HardwareThread(hardwareMap, vals, config);
            configureMacros();
            waitForStart();
            hardware.start();
            if(!isStopRequested()) {
                while(opModeIsActive()){
                    vals.waitForCycle();
                    getInput();
                }
            }
        } catch(Exception e){

        } finally{
            hardware.Stop();
        }

    }

    public void getInput(){
        //Main loop
        config.shooter.set(1600);
        if(gamepad1.x) {
            config.setPoseEstimate(new Pose2d(5.0, -56.0, 0.0));
        }
        if(gamepad1.start) {
            returnToShoot.start();
            while(returnToShoot.isAlive());
        }
        else if(gamepad1.back) {
            powerShots.start();
            while(powerShots.isAlive());
        }
        else setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(gamepad1.b) config.loader.set(load);
        else config.loader.set(reload);
        if(gamepad1.left_bumper) config.gripper.set(gripperClosed);
        else if(gamepad1.right_bumper) config.gripper.set(gripperOpen);
        if(gamepad1.left_trigger > 0.2) config.wobble.set(wobbleDown);
        else if(gamepad1.right_trigger > 0.2) config.wobble.set(wobbleUp);
        if(gamepad1.y) config.ingester.set(0);
        else config.ingester.set(600);
    }

    public void configureMacros() {
        Sequence tripleShoot = new Sequence(() -> {
            for(int i = 0; i < 3; i++) {
                shootOnce();
                sleep(200);
            }
            return null;
        });
        Sequence returnToHighGoal = new Sequence(() -> {
            config.setPoseEstimate(config.getPoseEstimate());
            Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(highGoalShoot, new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
            return null;
        }, tripleShoot);
        returnToShoot = new Thread(returnToHighGoal);

        Sequence pivotShoot = new Sequence(() -> {
            for(int i = 0; i < 2; i++) {
                shootOnce();
                config.turn(Math.toRadians(8));
                sleep(200);
            }
            shootOnce();
            return null;
        });
        Sequence returnToPowerShot = new Sequence(() -> {
            config.setPoseEstimate(config.getPoseEstimate());
            Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(powerShotShoot, new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
            return null;
        }, pivotShoot);
        powerShots = new Thread(returnToPowerShot);
    }

    public void shootOnce() {
        config.loader.set(load);
        sleep(800);
        config.loader.set(reload);
    }

    public void setPower(double x, double y, double a){
        config.leftRear.setPower(-x + y + a);
        config.leftFront.setPower(x + y + a);
        config.rightFront.setPower(-x + y - a);
        config.rightRear.setPower(x + y - a);
    }
}
