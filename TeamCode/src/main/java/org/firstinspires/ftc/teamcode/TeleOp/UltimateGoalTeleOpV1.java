package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.DriveObjectRobotMovement;
import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.Point;
import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.HardwareConfigs.UltimateGoalConfig;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Configuration;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ConfigurationRR;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DMotor;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Sequence;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ValueStorage;

@Config
@TeleOp
public class UltimateGoalTeleOpV1 extends LinearOpMode {

    ConfigurationRR config;
    DriveObjectRobotMovement drive;
    ValueStorage vals = new ValueStorage();
    HardwareThread hardware;
    Thread returnToShoot, powerShots;
    private double header;
    public static double wobbleUp = 0.22, wobbleDown = 0.65, wobbleMid = 0.45, gripperOpen = 0, gripperClosed = 1, load = 0.6, reload = 0.24, shooterSpeed = -1700;

    Pose2d highGoalShoot = new Pose2d(-12.0, -25.0, 0);
    Pose2d powerShotShoot = new Pose2d(-30.0, -36.0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            config = new ConfigurationRR(hardwareMap);
            config.Configure(hardwareMap, vals);
            drive = new DriveObjectRobotMovement(config);
            hardware = new HardwareThread(hardwareMap, vals, config);
            for(DMotor d : config.motors) d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        config.update();
        telemetry.addData("Pose: ", config.getPoseEstimate());
        telemetry.addData("Heading: ", config.imu.get()[0]);
        telemetry.update();
        config.shooter.set(shooterSpeed);
        if(gamepad1.x) {
            config.imu.resetIMU();
            header = 0;
            config.setPoseEstimate(new Pose2d(5.0, -63.0, 0.0));
        }
        if(gamepad1.start) {
            returnToShoot.start();
            //config.setPoseEstimate(config.getPoseEstimate());
            /*Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(highGoalShoot.getX(), highGoalShoot.getY(), header), new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
             */
            while(returnToShoot.isAlive());
        }
        else if(gamepad1.dpad_down) {
            drive.runWithEncoder(false);
            config.motors.get(0).reverse(false);
            config.motors.get(1).reverse(false);
            config.motors.get(2).reverse(true);
            config.motors.get(3).reverse(true);
            Pose2d currentPose = config.getPoseEstimate();
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
            while((Math.hypot(Math.abs(highGoalShoot.getX()-currentPose.getX()),Math.abs(highGoalShoot.getY()-currentPose.getY()))>0.75)&&!isStopRequested()){
                config.update();
                currentPose = config.getPoseEstimate();
                drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
                telemetry.addData("Position: ", currentPose);
                telemetry.addData("Pose: ", drive.robotPosition + " " + drive.worldAngle);
                telemetry.update();
                drive.goToPosition(highGoalShoot.getX(),highGoalShoot.getY(),0.8,270,0.7);
            }
            drive.setPower(0,0,0);
            sleep(200);
            double imuHeading = config.imu.get()[0];
            while((Math.abs(imuHeading-header)>Math.toRadians(2)) && !isStopRequested()){
                config.update();
                imuHeading = config.imu.get()[0];
                int invert = (imuHeading + (360 - header)) % 360 > 180 ? -1 : 1;
                double p = 0.2, f = 0.15;
                drive.setPower(0,0, invert * (f+(p*Math.abs(imuHeading-header))));
            }
            drive.runWithEncoder(true);
            config.motors.get(0).reverse(true);
            config.motors.get(1).reverse(true);
            config.motors.get(2).reverse(false);
            config.motors.get(3).reverse(false);
        }
        else if(gamepad1.back) {
            //config.setPoseEstimate(config.getPoseEstimate());
            //config.turn(Math.toRadians(12));
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
        else config.ingester.set(1);
    }

    public void configureMacros() {
        /*Sequence returnToHighGoal = new Sequence(() -> {
            config.setPoseEstimate(config.getPoseEstimate());
            Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(highGoalShoot.getX(), highGoalShoot.getY(), header), new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
            return null;
        });
         */
        Sequence returnToHighGoal = new Sequence(() -> {
            purePursuitToPosition(highGoalShoot);
            return null;
        });
        Sequence tripleShoot = new Sequence(() -> {
            for(int i = 0; i < 3; i++) {
                shootOnce();
                sleep(500);
            }
            return null;
        }, returnToHighGoal);
        returnToShoot = new Thread(tripleShoot);


        /*Sequence returnToPowerShot = new Sequence(() -> {
            config.setPoseEstimate(config.getPoseEstimate());
            Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(powerShotShoot, new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
            return null;
        });
         */
        Sequence returnToPowerShot = new Sequence(() -> {
            purePursuitToPosition(powerShotShoot);
            return null;
        });
        Sequence pivotShoot = new Sequence(() -> {
            config.shooter.set(-1500);
            for(int i = 0; i < 2; i++) {
                shootOnce();
                config.turn(Math.toRadians(-8));
                sleep(400);
            }
            shootOnce();
            return null;
        }, returnToPowerShot);
        powerShots = new Thread(pivotShoot);
    }

    public void purePursuitToPosition(Pose2d targetPose) {
        drive.runWithEncoder(false);
        config.motors.get(0).reverse(false);
        config.motors.get(1).reverse(false);
        config.motors.get(2).reverse(true);
        config.motors.get(3).reverse(true);
        Pose2d currentPose = config.getPoseEstimate();
        drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
        while((Math.hypot(Math.abs(targetPose.getX()-currentPose.getX()),Math.abs(targetPose.getY()-currentPose.getY()))>0.7)&&!isStopRequested()){
            config.update();
            currentPose = config.getPoseEstimate();
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
            telemetry.addData("Positiom: ", currentPose);
            telemetry.addData("Pose: ", drive.robotPosition + " " + drive.worldAngle);
            telemetry.update();
            drive.goToPosition(targetPose.getX(),targetPose.getY(),1,270,0.8);
        }
        drive.setPower(0,0,0);
        sleep(200);
        double imuHeading = config.imu.get()[0];
        while((Math.abs(imuHeading-targetPose.getHeading())>Math.toRadians(2)) && !isStopRequested()){
            config.update();
            imuHeading = config.imu.get()[0];
            int invert = (imuHeading + (360 - targetPose.getHeading())) % 360 > 180 ? -1 : 1;
            double p = 0.2, f = 0.15;
            drive.setPower(0,0, invert * (f+(p*Math.abs(imuHeading-targetPose.getHeading()))));
        }
        for(DMotor d : drive.Motors) d.setPower(0);
        drive.runWithEncoder(true);
        config.motors.get(0).reverse(true);
        config.motors.get(1).reverse(true);
        config.motors.get(2).reverse(false);
        config.motors.get(3).reverse(false);
    }

    public void shootOnce() {
        config.loader.set(load);
        sleep(800);
        config.loader.set(reload);
    }

    public void setPower(double x, double y, double a){
        config.leftRear.setPower(x + y + a);
        config.leftFront.setPower(-x + y + a);
        config.rightFront.setPower(x + y - a);
        config.rightRear.setPower(-x + y - a);
    }
}
