package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.DriveObjectRobotMovement;
import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.Point;
import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.HardwareConfigs.UltimateGoalConfig;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Configuration;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ConfigurationRR;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DEncoderlessMotor;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DMotor;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Sequence;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ValueStorage;

//@Config
@TeleOp
public class UltimateGoalTeleOpV1 extends LinearOpMode {

    ConfigurationRR config;
    DriveObjectRobotMovement drive;
    ValueStorage vals = new ValueStorage();
    HardwareThread hardware;
    Thread returnToShoot, powerShots, shootMacro, grabWobble, dropWobble, lowerWobble;
    public static double wobbleUp = 0.22, wobbleDown = 0.65, wobbleMid = 0.45, gripperOpen = 0, gripperClosed = 1, load = 0.6, reload = 0.2, shooterSpeed = -1700;

    public static double highGoalX = -6, highGoalY = -42, powerShotX = -24, powerShotY = -34;

    Pose2d highGoalShoot = new Pose2d(highGoalX, highGoalY, 0);
    Pose2d powerShotShoot = new Pose2d(powerShotX, powerShotY, 0);

    private boolean lockedLoader = false, pressedLock = false, pressedShooter = false, shooterFast = true;

    private ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            config = new ConfigurationRR(hardwareMap);
            config.Configure(hardwareMap, vals);
            drive = new DriveObjectRobotMovement(config);
            hardware = new HardwareThread(hardwareMap, vals, config);
            for(DEncoderlessMotor d : config.motors) d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            configureMacros();
            waitForStart();
            time = new ElapsedTime();
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
        config.imu.retrievingHardware(false);
        telemetry.addData("Pose: ", config.getPoseEstimate());
        telemetry.addData("Heading: ", config.imu.get()[0]);
        telemetry.update();
        if(gamepad2.a && !pressedShooter) {
            pressedShooter = true;
            shooterFast = !shooterFast;
        }
        else if(pressedShooter && !gamepad2.a) pressedShooter = false;
        if(gamepad2.dpad_down) shooterSpeed = -1540;
        else if(gamepad2.dpad_up) shooterSpeed = -1700;
        config.shooter.set(shooterFast ? shooterSpeed : 0);
        if(gamepad1.start) { //Will be changed later
            //config.imu.resetIMU();
            //if(time.seconds() < 5) config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), 0.0));
            config.setPoseEstimate(new Pose2d(-63, -63, Math.PI)); //In corner, AKA after dropping off wobble goal.
        }
        if(gamepad1.start && !shootMacro.isAlive()) {
            //returnToShoot.start();
            //config.setPoseEstimate(config.getPoseEstimate());
            /*Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(highGoalShoot.getX(), highGoalShoot.getY(), header), new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
             */
            //while(returnToShoot.isAlive());
        }
        else if(gamepad1.a && !shootMacro.isAlive()) {
            shootMacro.start();
            /*drive.runWithEncoder(false);
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
            while((Math.abs(imuHeading)>Math.toRadians(2)) && !isStopRequested()){
                config.update();
                imuHeading = config.imu.get()[0];
                int invert = imuHeading > 180 ? -1 : 1;
                double p = 0.2, f = 0.15;
                drive.setPower(0,0, invert * (f+(p*Math.abs(imuHeading))));
            }
            drive.runWithEncoder(true);
            config.motors.get(0).reverse(true);
            config.motors.get(1).reverse(true);
            config.motors.get(2).reverse(false);
            config.motors.get(3).reverse(false);
             */
        }
        else if(gamepad1.back && !shootMacro.isAlive()) {
            //config.setPoseEstimate(config.getPoseEstimate());
            //config.turn(Math.toRadians(12));
            powerShots.start();
            while(powerShots.isAlive());
        }
        double multiplier = 1;
        if(gamepad1.left_bumper) multiplier = 0.2;
        setPower(multiplier * gamepad1.left_stick_x, multiplier * gamepad1.left_stick_y, (multiplier == 0.2 ? 0.1 : multiplier) * gamepad1.right_stick_x);
        if(shootMacro.isAlive()) {}
        else if((gamepad1.right_bumper || gamepad2.back) && !pressedLock) {
            lockedLoader = !lockedLoader;
            pressedLock = true;
        }
        else if(pressedLock && !gamepad1.right_bumper && !gamepad2.back) pressedLock = false;
        else if(gamepad1.b || lockedLoader) config.loader.set(load);
        else config.loader.set(reload);

        if(gamepad1.x && !grabWobble.isAlive() && !dropWobble.isAlive() && !lowerWobble.isAlive()) grabWobble.start();
        else if(gamepad2.x && !grabWobble.isAlive() && !dropWobble.isAlive() && !lowerWobble.isAlive()) dropWobble.start();
        else if(gamepad2.y && !grabWobble.isAlive() && !dropWobble.isAlive() && !lowerWobble.isAlive()) lowerWobble.start();

        if(grabWobble.isAlive() || dropWobble.isAlive()) {}
        else if(gamepad2.left_bumper && !grabWobble.isAlive()) config.gripper.set(gripperClosed); //Change to g2
        else if(gamepad2.right_bumper) config.gripper.set(gripperOpen); //Change to g2

        if(grabWobble.isAlive() || dropWobble.isAlive() || lowerWobble.isAlive()) {}
        else if(gamepad2.left_trigger > 0.2) config.wobble.set(wobbleUp); //Change to g2
        else if(gamepad2.right_trigger > 0.2) config.wobble.set(wobbleDown); //Change to g2
        if(gamepad1.y) config.ingester.set(-1);

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
            purePursuitToPosition(highGoalShoot, 0.8);
            return null;
        });
        Sequence tripleShoot = new Sequence(() -> {
            for(int i = 0; i < 3; i++) {
                shootOnce();
                sleep(700);
            }
            return null;
        }, returnToHighGoal);
        returnToShoot = new Thread(tripleShoot);

        Sequence shootThrice = new Sequence(() -> {
            if(lockedLoader) {
                config.loader.set(reload);
                lockedLoader = false;
                sleep(600);
            }
            for(int i = 0; i < 2; i++) {
                shootOnce();
                sleep(600);
            }
            shootOnce();
            return null;
        });
        shootMacro = new Thread(shootThrice);

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
            purePursuitToPosition(powerShotShoot, 0.6);
            return null;
        });
        Sequence pivotShoot = new Sequence(() -> {
            config.shooter.set(-1600);
            for(int i = 0; i < 2; i++) {
                shootOnce();
                config.turn(Math.toRadians(-8));
                sleep(400);
            }
            shootOnce();
            return null;
        }, returnToPowerShot);
        powerShots = new Thread(pivotShoot);

        Sequence grab = new Sequence(() -> {
            config.gripper.set(gripperClosed);
            sleep(1200);
            config.wobble.set(wobbleUp);
            return null;
        });
        grabWobble = new Thread(grab);

        Sequence drop = new Sequence(() -> {
            config.wobble.set(wobbleMid);
            sleep(200);
            config.gripper.set(gripperOpen);
            return null;
        });
        dropWobble = new Thread(drop);

        Sequence lowerToGrab = new Sequence(() -> {
            config.wobble.set(wobbleDown);
            sleep(400);
            config.gripper.set(gripperOpen);
            return null;
        });
        lowerWobble = new Thread(lowerToGrab);
    }

    public void purePursuitToPosition(Pose2d targetPose, double speed) {
        //drive.runWithEncoder(false);
        config.motors.get(0).reverse(false);
        config.motors.get(1).reverse(false);
        config.motors.get(2).reverse(true);
        config.motors.get(3).reverse(true);
        Pose2d currentPose = config.getPoseEstimate();
        drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
        while((Math.hypot(Math.abs(targetPose.getX()-currentPose.getX()),Math.abs(targetPose.getY()-currentPose.getY()))>0.6)&&!isStopRequested()){
            config.update();
            currentPose = config.getPoseEstimate();
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
            telemetry.addData("Position: ", currentPose);
            telemetry.addData("Pose: ", drive.robotPosition + " " + drive.worldAngle);
            telemetry.update();
            drive.goToPosition(targetPose.getX(),targetPose.getY(), speed,270,0.8);
        }
        drive.setPower(0,0,0);
        config.imu.retrievingHardware(true);
        sleep(200);
        double imuHeading = config.imu.get()[0];
        while((Math.abs(imuHeading-targetPose.getHeading())>Math.toRadians(2)) && !isStopRequested()){
            config.update();
            imuHeading = config.imu.get()[0];
            int invert = (imuHeading + (360 - targetPose.getHeading())) % 360 > 180 ? -1 : 1;
            double p = 0.2, f = 0.15;
            drive.setPower(0,0, invert * (f+(p*Math.abs(imuHeading-targetPose.getHeading()))));
        }
        for(DEncoderlessMotor d : drive.Motors) d.setPower(0);
        //drive.runWithEncoder(true);
        config.motors.get(0).reverse(true);
        config.motors.get(1).reverse(true);
        config.motors.get(2).reverse(false);
        config.motors.get(3).reverse(false);
        config.imu.retrievingHardware(false);
    }

    public void shootOnce() {
        config.loader.set(load);
        sleep(600);
        config.loader.set(reload);
    }

    public void setPower(double x, double y, double a){
        config.leftRear.setPower(x + y + a);
        config.leftFront.setPower(-x + y + a);
        config.rightFront.setPower(x + y - a);
        config.rightRear.setPower(-x + y - a);
    }
}
