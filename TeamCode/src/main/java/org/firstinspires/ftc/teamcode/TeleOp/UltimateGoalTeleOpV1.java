package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@Config
@TeleOp
public class UltimateGoalTeleOpV1 extends LinearOpMode {

    ConfigurationRR config;
    DriveObjectRobotMovement drive;
    ValueStorage vals = new ValueStorage();
    HardwareThread hardware;
    Thread returnToShoot, powerShots, shootMacro, grabWobble, dropWobble, lowerWobble;
    public static double wobbleUp = 0.22, wobbleDown = 0.65, wobbleMid = 0.45, gripperOpen = 0, gripperClosed = 1, load = 0.46, reload = 0.11, shooterSpeed = -1660;

    public static double highGoalX = 0, highGoalY = 0, powerShotX = 0, powerShotY = 0, wallDistance = 18, distanceLeft = 21, distanceRight = 15;

    private

    Pose2d highGoalShoot = new Pose2d(highGoalX, highGoalY, 0);
    Pose2d powerShotShoot = new Pose2d(powerShotX, powerShotY, 0);

    DistanceSensor leSense;

    private boolean lockedLoader = false, pressedLock = false, pressedShooter = false, shooterFast = true, pressedOdoAdjust = false;

    double intakeSpeed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            leSense = hardwareMap.get(DistanceSensor.class, "distanceRight");
            config = new ConfigurationRR(hardwareMap);
            config.Configure(hardwareMap, vals);
            drive = new DriveObjectRobotMovement(config);
            hardware = new HardwareThread(hardwareMap, vals, config);
            hardware.start();
            for(DEncoderlessMotor d : config.motors) d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            configureMacros();
            waitForStart();
            if(!isStopRequested()) {
                while(opModeIsActive()){
                    vals.waitForCycle();
                    getInput();
                }
            }
        } catch(Exception e){
            System.out.println(e);
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
        if(gamepad2.start) {
            config.setPoseEstimate(new Pose2d(0, 0, 0));
            config.imu.resetIMU();
        }
        if(gamepad2.left_stick_y > 0.5) shooterSpeed = -1540;
        else if(gamepad2.left_stick_y < -0.5) shooterSpeed = -1660;
        config.shooter.set(shooterFast ? shooterSpeed : 0);
        //if(gamepad1.start) { //Will be changed later
            //config.imu.resetIMU();
            //if(time.seconds() < 5) config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), 0.0));
            //config.setPoseEstimate(new Pose2d(-63, -63, Math.PI)); //In corner, AKA after dropping off wobble goal.
        //}
        if(gamepad1.start && !shootMacro.isAlive()) {
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
        else if(gamepad1.a && !shootMacro.isAlive() && config.ingester.get()[0] >= shooterSpeed - 500) { //Just making sure we don't shoot until after shooter reaches speed, may add this check to the macro.
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
            //powerShots.start();
            //while(powerShots.isAlive());
        }
        double multiplier = 1;
        if(gamepad1.left_bumper) multiplier = 0.2;
        setPower(multiplier * gamepad1.left_stick_x, multiplier * gamepad1.left_stick_y, multiplier * gamepad1.right_stick_x);
        if(!pressedOdoAdjust && (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down)) {
            Pose2d currentPose = config.getPoseEstimate();
            config.setPoseEstimate(new Pose2d(currentPose.getX() + (gamepad2.dpad_up ? -2 : (gamepad2.dpad_down ? 2 : 0)), currentPose.getY() + (gamepad2.dpad_left ? -2 : (gamepad2.dpad_right ? 2 : 0)), currentPose.getHeading()));
            pressedOdoAdjust = true;
        }
        else if(pressedOdoAdjust && !(gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down)) pressedOdoAdjust = false;
        if(shootMacro.isAlive()) {}
        else if((gamepad1.right_bumper || gamepad2.back) && !pressedLock) {
            lockedLoader = !lockedLoader;
            pressedLock = true;
        }
        else if(pressedLock && !gamepad1.right_bumper && !gamepad2.back) pressedLock = false;
        else if(gamepad1.b || lockedLoader && config.ingester.get()[0] >= shooterSpeed - 500) config.loader.set(load);
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

        if(gamepad2.right_stick_button) intakeSpeed = -1;
        else if(gamepad2.b) intakeSpeed = 0;
        else if(gamepad2.left_stick_button) intakeSpeed = 1;
        config.ingester.set(intakeSpeed);

        if(gamepad1.right_stick_button) {
            config.imu.retrievingHardware(true);
            sleep(40);
            double imuHeading = config.imu.get()[0];
            while((Math.abs(imuHeading)>Math.toRadians(0.5)) && !isStopRequested() && opModeIsActive()){
                config.update();
                imuHeading = config.imu.get()[0];
                if(imuHeading < 0) imuHeading += 2 * Math.PI;
                //currentPose = drive.getPoseEstimate();
                double p = 0.35, f = 0.04;
                int invert = ((2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
                double power = invert * p * (Math.abs(imuHeading) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((0 > Math.PI ? 2 * Math.PI : 0) - 0)) : Math.abs(imuHeading - 0)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
                power += (power > 0 ? f : -f);
                drive.setPower(0, 0, -power);
            }
            drive.setPower(0, 0, 0);
            Pose2d currentPose = config.getPoseEstimate();
            config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
            config.imu.retrievingHardware(false);
        }
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
            roadRunnerToPosition(highGoalShoot, 0.8);
            highGoalWallAdjust();
            return null;
        });
        Sequence tripleShoot = new Sequence(() -> {
            config.shooter.set(-1660);
            for(int i = 0; i < 3; i++) {
                shootOnce();
                config.shooter.set(-1600);
                sleep(300);
            }
            config.loader.set(load);
            return null;
        }, returnToHighGoal);
        returnToShoot = new Thread(tripleShoot);

        Sequence shootThrice = new Sequence(() -> {
            if(lockedLoader) {
                config.loader.set(reload);
                lockedLoader = false;
                sleep(300);
            }
            for(int i = 0; i < 2; i++) {
                shootOnce();
                sleep(300);
            }
            shootOnce();
            config.loader.set(load);
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
            purePursuitToPosition(powerShotShoot, 0.7);
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
            config.loader.set(load);
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
        while((Math.hypot(Math.abs(targetPose.getX()-currentPose.getX()),Math.abs(targetPose.getY()-currentPose.getY()))>0.5)&&!isStopRequested()){
            config.update();
            currentPose = config.getPoseEstimate();
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
            //t.addData("Current Position (Odo): ", currentPose);
            //t.addData("Target Point: ", targetPose);
            //t.update();
            drive.goToPosition(targetPose.getX(),targetPose.getY(),speed,90,0.3);
        }
        drive.setPower(0,0,0);
        sleep(500);
        if(isStopRequested()) return;
        config.imu.retrievingHardware(true);
        sleep(200);
        double imuHeading = config.imu.get()[0];
        while((Math.abs(imuHeading - targetPose.getHeading())>Math.toRadians(1)) && !isStopRequested() && opModeIsActive()){
            config.update();
            imuHeading = config.imu.get()[0];
            if(imuHeading < 0) imuHeading += 2 * Math.PI;
            //currentPose = drive.getPoseEstimate();
            double p = 0.3, f = 0.07;
            int invert = (targetPose.getHeading() + (2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double power = invert * p * (Math.abs(imuHeading - targetPose.getHeading()) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((targetPose.getHeading() > Math.PI ? 2 * Math.PI : 0) - targetPose.getHeading())) : Math.abs(imuHeading - targetPose.getHeading())); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setPower(0, 0, power);
        }
        if(isStopRequested()) return;
        config.imu.retrievingHardware(false);
        double distance = leSense.getDistance(DistanceUnit.INCH);
        currentPose = config.getPoseEstimate();
        if(distance > distanceLeft || distance < distanceRight) {
            Trajectory traj = config.trajectoryBuilder(currentPose)
                    .strafeTo(new Vector2d(currentPose.getX(), currentPose.getY() - distance + 20))
                    .build();
            config.followTrajectory(traj);
        }
        if(isStopRequested()) return;
        drive.setPower(0, 0, 0);
        sleep(200);
        //drive.runWithEncoder(true);
        config.motors.get(0).reverse(true);
        config.motors.get(1).reverse(true);
        config.motors.get(2).reverse(false);
        config.motors.get(3).reverse(false);
    }

    public void roadRunnerToPosition(Pose2d targetPose, double speed) {
        config.motors.get(0).reverse(false);
        config.motors.get(1).reverse(false);
        config.motors.get(2).reverse(true);
        config.motors.get(3).reverse(true);
        drive.runWithEncoder(true);
        Pose2d currentPose = config.getPoseEstimate();
        Trajectory traj = config.trajectoryBuilder(currentPose)
                .lineToLinearHeading(targetPose)
                .build();
        config.followTrajectory(traj);
        drive.setPower(0,0,0);
        config.imu.retrievingHardware(true);
        sleep(40);
        double imuHeading = config.imu.get()[0];
        if(Math.abs(imuHeading) > Math.toRadians(4)) {
            while((Math.abs(imuHeading)>Math.toRadians(0.5)) && !isStopRequested() && opModeIsActive()){
                config.update();
                imuHeading = config.imu.get()[0];
                if(imuHeading < 0) imuHeading += 2 * Math.PI;
                currentPose = config.getPoseEstimate();
                double p = 0.35, f = 0.04;
                int invert = ((2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
                double power = invert * p * (Math.abs(imuHeading) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((0 > Math.PI ? 2 * Math.PI : 0) - 0)) : Math.abs(imuHeading - 0)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
                power += (power > 0 ? f : -f);
                drive.setPower(0, 0, power);
            }
            drive.setPower(0, 0, 0);
            config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
            config.imu.retrievingHardware(false);
        }
        /*config.imu.retrievingHardware(true);
        sleep(40);
        double imuHeading = config.imu.get()[0];
        while((Math.abs(imuHeading - targetPose.getHeading())>Math.toRadians(0.5)) && !isStopRequested() && opModeIsActive()){
            config.update();
            imuHeading = config.imu.get()[0];
            if(imuHeading < 0) imuHeading += 2 * Math.PI;
            //currentPose = drive.getPoseEstimate();
            double p = 0.35, f = 0.04;
            int invert = (targetPose.getHeading() + (2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double power = invert * p * (Math.abs(imuHeading - targetPose.getHeading()) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((targetPose.getHeading() > Math.PI ? 2 * Math.PI : 0) - targetPose.getHeading())) : Math.abs(imuHeading - targetPose.getHeading())); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setPower(0, 0, power);
        }
        if(isStopRequested()) return;
        drive.setPower(0, 0, 0);
        currentPose = config.getPoseEstimate();
        imuHeading = config.imu.get()[0];
        config.imu.retrievingHardware(false);
        config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
         */

    }

    private void highGoalWallAdjust() {
        double distance = leSense.getDistance(DistanceUnit.INCH);
        System.out.println("Distance sensor: " + distance);
        if((distance > distanceLeft || distance < distanceRight) && distance < 300) {
            Pose2d currentPose = config.getPoseEstimate();
            Trajectory correct = config.trajectoryBuilder(config.getPoseEstimate())
                    .strafeRight(distance - wallDistance)
                    .build();
            config.followTrajectory(correct);
            config.imu.retrievingHardware(true);
            sleep(40);
            double imuHeading = config.imu.get()[0];
            config.setPoseEstimate(new Pose2d(currentPose.getX(), distance - wallDistance, imuHeading));
            while((Math.abs(imuHeading)>Math.toRadians(0.5)) && !isStopRequested() && opModeIsActive()){
                config.update();
                imuHeading = config.imu.get()[0];
                if(imuHeading < 0) imuHeading += 2 * Math.PI;
                //currentPose = drive.getPoseEstimate();
                double p = 0.35, f = 0.04;
                int invert = ((2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
                double power = invert * p * (Math.abs(imuHeading) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((0 > Math.PI ? 2 * Math.PI : 0) - 0)) : Math.abs(imuHeading - 0)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
                power += (power > 0 ? f : -f);
                drive.setPower(0, 0, power);
            }
            drive.setPower(0, 0, 0);
            config.imu.retrievingHardware(false);
        }
        if(isStopRequested()) return;
        drive.runWithEncoder(false);
        config.motors.get(0).reverse(true);
        config.motors.get(1).reverse(true);
        config.motors.get(2).reverse(false);
        config.motors.get(3).reverse(false);
    }

    public void shootOnce() {
        config.loader.set(load);
        sleep(450);
        config.loader.set(reload);
    }

    public void setPower(double x, double y, double a){
        config.leftRear.setPower(x + y + a);
        config.leftFront.setPower(-x + y + a);
        config.rightFront.setPower(x + y - a);
        config.rightRear.setPower(-x + y - a);
    }
}
