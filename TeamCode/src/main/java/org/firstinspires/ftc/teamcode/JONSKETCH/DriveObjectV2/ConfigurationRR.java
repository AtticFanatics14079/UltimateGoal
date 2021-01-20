package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveObjectTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.Utils.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants.getMotorVelocityF;

@Config
public class ConfigurationRR extends MecanumDrive implements Configuration {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(2, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(1, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private Pose2d lastPoseOnTurn;

    public DEncoderlessMotor leftFront, leftRear, rightRear, rightFront;
    public DOdometryPod leftEncoder, rightEncoder, frontEncoder;
    public DServo loader, gripper, wobble;
    public DMotor shooter;
    public DEncoderlessMotor ingester;
    public List<DOdometryPod> pods;
    public List<DEncoderlessMotor> motors;
    private List<LynxModule> allHubs;
    public DIMU imu;
    private VoltageSensor batteryVoltageSensor;

    public ConfigurationRR(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(new DriveConstraints(
                100.0, 80.0, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0
        ), TRACK_WIDTH); //Just has to be high enough to not mess with driving.
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        allHubs = hardwareMap.getAll(LynxModule.class);

        // TODO: adjust the names of the following hardware devices to match your configuration

    }

    public void Configure(HardwareMap hwMap, ValueStorage vals){
        //Add all hardware devices here.
        //Example: hardware.put("motor1", new DriveObject(DriveObject.type.DcMotorImplEx, "left_back_motor", DriveObject.classification.Drivetrain, hwMap));
        //In this example, "left_back_motor" is whatever your configuration says.
        int i = 0;

        hardware.clear();

        leftFront = new DEncoderlessMotor(vals, hwMap, "front_left_motor", i++);
        leftRear = new DEncoderlessMotor(vals, hwMap, "back_left_motor", i++);
        rightRear = new DEncoderlessMotor(vals, hwMap, "back_right_motor", i++);
        rightFront = new DEncoderlessMotor(vals, hwMap, "front_right_motor", i++);
        leftEncoder = new DOdometryPod(vals, hwMap, "leftEncoder", i++);
        rightEncoder = new DOdometryPod(vals, hwMap, "rightEncoder", i++);
        frontEncoder = new DOdometryPod(vals, hwMap, "frontEncoder", i++);
        loader = new DServo(vals, hwMap, "loader", i++);
        gripper = new DServo(vals, hwMap, "gripper", i++);
        wobble = new DServo(vals, hwMap, "wobble", i++);
        shooter = new DMotor(vals, hwMap, "shooter", i++);
        ingester = new DEncoderlessMotor(vals, hwMap, "rightEncoder", i++);
        imu = new DIMU(vals, hwMap, i++);
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        pods = Arrays.asList(leftEncoder, rightEncoder, frontEncoder);
        hardware.add(motors.get(0));
        hardware.add(motors.get(1));
        hardware.add(motors.get(2));
        hardware.add(motors.get(3));
        hardware.add(leftEncoder);
        hardware.add(rightEncoder);
        hardware.add(frontEncoder);
        hardware.add(loader);
        hardware.add(gripper);
        hardware.add(wobble);
        hardware.add(shooter);
        hardware.add(ingester);
        hardware.add(imu);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        //Not sure what the next part does so if stuff is wonky check it.
        for (DEncoderlessMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setBulkCachingManual(true);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(MOTOR_VELO_PID);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        motors.get(0).reverse(true);
        motors.get(1).reverse(true);

        leftEncoder.reverse(true);
        rightEncoder.reverse(true);
        frontEncoder.reverse(true);
        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new DriveObjectTrackingWheelLocalizer(vals, this));
    }

    public void setBulkCachingManual(boolean manual){
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(manual ? LynxModule.BulkCachingMode.MANUAL : LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void clearBulkCache(){
        for (LynxModule module : allHubs) {
            if(module.getBulkCachingMode() == LynxModule.BulkCachingMode.MANUAL) {
                System.out.println("Clearing");
                module.clearBulkCache();
                //module.getBulkData();
            }
        }
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();

        lastPoseOnTurn = getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        turnStart = clock.seconds();
        mode = ConfigurationRR.Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawRobot(fieldOverlay, newPose);

                if (t >= turnProfile.duration()) {
                    mode = ConfigurationRR.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

                if (!follower.isFollowing()) {
                    mode = ConfigurationRR.Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    //Need to fix these at some point
    public void setMode(DcMotor.RunMode runMode) {
        for (DEncoderlessMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DEncoderlessMotor motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public PIDFCoefficients getPIDFCoefficients() { //Removed DcMotor.RunMode runMode
        double[] temp = motors.get(0).getPID();
        PIDFCoefficients coefficients = new PIDFCoefficients(temp[0], temp[1], temp[2], temp[3]); //Change last value later
        return new PIDFCoefficients(coefficients.p, coefficients.i, coefficients.d, coefficients.f);
    }

    public void setPIDFCoefficients(PIDFCoefficients coefficients) { //Removed DcMotor.RunMode runMode,
        for (DEncoderlessMotor motor : motors) {

            //ONLY ENCODERLESS FOR THIS SPECIFIC SCRIM

            //motor.setInternalPID(coefficients.p, coefficients.p, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage());
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DriveObject motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.get()[1]));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DriveObject motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.get()[0]));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        motors.get(0).setPower(v);
        motors.get(1).setPower(v1);
        motors.get(2).setPower(v2);
        motors.get(3).setPower(v3);
    }

    //Fix later
    @Override
    public double getRawExternalHeading() {
        return imu.get()[0];
    }
}