package org.firstinspires.ftc.teamcode.Autonomous.RRTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.external.samples.PushbotAutoDriveByEncoder_Linear;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants.*;

/*
 * This routine is designed to tune the PID coefficients used by the REV Expansion Hubs for closed-
 * loop velocity control. Although it may seem unnecessary, tuning these coefficients is just as
 * important as the positional parameters. Like the other manual tuning routines, this op mode
 * relies heavily upon the dashboard. To access the dashboard, connect your computer to the RC's
 * WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're using the RC
 * phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once you've successfully
 * connected, start the program, and your robot will begin moving forward and backward according to
 * a motion profile. Your job is to graph the velocity errors over time and adjust the PID
 * coefficients (note: the tuning variable will not appear until the op mode finishes initializing).
 * Once you've found a satisfactory set of gains, add them to the DriveConstants.java file under the
 * MOTOR_VELO_PID field.
 *
 * Recommended tuning process:
 *
 * 1. Increase kP until any phase lag is eliminated. Concurrently increase kD as necessary to
 *    mitigate oscillations.
 * 2. Add kI (or adjust kF) until the steady state/constant velocity plateaus are reached.
 * 3. Back off kP and kD a little until the response is less oscillatory (but without lag).
 */
@Config
@Autonomous(group = "drive")
public class DriveVelocityPIDTuner extends LinearOpMode {
    public static double DISTANCE = 96; // in

    private static final String PID_VAR_NAME = "VELO_PID";

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private String catName;
    private CustomVariable catVar;

    //private SampleMecanumDrive drive;
    private SampleMecanumDrive drive;

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                DriveConstants.BASE_CONSTRAINTS.maxVel,
                DriveConstants.BASE_CONSTRAINTS.maxAccel,
                DriveConstants.BASE_CONSTRAINTS.maxJerk);
    }

    private void addPidVariable() {
        catName = getClass().getSimpleName();
        catVar = (CustomVariable) dashboard.getConfigRoot().getVariable(catName);
        if (catVar == null) {
            // this should never happen...
            catVar = new CustomVariable();
            dashboard.getConfigRoot().putVariable(catName, catVar);

            RobotLog.w("Unable to find top-level category %s", catName);
        }

        CustomVariable pidVar = new CustomVariable();
        pidVar.putVariable("kP", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
            }

            @Override
            public void set(Double value) {
                PIDFCoefficients coeffs = drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(value, coeffs.i, coeffs.d, coeffs.f));
            }
        }));
        pidVar.putVariable("kI", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
            }

            @Override
            public void set(Double value) {
                PIDFCoefficients coeffs = drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(coeffs.p, value, coeffs.d, coeffs.f));
            }
        }));
        pidVar.putVariable("kD", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
            }

            @Override
            public void set(Double value) {
                PIDFCoefficients coeffs = drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(coeffs.p, coeffs.i, value, coeffs.f));
            }
        }));

        pidVar.putVariable("kF", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;
            }

            @Override
            public void set(Double value) {
                PIDFCoefficients coeffs = drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(coeffs.p, coeffs.i, coeffs.d, value));
            }
        }));
        catVar.putVariable(PID_VAR_NAME, pidVar);
        dashboard.updateConfig();
    }

    private void removePidVariable() {
        if (catVar.size() > 1) {
            catVar.removeVariable(PID_VAR_NAME);
        } else {
            dashboard.getConfigRoot().removeVariable(catName);
        }
        dashboard.updateConfig();
    }

    @Override
    public void runOpMode() {
        if (!RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("%s does not need to be run if the built-in motor velocity" +
                    "PID is not in use", getClass().getSimpleName());
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //drive = new SampleMecanumDrive(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        addPidVariable();

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            // calculate and set the motor power
            double profileTime = clock.seconds() - profileStart;

            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards;
                activeProfile = generateProfile(movingForwards);
                profileStart = clock.seconds();
            }

            MotionState motionState = activeProfile.get(profileTime);
            double targetPower = kV * motionState.getV();
            drive.setDrivePower(new Pose2d(targetPower, 0, 0));

            List<Double> velocities = drive.getWheelVelocities();

            // update telemetry
            telemetry.addData("targetVelocity", motionState.getV());
            for (int i = 0; i < velocities.size(); i++) {
                telemetry.addData("velocity" + i, velocities.get(i));
                telemetry.addData("error" + i, motionState.getV() - velocities.get(i));
            }
            telemetry.update();
        }

        removePidVariable();
    }
}
