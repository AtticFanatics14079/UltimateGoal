package org.firstinspires.ftc.teamcode.Autonomous.RoadRunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ConfigurationRR;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DMotorEncoder;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.SharedObjects;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ThreadedOdometry;
import org.firstinspires.ftc.teamcode.Utils.Encoder;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class DriveObjectTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer implements SharedObjects {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.59; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 0.993;
    public static double Y_MULTIPLER = 1.011;

    private ThreadedOdometry leftEncoder, rightEncoder, frontEncoder;

    public DriveObjectTrackingWheelLocalizer(HardwareMap hwMap) {

        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new ThreadedOdometry(hwMap, "leftEncoder");
        rightEncoder = new ThreadedOdometry(hwMap, "rightEncoder");
        frontEncoder = new ThreadedOdometry(hwMap, "frontEncoder");

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List a = Arrays.asList(
                encoderTicksToInches(leftEncoder.get()[0] * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.get()[0] * X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.get()[0]) * Y_MULTIPLER);
        return a;
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        //vals.waitForCycle();

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.get()[1]),
                encoderTicksToInches(rightEncoder.get()[1]),
                encoderTicksToInches(frontEncoder.get()[1])
        );
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }
}
