package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfigs.shooterTestConfig;

@Config
@TeleOp
public class DuckProfileTest extends LinearOpMode {

    public static int goalPos = 1000, goalVel = 0, goalAcc = 0, maxVel = 250, maxAccel = 400, maxJerk = 1000;
    public static double kP = 5, kI = 0, kD = 1;
    private ElapsedTime time = new ElapsedTime();

    //Duck carousel notes: good ducks can go about 0.1 faster on p1 and p2, and have a shorter s2 delay. However, bad ducks be scuffed. Maybe put bad ducks
    //on their side from the start to roll in? They turn over quickly regardless.

    @Override
    public void runOpMode() throws InterruptedException {
        shooterTestConfig config = new shooterTestConfig();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        config.Configure(hardwareMap);

        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0,0,0),
                new MotionState(goalPos,goalVel,goalAcc), //maybe do a velocity goal in the future, so that we can instantly stop afterwards
                maxVel,
                maxAccel,
                maxJerk
        ); // these numbers, as well as PID Coefficients, are definitely extremely wrong

        PIDCoefficients coeffs = new PIDCoefficients(kP,kI,kD);
        PIDFController controller = new PIDFController(coeffs);
        telemetry.addData("Profile Duration: ", profile.duration());
        telemetry.update();
        waitForStart();
        time.reset();

        while(!isStopRequested() && time.seconds() < profile.duration()) {
            //add controller trigger to this motion profile eventually
            int tempPosition = config.shooter.getCurrentPosition();
            //double tempVelocity = config.shooter.getVelocity();

            MotionState tempState = profile.get(time.seconds());
            controller.setTargetPosition(tempState.getX());
            controller.setTargetVelocity(tempState.getV());
            controller.setTargetAcceleration(tempState.getA());

            double correction = controller.update(tempPosition);
            config.shooter.setVelocity(correction);

            telemetry.addData("Time elapsed: ", time.seconds());
            telemetry.addData("tempState: ", tempState);
            telemetry.addData("tempPosition: ", tempPosition);
            telemetry.addData("Correction: ", correction);
            telemetry.update();
        }
    }
}
