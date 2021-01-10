package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.CVConfig;

@Config
@TeleOp
public class SweepTest extends LinearOpMode {

    CVConfig config;

    public static double open = 0, resting = 0.5, closed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        config = new CVConfig();
        config.Configure(hardwareMap);

        waitForStart();

        while(!isStopRequested()) {
            setPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            //config.sweep.setPower(gamepad1.a ? 1 : (gamepad1.b ? -1 : 0)); //In case it's the wrong way

            config.wiperRight.setPosition(gamepad1.x ? open : (gamepad1.y ? closed : resting));
            config.wiperLeft.setPosition(gamepad1.x ? closed : (gamepad1.y ? open : resting)); //Just trust the sketch, I think the servos are symmetric but mirrored.
        }
    }

    public void setPower(double x, double y, double a) {
        config.backLeft.setPower(x + y + a);
        config.frontLeft.setPower(-x + y + a);
        config.frontRight.setPower(x + y - a);
        config.backRight.setPower(-x + y - a);
    }
}
