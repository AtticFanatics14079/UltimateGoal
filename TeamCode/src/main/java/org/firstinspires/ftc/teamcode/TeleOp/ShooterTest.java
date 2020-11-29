package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.shooterTestConfig;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {

    public static int velo = 1700;
    public static double open = 0.15;
    public static double closed = 0.65;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterTestConfig config = new shooterTestConfig();
        config.Configure(hardwareMap);
        config.loader.setPosition(open);
        waitForStart();
        while(!isStopRequested()) {
            if(gamepad1.a) config.loader.setPosition(closed);
            else config.loader.setPosition(open);

            config.shooter.setVelocity(velo);

            telemetry.addData("Velocity ", config.shooter.getVelocity());
            telemetry.update();
        }
        config.shooter.setVelocity(0);
    }
}
