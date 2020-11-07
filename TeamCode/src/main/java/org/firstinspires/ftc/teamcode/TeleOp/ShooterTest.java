package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.shooterTestConfig;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    public static int velo = 0;
    public static double ServoPosition = 0.9;
    @Override
    public void runOpMode() throws InterruptedException {
        shooterTestConfig config = new shooterTestConfig();
        config.Configure(hardwareMap);
        config.servo.setPosition(0.5);
        waitForStart();
        while(!isStopRequested()) {
            config.servo.setPosition(ServoPosition);
            config.motor.setVelocity(velo);
            telemetry.addData("Velocity ", config.motor.getVelocity());
            telemetry.update();
        }
    }
}
