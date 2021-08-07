package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.shooterTestConfig;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {

    public static int velo = 1700;
    public static double open = 0.15;
    public static double closed = 0.55;
    public static double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterTestConfig config = new shooterTestConfig();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        config.Configure(hardwareMap);
        waitForStart();

        while(!isStopRequested()) {
            config.shooter.setPower(power);
        }


    }
}
