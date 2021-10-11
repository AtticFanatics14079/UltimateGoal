package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.shooterTestConfig;

@Config
@TeleOp
public class IngestTest extends LinearOpMode {

    public static double velo = -1000;
    public static double open = 0.15;
    public static double closed = 0.55;
    public static double power = 0.85, p1 = -0.5, p2 = -0.7, p3 = -1;
    public static int s1 = 250, s2 = 300, s3 = 500;

    //Duck carousel notes: good ducks can go about 0.1 faster on p1 and p2, and have a shorter s2 delay. However, bad ducks be scuffed. Maybe put bad ducks
    //on their side from the start to roll in? They turn over quickly regardless.

    @Override
    public void runOpMode() throws InterruptedException {
        shooterTestConfig config = new shooterTestConfig();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        config.Configure(hardwareMap);
        waitForStart();

        while(!isStopRequested()) {
            /*if(gamepad1.a) {
                config.shooter.setPower(p1);
                sleep(s1);
                config.shooter.setPower(p2);
                sleep(s2);
                config.shooter.setPower(p3);
                sleep(s3);
                config.shooter.setPower(0);
            }
            telemetry.addLine("Speed: " + config.shooter.getPower());
            telemetry.update();
             */
            config.shooter.setPower(power);
        }


    }
}
