package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.SampleConfiguration;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Sequence;

@Config
@TeleOp
public class TestTeleOp extends LinearOpMode {

    SampleConfiguration config;

    boolean turning = false;

    public static double openPos = 0, closePos = 1;

    Thread waitThread;

    @Override
    public void runOpMode() throws InterruptedException {

        config  = new SampleConfiguration();
        HardwareThread hardware = new HardwareThread(hardwareMap, config);
        hardware.start();

        double lastHeading = 0, ingesterSpeed = 0;

        boolean pressed = false;

        config.imu.gettingInput = true;

        waitForStart();

        Sequence waitMillis = new Sequence(() -> {
            sleep(140);
            turning = false;
        }, null);
        waitThread = new Thread(waitMillis);

        while(!isStopRequested()) {

            config.ingest.setPower(ingesterSpeed);

            if(gamepad1.b && !pressed) {
                ingesterSpeed += 0.1;
                pressed = true;
            }
            else if(gamepad1.a && !pressed) {
                ingesterSpeed -= 0.1;
                pressed = true;
            }
            else if(!gamepad1.a && !gamepad1.b) pressed = false;

            double imuHeading = config.imu.get()[0];
            double tempHeading = imuHeading;
            double tempTarget = lastHeading;
            if(tempHeading < 0) tempHeading += 2 * Math.PI;
            if(lastHeading < 0) tempTarget += 2 * Math.PI;
            double invert = lastHeading - imuHeading;
            if(invert > Math.PI) invert -= 2 * Math.PI;
            else if(invert < -Math.PI) invert += 2 * Math.PI;
            invert = invert < 0 ? 1 : -1;
            double power = invert * 0.5 * (Math.abs(tempHeading - tempTarget) > Math.PI ? (Math.abs(tempHeading > Math.PI ? 2 * Math.PI - tempHeading : tempHeading) + Math.abs(tempTarget > Math.PI ? 2 * Math.PI - tempTarget : tempTarget)) : Math.abs(tempHeading - tempTarget)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            if(Math.abs(gamepad1.right_stick_x) > 0.05) turning = true;

            else if(turning && !waitThread.isAlive()) waitThread.start();

            if(turning) {
                power = 0;
                lastHeading = imuHeading;
            }
            if(Math.abs(power) < 0.02) power = 0;

            telemetry.addData("Heading: ", imuHeading);
            telemetry.addData("Sped: ", config.ingest.get()[0]);
            telemetry.update();

            double speed = gamepad1.left_bumper ? 0.4 : 1;

            setPower(-speed * (Math.cos(tempHeading)*gamepad1.left_stick_y+Math.sin(tempHeading)*gamepad1.left_stick_x), -speed * (-Math.cos(tempHeading)*gamepad1.left_stick_x+Math.sin(tempHeading)*gamepad1.left_stick_y), speed * gamepad1.right_stick_x+power);

            double slidePower = gamepad2.left_stick_y;
            //if(config.limit.get()[0] == 1 && slidePower < 0) slidePower = 0;

            config.slides.setPower(slidePower);

            if(gamepad1.x) config.servo.set(openPos);
            else if(gamepad1.y) config.servo.set(closePos);
        }

        hardware.Stop();
    }

    public void setPower(double x, double y, double a){
        config.backLeft.setPower(x + y + a);
        config.frontLeft.setPower(-x + y + a);
        config.frontRight.setPower(x + y - a);
        config.backRight.setPower(-x + y - a);
    }
}