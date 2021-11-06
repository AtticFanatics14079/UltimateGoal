package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.SampleConfiguration;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Sequence;

@Config
@TeleOp(name="ScrimTeleOp")
public class TestTeleOp extends LinearOpMode {

    SampleConfiguration config;

    boolean turning = false;

    public static double OPEN = 0.1, CLOSE = 0.5, FLIPDOWN = 0.5;

    public static int[] levels = {0, -2500, -5000, -6000, -7000, -8000, -9000};

    private int currentLevel = 0;

    public boolean levelPressed = false;

    public double slidesOffset = 0;

    Thread waitThread;

    HardwareThread hardware;

    @Override
    public void runOpMode() throws InterruptedException {

        config = new SampleConfiguration();
        hardware = new HardwareThread(hardwareMap, config);
        hardware.start();

        double lastHeading = 0, ingesterSpeed = 0;

        config.imu.gettingInput = true;

        sleep(1000);

        config.slides.setPower(0.4);
        while(!isStopRequested() && config.limit.get()[0] == 0) {}
        config.slides.setPower(0);

        slidesOffset = config.slides.get()[1];

        waitForStart();

        Sequence waitMillis = new Sequence(() -> {
            sleep(140);
            turning = false;
        }, null);
        waitThread = new Thread(waitMillis);

        while(!isStopRequested()) {

            config.ingest.setPower(ingesterSpeed);
            config.spinner.setPower(gamepad2.left_stick_y);
            config.preIngest.setPower(ingesterSpeed);

            if(gamepad1.y) config.flipdown.set(FLIPDOWN);

            if(gamepad1.a) ingesterSpeed = 1;
            else if(gamepad1.b) ingesterSpeed = -1;
            else if(gamepad1.x) ingesterSpeed = 0;

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

            //config.slides.setPower(gamepad2.left_stick_y);
            if((gamepad1.dpad_up || gamepad2.dpad_up) && !levelPressed) {
                currentLevel += currentLevel < 6 ? 1 : 0;
                levelPressed = true;
            }
            else if((gamepad1.dpad_down || gamepad2.dpad_down) && !levelPressed) {
                currentLevel -= currentLevel > 0 ? 1 : 0;
                levelPressed = true;
            }
            else if(!gamepad1.dpad_down && !gamepad1.dpad_up) levelPressed = false;

            double tempPos = config.slides.get()[1] - slidesOffset;

            int pow = tempPos > levels[currentLevel] ? -1 : 1;

            if(Math.abs(tempPos - levels[currentLevel]) < 150 || (pow == 1 && config.limit.get()[0] == 1)) pow = 0;

            config.slides.setPower(pow);

            if(turning) {
                lastHeading = imuHeading;
                power = 0;
            }
            if(power < 0.02) power = 0;

            if(gamepad1.dpad_right) config.dropper.set(OPEN);
            else if(gamepad1.dpad_left) config.dropper.set(CLOSE);

            double speed = gamepad1.left_bumper ? 0.4 : 1;

            setPower(-speed * gamepad1.left_stick_x, -speed * gamepad1.left_stick_y, speed * (gamepad1.right_stick_x + power));

            telemetry.addData("Heading: ", imuHeading);
            telemetry.addData("Sped: ", config.ingest.get()[0]);
            telemetry.addData("Slide Height: ", tempPos);
            telemetry.addData("Limit: ", config.limit.get()[0]);
            telemetry.addData("Expected Height: ", levels[currentLevel]);
            telemetry.addData("Level: ", currentLevel);
            telemetry.addData("Servo Position:" , config.dropper.get());
            telemetry.update();
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