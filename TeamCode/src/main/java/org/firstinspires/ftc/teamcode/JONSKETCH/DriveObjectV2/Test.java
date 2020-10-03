package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {

    ValueStorage vals = new ValueStorage();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleConfiguration config = new SampleConfiguration();
        HardwareThread hardware = new HardwareThread(hardwareMap, vals, config);
        hardware.start();
        //System.out.println(hardware.config.hardware.get(0).getPID()[0] + " hf");
        Sequence s = new Sequence(() -> config.backLeft.groupSetPosition(3000, 1, 10, config.backLeft, config.backRight, config.frontLeft, config.frontRight), null);
        Sequence s2 = new Sequence(() -> config.backLeft.groupSetPosition(0, 1, 10, config.backLeft, config.backRight, config.frontLeft, config.frontRight), s);
        Thread t = new Thread(s2);
        /*Sequence t1 = new Sequence(() -> {
            hardware.config.hardware.get(1).setPower(-100, 3);
            Thread a = hardware.config.hardware.get(0).setPower(100, 2);
            System.out.println(a.isAlive());
            return a;
        }, null);
        Sequence t2 = new Sequence(() -> hardware.config.hardware.get(2).setPower(100, 2), t1);
        Thread timeThread = new Thread(t2);

         */
        waitForStart();
        //timeThread.start();
        t.start();
        while (!isStopRequested()){
            //telemetry.addData("M1 Position", hardware.config.hardware.get(0).get());
            //telemetry.addData("T1 Alive", timeThread.isAlive());
            //telemetry.update();
        }
        hardware.Stop();
    }
}
