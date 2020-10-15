package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareConfigs.UltimateGoalConfig;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ValueStorage;

@TeleOp
public class UltimateGoalTeleOpV1 extends LinearOpMode {

    UltimateGoalConfig config;
    ValueStorage vals = new ValueStorage();
    HardwareThread hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            config = new UltimateGoalConfig();
            config.Configure(hardwareMap, vals);
            hardware = new HardwareThread(hardwareMap, vals, config);
            waitForStart();
            hardware.start();
            if(!isStopRequested()) {
                while(opModeIsActive()){
                    vals.waitForCycle();
                    getInput();
                }
            }
        } catch(Exception e){

        } finally{
            hardware.Stop();
        }

    }

    public void getInput(){
        //Main loop
        setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(gamepad1.a) config.shooter.setPower(1);
    }

    public void setPower(double x, double y, double a){
        config.backLeft.setPower(-x + y + a);
        config.frontLeft.setPower(x + y + a);
        config.frontRight.setPower(-x + y - a);
        config.backRight.setPower(x + y - a);
    }
}
