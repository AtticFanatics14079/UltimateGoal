package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class shooterTestConfig {

    HardwareMap hwMap;

    public Servo loader;
    public DcMotorImplEx shooter;

    public HardwareMap Configure(HardwareMap ahwMap) {

        hwMap = ahwMap;

        shooter = hwMap.get(DcMotorImplEx.class,"motor");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        loader = hwMap.get(Servo.class, "servo");

        return hwMap;
    }
}
