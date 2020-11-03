package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class shooterTestConfig {

    HardwareMap hwMap;

    public Servo servo;
    public DcMotorImplEx motor;

    public HardwareMap Configure(HardwareMap ahwMap) {

        hwMap = ahwMap;

        motor = hwMap.get(DcMotorImplEx.class,"motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hwMap.get(Servo.class, "servo");

        return hwMap;
    }
}
