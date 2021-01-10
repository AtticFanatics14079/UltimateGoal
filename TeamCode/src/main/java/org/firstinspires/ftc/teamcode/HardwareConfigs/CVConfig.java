package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CVConfig {

    HardwareMap hwMap;

    public DcMotorImplEx backLeft;
    public DcMotorImplEx backRight;
    public DcMotorImplEx frontLeft;
    public DcMotorImplEx frontRight;

    public Servo wiperLeft, wiperRight;

    public DcMotorImplEx sweep;

    public HardwareMap Configure(HardwareMap ahwMap) {

        hwMap = ahwMap;

        backLeft = hwMap.get(DcMotorImplEx.class, "back_left_motor");
        backRight = hwMap.get(DcMotorImplEx.class, "back_right_motor");
        frontLeft = hwMap.get(DcMotorImplEx.class, "front_left_motor");
        frontRight = hwMap.get(DcMotorImplEx.class, "front_right_motor");

        sweep = hwMap.get(DcMotorImplEx.class, "sweep");

        wiperRight = hwMap.get(Servo.class, "wiperRight");
        wiperLeft = hwMap.get(Servo.class, "wiperLeft");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        return hwMap;
    }

}
