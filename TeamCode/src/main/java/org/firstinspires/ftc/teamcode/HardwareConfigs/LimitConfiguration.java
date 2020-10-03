package org.firstinspires.ftc.teamcode.HardwareConfigs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
public class LimitConfiguration {

    HardwareMap hwMap;

    public Servo servo;
    private DigitalChannel limit;
    private DigitalChannel limit2;

    public HardwareMap Configure(HardwareMap ahwMap) {

        hwMap = ahwMap;

        limit = hwMap.get(DigitalChannel.class, "limit");
        limit2 = hwMap.get(DigitalChannel.class, "limit2");
        servo = hwMap.get(Servo.class, "servo");

        return hwMap;
    }

    public boolean isPressed1() {
        return limit.getState();
    }

    public boolean isPressed2() {
        return limit2.getState();
    }
}
