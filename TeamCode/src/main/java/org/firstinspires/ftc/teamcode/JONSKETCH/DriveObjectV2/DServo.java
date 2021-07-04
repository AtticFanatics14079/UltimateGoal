package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DServo implements Active {

    private Servo servo;
    private int partNum;

    private DThread thread = new NullThread();

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateHardware = true;

    //Value that the motor is set to
    private volatile double runVal = 0;

    //Constructors

    public DServo(HardwareMap hwMap, String objectName){
        servo = hwMap.get(Servo.class, objectName);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    //Interface methods

    public void set(double position) {
        runVal = position;
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    public void setHardware() {
        servo.setPosition(runVal);
    }

    public void getHardware() {
        hardwareVals = new double[]{servo.getPosition()};
        updateHardware = !updateHardware;
    }

    public void endThreads() {
        thread.Stop();
    }
}
