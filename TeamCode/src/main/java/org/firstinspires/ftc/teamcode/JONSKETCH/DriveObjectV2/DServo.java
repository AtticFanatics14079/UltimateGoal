package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DServo implements Active {

    private Servo servo;
    private int partNum;

    private DThread thread = new NullThread();

    //Constructors

    public DServo(HardwareMap hwMap, String objectName){
        servo = hwMap.get(Servo.class, objectName);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    //Interface methods

    public void set(double position) {
        vals.runValues(true, position, partNum);
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    public void setHardware(double position) {
        servo.setPosition(position);
    }

    public double[] getHardware() {
        return new double[]{servo.getPosition()};
    }

    public void endThreads() {
        thread.Stop();
    }
}
