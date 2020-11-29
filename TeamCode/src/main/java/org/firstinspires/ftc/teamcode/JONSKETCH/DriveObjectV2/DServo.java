package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DServo implements Active {

    private Servo servo;
    private int partNum;

    private DThread thread = new NullThread();
    private ValueStorage vals;

    //Constructors

    public DServo(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum){
        servo = hwMap.get(Servo.class, objectName);
        this.partNum = partNum;
        this.vals = vals;
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
