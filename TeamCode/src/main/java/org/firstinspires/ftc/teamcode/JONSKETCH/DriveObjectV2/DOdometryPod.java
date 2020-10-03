package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DOdometryPod implements DriveObject {

    private DcMotor odoPod;
    private int partNum;
    private double ticksPerInch = 1000; //MODIFY WITH THE EXACT VALUE

    private ValueStorage vals;

    public DOdometryPod(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum) {
        odoPod = hwMap.get(DcMotor.class, objectName);
        this.vals = vals;
        this.partNum = partNum;
    }

    public void set(double value) {
        //Do nothing
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    public void setHardware(double value) {

    }

    public double[] getHardware() {
        System.out.println(odoPod.getCurrentPosition());
        return new double[]{odoPod.getCurrentPosition()};
    }

    public void endThreads() {
        //Do nothing
    }
}
