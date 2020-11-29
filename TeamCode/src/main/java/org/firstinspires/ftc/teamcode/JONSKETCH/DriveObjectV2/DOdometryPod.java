package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Encoder;

public class DOdometryPod implements Sensor {

    private Encoder odoPod;
    private int partNum;
    private double ticksPerInch = 1000; //MODIFY WITH THE EXACT VALUE

    private ValueStorage vals;

    public DOdometryPod(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum) {
        odoPod = new Encoder(hwMap.get(DcMotorImplEx.class, objectName));
        this.vals = vals;
        this.partNum = partNum;
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    public double[] getHardware() {
        return new double[]{odoPod.getCurrentPosition(), odoPod.getCorrectedVelocity()};
    }

    public void endThreads() {
        //Do nothing
    }
}
