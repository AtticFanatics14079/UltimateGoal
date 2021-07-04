package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Encoder;

public class DOdometryPod implements Sensor, DriveObject {

    private Encoder odoPod;
    private int partNum;
    private double ticksPerInch = 1000; //MODIFY WITH THE EXACT VALUE

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateHardware = true;

    public DOdometryPod(HardwareMap hwMap, String objectName) {
        DcMotorImplEx d = hwMap.get(DcMotorImplEx.class, objectName);
        d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoPod = new Encoder(d);
        this.partNum = hardware.size();
        hardware.add(this);
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    public void getHardware() {
        hardwareVals = new double[]{odoPod.getCurrentPosition(), odoPod.getCorrectedVelocity()};
        updateHardware = !updateHardware;
    }

    public void endThreads() {
        //Do nothing
    }

    public void reverse(boolean reverse) {
        odoPod.setDirection(reverse ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
    }
}
