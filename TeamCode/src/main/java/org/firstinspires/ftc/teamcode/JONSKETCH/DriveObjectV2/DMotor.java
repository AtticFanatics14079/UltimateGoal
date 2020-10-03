package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DMotor implements DriveObject {

    private DcMotorImplEx motor;
    private double[] pid = new double[]{30.0, 0.0, 0.0, 2700};
    private int partNum;
    private double powerToVelocity = 2700; //MODIFY WITH THE EXACT VALUE

    private DOThread thread = new NullThread();
    private ValueStorage vals;

    //Constructors

    public DMotor(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum){
        motor = hwMap.get(DcMotorImplEx.class, objectName);
        this.partNum = partNum;
        this.vals = vals;
    }

    //Interface methods

    public void set(double velocity) {
        vals.runValues(true, velocity, partNum);
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    public void setHardware(double velocity) {
        motor.setVelocity(velocity);
    }

    public double[] getHardware() {
        return new double[]{motor.getVelocity(), (double) motor.getCurrentPosition()};
    }

    public void endThreads() {
        thread.Stop();
    }

    //Class-specific methods

    public void setPower(double power) {
        set(power * powerToVelocity);
    }

    //Assumes set to 0 at the end
    public DOThread setForTime(double velocity, double seconds) {
        if(thread != null && thread.isAlive()) thread.Stop();
        thread = new TimeThread(velocity, seconds, this);
        thread.start();
        return thread;
    }

    public DOThread setForTime(double velocity, double endVelocity, double seconds) {
        if(thread != null && thread.isAlive()) thread.Stop();
        thread = new TimeThread(velocity, endVelocity, seconds, this);
        thread.start();
        return thread;
    }

    public DOThread setPosition(int position, double relativeSpeed, double tolerance) {
        //Trying out never overriding threads (aka forcing use of endThreads() when need to replace active thread)
        if(thread != null && thread.isAlive()) return null;
        thread = new PositionThread(position, relativeSpeed, tolerance, this, vals);
        thread.start();
        return thread;
    }

    public DOThread groupSetPosition(int position, double relativeSpeed, double tolerance, DMotor... motors) {
        if(thread.isAlive()) thread.Stop();
        thread = new PositionThread(position, relativeSpeed, tolerance, motors, vals);
        thread.start();
        return thread;
    }

    //RUN_TO_POSITION not supported, use setTargetPosition instead.
    public void setMode(DcMotor.RunMode mode){
        if(mode != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    public double[] getPID() {
        return pid;
    }

    public void setPID(double... pid) {
        if(pid.length != 4) return; //Potentially change
        this.pid = pid;
    }

    public void reverse(boolean reverse) {
        motor.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

}
