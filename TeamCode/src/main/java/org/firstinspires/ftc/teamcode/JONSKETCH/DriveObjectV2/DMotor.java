package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants.getMotorVelocityF;

public class DMotor implements Active, DcMotor {

    DcMotorImplEx motor;
    private double[] pid = new double[]{30.0, 0.0, 0.0, 2700};
    private int partNum;
    private double powerToVelocity = 2700; //MODIFY WITH THE EXACT VALUE

    private DThread thread = new NullThread();
    private ValueStorage vals;

    public DMotor(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum) {
        motor = hwMap.get(DcMotorImplEx.class, objectName);
        motor.setMode(RunMode.RUN_USING_ENCODER);
        this.partNum = partNum;
        this.vals = vals;
        //Goal: use super and hwMap.get() in a way that doesn't
    }

    //Constructors



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

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    public void setPower(double power) {
        set(power * powerToVelocity);
    }

    @Override
    public double getPower() {
        return 0;
    }

    //Assumes set to 0 at the end
    public DThread setForTime(double velocity, double seconds) {
        if(thread != null && thread.isAlive()) thread.Stop();
        thread = new TimeThread(velocity, seconds, this);
        thread.start();
        return thread;
    }

    public DThread setForTime(double velocity, double endVelocity, double seconds) {
        if(thread != null && thread.isAlive()) thread.Stop();
        thread = new TimeThread(velocity, endVelocity, seconds, this);
        thread.start();
        return thread;
    }

    public DThread setPosition(int position, double relativeSpeed, double tolerance) {
        //Trying out never overriding threads (aka forcing use of endThreads() when need to replace active thread)
        if(thread != null && thread.isAlive()) return null;
        thread = new PositionThread(position, relativeSpeed, tolerance, this, vals);
        thread.start();
        return thread;
    }

    public DThread groupSetPosition(int position, double relativeSpeed, double tolerance, DMotor... motors) {
        if(thread.isAlive()) thread.Stop();
        thread = new PositionThread(position, relativeSpeed, tolerance, motors, vals);
        thread.start();
        return thread;
    }

    //RUN_TO_POSITION not supported, use setTargetPosition instead.
    public void setMode(DcMotor.RunMode mode){
        if(mode != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        //Should never be called
    }

    @Deprecated
    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Deprecated
    @Override
    public void setTargetPosition(int position) {
        //Leaving this deprecated, use setPosition instead.

        //Default values below
        setPosition(position, 1, 10);
    }

    @Deprecated
    @Override
    public int getTargetPosition() {
        return -1;
    }

    //isBusy checks for alive threads and non-zero velocity, THIS IS NOT THE SAME AS NORMAL ISBUSY.
    @Override
    public boolean isBusy() {
        return (thread.isAlive() || get()[0] != 0);
    }

    @Override
    public int getCurrentPosition() {
        return (int) get()[1];
    }

    public double[] getPID() {
        return pid;
    }

    public void setPID(double... pid) {
        if(pid.length != 4) return; //Potentially change
        this.pid = pid;
    }

    public void setInternalPID(double... pid) {
        motor.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                pid[0], pid[1], pid[2], pid[3]
        ));
    }

    public void reverse(boolean reverse) {
        motor.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        //Just gonna assume this is either not used or used at the end of an opmode.
        motor.close();
    }
}
