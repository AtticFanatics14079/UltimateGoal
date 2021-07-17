package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DEncoderlessMotor implements Active, DcMotor {

    DcMotorImpl motor;
    private int partNum;

    private DThread thread = new NullThread();

    //Array holding all the hardware inputs.
    private double[] hardwareVals;

    //This variable is here to make sure that hardwareVals is visible to every thread.
    private volatile boolean updateHardware = true;

    //Value that the motor is set to
    protected volatile double runVal = 0;

    public DEncoderlessMotor(HardwareMap hwMap, String objectName) {
        motor = hwMap.get(DcMotorImpl.class, objectName);
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        this.partNum = hardware.size();
        hardware.add(this);
        //Goal: use super and hwMap.get() in a way that doesn't
    }

    //Constructors



    //Interface methods

    public void set(double power) {
        runVal = power;
    }

    public int getPartNum() {
        return partNum;
    }

    public double[] get() {
        return hardwareVals;
    }

    public void setHardware() {
        motor.setPower(runVal);
    }

    public double getRunVal() {
        return runVal;
    }

    public void getHardware() {
        hardwareVals = new double[]{motor.getPower()};

        updateHardware = !updateHardware;
    }

    public void endThreads() {
        thread.Stop();
    }

    //Class-specific methods

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    public void setPower(double power) {
        set(power);
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

    //RUN_TO_POSITION not supported, use setTargetPosition instead.
    public void setMode(RunMode mode){
        if(mode != RunMode.RUN_TO_POSITION) motor.setMode(mode);
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

    public void setZeroPowerBehavior(ZeroPowerBehavior behavior){
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
        return null;
    }

    public void setPID(double... pid) {
        if(pid.length != 4) return; //Potentially change
    }

    public void reverse(boolean reverse) {
        motor.setDirection(reverse ? Direction.REVERSE : Direction.FORWARD);
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
