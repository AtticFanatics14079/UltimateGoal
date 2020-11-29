package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DDigitalSensor implements Sensor, DigitalChannel {

    private DigitalChannel sensor;
    private int partNum;

    //Maybe add a thread to stop when a condition is met
    //private DOThread thread = new NullThread();
    private ValueStorage vals;

    //Constructors

    public DDigitalSensor(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum) {
        sensor = hwMap.get(DigitalChannel.class, objectName);
        this.partNum = partNum;
        this.vals = vals;
    }

    //Interface methods

    public int getPartNum() {
        return partNum;
    }

    public void set(double value) {
        //Do nothing
    }

    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    public void setHardware(double value) {
        //Do nothing
    }

    //Binarizes boolean
    public double[] getHardware() { return new double[]{sensor.getState() ? 1 : 0}; }

    public void endThreads() {
        //thread.Stop();
    }

    @Override
    public Mode getMode() {
        return null;
    }

    @Override
    public void setMode(Mode mode) {

    }

    @Override
    public boolean getState() {
        return false;
    }

    @Override
    public void setState(boolean state) {

    }

    @Override
    public void setMode(DigitalChannelController.Mode mode) {

    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
