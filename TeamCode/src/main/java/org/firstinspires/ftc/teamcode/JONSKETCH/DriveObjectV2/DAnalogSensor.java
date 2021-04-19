package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DAnalogSensor implements Sensor, AnalogSensor {

    private AnalogInput sensor;
    private int partNum;

    private InterpretVoltage interpret;

    private ValueStorage vals;

    private double maxVolt;

    public DAnalogSensor(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum, InterpretVoltage method) {
        sensor = hwMap.get(AnalogInput.class, objectName);
        this.partNum = partNum;
        this.vals = vals;
        interpret = method;
        maxVolt = getMaxVoltage();
    }

    public DAnalogSensor(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum, double maxVoltage, InterpretVoltage method) {
        sensor = hwMap.get(AnalogInput.class, objectName);
        this.partNum = partNum;
        this.vals = vals;
        interpret = method;
        maxVolt = maxVoltage;
    }

    public interface InterpretVoltage {
        //Does something with the voltage
        double interpret(double voltage, double maxVoltage);
    }

    @Override
    public double readRawVoltage() {
        return sensor.getVoltage();
    }

    public double getMaxVoltage() {
        return sensor.getMaxVoltage();
    }

    @Override
    public int getPartNum() {
        return partNum;
    }

    @Override
    public double[] get() {
        return vals.hardware(false, null, partNum);
    }

    @Override
    public double[] getHardware() {
        double input = sensor.getVoltage();
        return new double[]{interpret.interpret(input, maxVolt), input, sensor.getMaxVoltage()};
    }

    @Override
    public void endThreads() {
        //Do nothing
    }
}
