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

    public DAnalogSensor(ValueStorage vals, HardwareMap hwMap, String objectName, int partNum, InterpretVoltage method) {
        sensor = hwMap.get(AnalogInput.class, objectName);
        this.partNum = partNum;
        this.vals = vals;
        interpret = method;
    }

    public interface InterpretVoltage {
        //Does something with the voltage
        double interpret(double voltage);
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
        return new double[]{interpret.interpret(input), input, sensor.getMaxVoltage()};
    }

    @Override
    public void endThreads() {
        //Do nothing
    }
}
