package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CombinedServo {
    private final List<Servo> servos = new ArrayList<>();

    public CombinedServo(Servo... servo) {
        this.servos.addAll(Arrays.asList(servo));
    }

    public ServoController getController(int index) {
        return servos.get(index).getController();
    }

    public int getPortNumber(int index) {
        return servos.get(index).getPortNumber();
    }

    public void setDirection(Servo.Direction direction, int index) {
        servos.get(index).setDirection(direction);
    }

    public Servo.Direction getDirection(int index) {
        return servos.get(index).getDirection();
    }

    public void setPosition(double position) {
        for (Servo servo : servos) {
            servo.setPosition(position);
        }
    }

    public void scaleRange(double min, double max) {
        for (Servo servo : servos) {
            servo.scaleRange(min, max);
        }
    }

    public double getPosition() {
        return servos.get(0).getPosition();
    }

    public HardwareDevice.Manufacturer getManufacturer(int index) {
        return servos.get(index).getManufacturer();
    }

    public String getDeviceName(int index) {
        return servos.get(index).getDeviceName();
    }

    public String getConnectionInfo(int index) {
        return servos.get(index).getConnectionInfo();
    }

    public int getVersion(int index) {
        return servos.get(index).getVersion();
    }

    public void close(int index) {
        servos.get(index).close();
    }
}