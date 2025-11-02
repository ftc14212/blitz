package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CombinedCRServo {
    private final List<CRServo> servos = new ArrayList<>();

    public CombinedCRServo(CRServo... servo) {
        this.servos.addAll(Arrays.asList(servo));
    }

    public ServoController getController(int index) {
        return servos.get(index).getController();
    }

    public int getPortNumber(int index) {
        return servos.get(index).getPortNumber();
    }

    public void setDirection(DcMotor.Direction direction, int index) {
        servos.get(index).setDirection(direction);
    }

    public DcMotor.Direction getDirection(int index) {
        return servos.get(index).getDirection();
    }

    public void setPower(double power) {
        for (CRServo servo : servos) {
            servo.setPower(power);
        }
    }

    public double getPower() {
        return servos.get(0).getPower();
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