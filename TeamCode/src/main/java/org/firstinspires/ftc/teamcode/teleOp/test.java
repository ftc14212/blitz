package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class test extends OpMode {

    private Servo LED;

    public static double LEDPos = 0.7;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LED = hardwareMap.get(Servo.class, "led");

    }

    @Override
    public void loop() {

        LED.setPosition(LEDPos);

        telemetry.addData("LED", LEDPos);
        telemetry.update();
    }
}
