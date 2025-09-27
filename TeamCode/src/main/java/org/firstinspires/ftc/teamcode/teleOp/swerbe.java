package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@Config("swerbe")
@TeleOp(name = "swerbe", group = "test_ftc14212")
public class swerbe extends LinearOpMode {
    boolean debugMode = true;
    private AnalogInput rfA;
    private AnalogInput lfA;
    private AnalogInput rrA;
    private AnalogInput lrA;
    @Override
    public void runOpMode() {
        // hardware
        rfA = hardwareMap.get(AnalogInput.class, "rf");
        lfA = hardwareMap.get(AnalogInput.class, "lf");
        rrA = hardwareMap.get(AnalogInput.class, "rr");
        lrA = hardwareMap.get(AnalogInput.class, "lr");
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        // motors
        CachingDcMotorEx leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        CachingDcMotorEx leftRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear"));
        CachingDcMotorEx rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        CachingDcMotorEx rightRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear"));
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        hardwareMap.get(IMU.class, "imu").resetYaw();
        // telemetry
        telemetryM.addLine("Metrobotics Team 14212 SWERVEEE!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // get the voltage of our analog line
                // divide by 3.3 (the max voltage) to get a value between 0 and 1
                // multiply by 360 to convert it to 0 to 360 degrees
                double rf = rfA.getVoltage() / 3.3 * 360;
                double lf = lfA.getVoltage() / 3.3 * 360;
                double rr = rrA.getVoltage() / 3.3 * 360;
                double lr = lrA.getVoltage() / 3.3 * 360;
                telemetryM.addData("right front", rf);
                telemetryM.addData("left front", lf);
                telemetryM.addData("right rear", rr);
                telemetryM.addData("left rear", lr);
                telemetryM.update();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
        }
    }
}
