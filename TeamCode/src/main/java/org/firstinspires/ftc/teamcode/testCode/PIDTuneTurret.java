package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config("PID Tune Turret")
@Autonomous(name="PID Tune Turret", group="test_ftc14212")
public class PIDTuneTurret extends OpMode {
    private CachingDcMotorEx turret;
    // private AnalogInput elc;
    private PIDController controller;
    public static double P = 0.008;
    public static double I = 0;
    public static double D = 0.00275;
    public static double F = 0;
    public static double TARGET = 0;
    public static double TPR = 4000; // ticks per revolution
    public static double ratio = (double) 92 / 53;
    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        // set the PID values
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        turret = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "turret"));
        // elc = hardwareMap.get(AnalogInput.class, "elc");
        // reverse motors
        // turret.setDirection(DcMotorEx.Direction.REVERSE);
        // reset encoders
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // turn on the motors without the built in controller
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // combine both FTCDashboard and the regular telemetry
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        // telemetry
        telemetry.addLine("Use this to tune the turret.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     **/
    @Override
    public void loop() {
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        // Get current positions

        double turretOR = (TPR * ratio);
        double turretCpos = (turret.getCurrentPosition() / turretOR) * 360;
        // Calculate PID
        double pid = controller.calculate(turretCpos, TARGET);
        double ff = F;
        double rawPower = pid + ff;
        // Apply power
        turret.setPower(-Math.max(-1, Math.min(1, rawPower))); // leader
        // telemetry for debugging
        telemetry.addData("PIDF", "P: " + P + " I: " + I + " D: " + D + " F: " + F);
        telemetry.addData("target", TARGET);
        telemetry.addData("turretCpos", turretCpos);
        telemetry.addData("turretPowerRAW", rawPower);
        telemetry.addData("turretPower", Math.max(-1, Math.min(1, rawPower)));
        telemetry.addData("error", Math.abs(TARGET - turretCpos));
        telemetry.update();
    }
}