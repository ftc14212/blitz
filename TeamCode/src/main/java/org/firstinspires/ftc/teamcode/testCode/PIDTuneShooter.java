package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config("PID Tune Shooter")
@Autonomous(name="PID Tune Shooter", group="test_ftc14212")
public class PIDTuneShooter extends OpMode {
    private CachingDcMotorEx shooter;
    private PIDController controller;
    public static double P = 8;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    public static double TARGET = 0;
    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        // set the PID values
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        shooter = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter"));
        // turn on the motors without the built in controller
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        // Get current positions
        double velocity = shooter.getVelocity();
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        F = TARGET;
        // Calculate PID
        double pid = controller.calculate(velocity, TARGET);
        double ff = F;
        double rawPower = pid + ff;
        // Apply power
        shooter.setVelocity(rawPower); // leader
        // telemetry for debugging
        telemetry.addData("PIDF", "P: " + P + " I: " + I + " D: " + D + " F: " + F);
        telemetry.addData("target", TARGET);
        telemetry.addData("velocity", velocity);
        telemetry.addData("shooterPower", rawPower);
        telemetry.addData("error", Math.abs(TARGET - velocity));
        telemetry.update();
    }
}