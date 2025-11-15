package org.firstinspires.ftc.teamcode.testCode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@Autonomous(name="PID Tune Shooter", group="test_ftc14212")
public class PIDTuneShooter extends OpMode {
    private CachingDcMotorEx shooterR;
    private CachingDcMotorEx shooterL;
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
        shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR"));
        shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL"));
        // reverse motor
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        // turn on the motors without the built in controller
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // combine both FTCDashboard and the regular telemetry
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
        double velocityR = shooterR.getVelocity();
        double velocityL = shooterL.getVelocity();
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        F = TARGET;
        // Calculate PID only on one motor (leader)
        double pid = controller.calculate(velocityR, TARGET);
        double ff = F;
        double rawPower = pid + ff;
        // Apply power
        shooterR.setVelocity(rawPower); // leader
        shooterL.setVelocity(rawPower); // follower
        // telemetry for debugging
        telemetry.addData("PIDF", "P: " + P + " I: " + I + " D: " + D + " F: " + F);
        telemetry.addData("target", TARGET);
        telemetry.addData("velocityR", velocityR);
        telemetry.addData("velocityL", velocityL);
        telemetry.addData("rawVelocity", rawPower);
        telemetry.addData("errorR", Math.abs(TARGET - velocityR));
        telemetry.addData("errorL", Math.abs(TARGET - velocityL));
        telemetry.addData("errorAvg", (Math.abs(TARGET - velocityR) + Math.abs(TARGET - velocityL)) / 2);
        telemetry.update();
    }
}