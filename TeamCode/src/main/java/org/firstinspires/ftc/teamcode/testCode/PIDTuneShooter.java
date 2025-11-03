package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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
    private SimpleMotorFeedforward feedforward;
    public static double TARGET = 0;


    public static double ks = 10;
    public static double kv = 20;

    /**
     * Initialization code.
     **/
    @Override
    public void init() {
        feedforward = new SimpleMotorFeedforward(ks, kv);
        // hardware
        shooter = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter"));
        // reverse motor
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
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
        // Get current velocity
        double velocity = shooter.getVelocity();
        double outPutVelo = -shooter.getVelocity();

        double newPID = feedforward.calculate(ks, kv);



        // Apply power
        shooter.setVelocity(-Math.max(-1000000, Math.min(100000, TARGET + newPID))); // leader
        // telemetry for debugging
        telemetry.addData("target", TARGET);
        telemetry.addData("velocity", outPutVelo);
        telemetry.addData("shooterPowerRAW", newPID);
        telemetry.addData("shooterPower", Math.max(-1, Math.min(1, newPID)));
        telemetry.addData("error", (TARGET - outPutVelo));
        telemetry.update();
    }
}