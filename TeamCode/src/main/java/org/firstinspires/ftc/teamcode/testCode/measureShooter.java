package org.firstinspires.ftc.teamcode.testCode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "measureShooter", group = "test_ftc14212")
public class measureShooter extends LinearOpMode {
    boolean debugMode = true;
    public static int shooterVelo = 0;
    public static double hoodCpos = 0;
    @Override
    public void runOpMode() {
        // hardware
        PIDController shooterPID = new PIDController(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        // motors
        CachingDcMotorEx shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        CachingDcMotorEx shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        CachingDcMotorEx indexer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        // servos
        CachingServo hood = new CachingServo(hardwareMap.get(Servo.class, "hood")); // 1x axon mini
        // reverse
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        hardwareMap.get(IMU.class, "imu").resetYaw();
        // telemetry
        telemetryM.addLine("Metrobotics Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                indexer.setPower(1);
                hood.setPosition(hoodCpos);
                double sPower = shooterPID.calculate(shooterR.getVelocity(), shooterVelo) + shooterVelo;
                shooterR.setVelocity(sPower); // leader
                shooterL.setVelocity(sPower); // follower
                telemetryM.update();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
        }
    }
}
