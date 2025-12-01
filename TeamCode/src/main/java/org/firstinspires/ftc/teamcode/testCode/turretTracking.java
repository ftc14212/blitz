package org.firstinspires.ftc.teamcode.testCode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name = "turretTracking", group = "test_ftc14212")
public class turretTracking extends LinearOpMode {
    boolean debugMode = true;
    public static boolean redSide = false;
    public static double blueShooter = 143;
    public static double redShooter = 42;
    public static double turretTpos = 0;
    public static boolean turretOn = false;
    public static boolean turretOffset = false;
    public static double offset = 12;
    @Override
    public void runOpMode() {
        // hardware
        Follower follower = Constants.createFollower(hardwareMap);
        PIDController turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        // motors
        CachingDcMotorEx intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake")); // 1150 rpm --> 460 rpm
        // servos
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon max
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon max
        CachingServo led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        CombinedCRServo turret = new CombinedCRServo(turret1, turret2); // 2x axon maxs
        // reset encoders
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // turn on motor
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        led.setPosition(redSide ? 0.277 : 0.611);
        // start pos
        follower.setStartingPose(new Pose(56.5, 8.3, Math.toRadians(90)));
        follower.update();
        // gamepads
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        // telemetry
        telemetryM.addLine("Metrobotics Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                follower.update();
                led.setPosition(turretOn ? redSide ? 0.3 : 0.611 : 1);
                // gamepad stuff
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
                // poses
                Pose bluePos = new Pose(34.9, 121.9, Math.toRadians(blueShooter));
                Pose redPos = new Pose(87.9, 136.5, Math.toRadians(redShooter));
                Pose target = redSide ? redPos : bluePos;
                // drive
                follower.setMaxPower(1);
                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                // vars
                double turretCpos = (intake.getCurrentPosition() / (PIDTuneTurret.TPR * PIDTuneTurret.ratio)) * 360;
                // turret control
                if ((currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b)) turretOn = !turretOn;
                // field side
                if ((currentGamepad1.share && !previousGamepad1.share) || (currentGamepad2.share && !previousGamepad2.share)) redSide = !redSide;
                // turret control
                if (turretOn) {
                    turretTpos = wrap(
                            alignTurret(
                                    follower.getPose().getX(),
                                    follower.getPose().getY(),
                                    Math.toDegrees(follower.getHeading()),
                                    target
                            )
                    );
                } else {
                    turretTpos = 0;
                }
                // turret code
                double error = turretTpos - turretCpos;
                double power = -turretPID.calculate(0, error) + PIDTuneTurret.F;
                power = Math.max(-1, Math.min(1, power));
                turret.setPower(power);
                // telemetry
                telemetryM.addData(true, "turretOn", turretOn);
                telemetryM.addData(true, "redSide", redSide);
                telemetryM.addData(true, "turretTpos", turretTpos);
                telemetryM.addData(true, "turretCpos", turretCpos);
                telemetryM.addData(true, "turretError", error);
                telemetryM.addData(true, "turretPower", power);
                telemetryM.addData(true, "turretOffset", turretOffset);
                telemetryM.addData(true, "\nFollower:\nX:", follower.getPose().getX());
                telemetryM.addData(true, "Y:", follower.getPose().getY());
                telemetryM.addData(true, "heading:", Math.toDegrees(follower.getHeading()));
                telemetryM.addData(true, "\nredPos:\nX:", redPos.getX());
                telemetryM.addData(true, "Y:", redPos.getY());
                telemetryM.addData(true, "heading:", Math.toDegrees(redPos.getHeading()));
                telemetryM.addData(true, "\nbluePos:\nX:", bluePos.getX());
                telemetryM.addData(true, "Y:", bluePos.getY());
                telemetryM.addData(true, "heading:", Math.toDegrees(bluePos.getHeading()));
                telemetryM.update();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
        }
    }
    // updated turret math code
    public double alignTurret(double x, double y, double headingDeg, Pose target) {
        double dx = target.getX() - x;
        double dy = target.getY() - y;
        // angle from robot to target
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // turret angle = angle to goal minus robot heading
        double turretAngle = angleToGoal - headingDeg;
        return turretAngle - (turretOffset ? offset : 0);
    }
    // wrap code
    public double wrap(double angle) {
        if (angle > 190) angle -= 360;
        if (angle < -210) angle += 360;
        return angle;
    }

}
