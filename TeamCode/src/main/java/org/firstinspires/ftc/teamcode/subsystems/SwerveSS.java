/***
 * SwerveSS
 * @author David Grieas - 14212 MetroBotics
 * Swerve susbsystem
 * started coding at 12/2/25  @  7:15 pm
 * finished coding at 12/2/25  @  11:27 pm
***/
package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class SwerveSS extends BlocksOpModeCompanion {

    // -------------------------
    // VALUES STUFF
    // -------------------------
    public static double steerP = 0.00002;
    public static double steerI = 0.0;
    public static double steerD = 0.000000004;

    public static double steerOutputScale = 1.0;
    public static double servoSmooth = 0.25;
    public static double mSpeed = 1.0;

    public static double L = 10.0;
    public static double W = 10.0;

    public static double rfOffset = 188.0;
    public static double lfOffset = 135.0;
    public static double rrOffset = 115.0;
    public static double lrOffset = 55.0;

    public static int rfDriveSign = 1;
    public static int lfDriveSign = 1;
    public static int rrDriveSign = 1;
    public static int lrDriveSign = 1;

    public static int rfSteerSign = 1;
    public static int lfSteerSign = 1;
    public static int rrSteerSign = 1;
    public static int lrSteerSign = 1;

    // -------------------------
    // FLIPPIN HARDWARE STUFF
    // -------------------------
    private final CachingDcMotorEx leftFront, leftRear, rightFront, rightRear;
    private final CachingCRServo lfServo, rfServo, lrServo, rrServo;
    private final AnalogInput rfA, lfA, rrA, lrA;

    // -------------------------
    // PID CONTROLLERS
    // -------------------------
    private final PIDController pidRF = new PIDController(Math.sqrt(steerP), steerI, steerD);
    private final PIDController pidLF = new PIDController(Math.sqrt(steerP), steerI, steerD);
    private final PIDController pidRR = new PIDController(Math.sqrt(steerP), steerI, steerD);
    private final PIDController pidLR = new PIDController(Math.sqrt(steerP), steerI, steerD);

    // Smoothed angle targets
    private double targetAngleRF = 0;
    private double targetAngleLF = 0;
    private double targetAngleRR = 0;
    private double targetAngleLR = 0;

    // -------------------------
    // X-LOCK SYSTEM
    // -------------------------
    private boolean xLockEnabled = false;

    // -------------------------
    // random okay leave me alone :C
    // -------------------------
    private double lastRotCommand = 0;
    private static final double HEADING_KP = 0.015;   // adjust if oscillation
    private static final double HEADING_TOLERANCE = 2.0; // degrees
    double pidOutLF;
    double pidOutRF;
    double pidOutLR;
    double pidOutRR;

    // -------------------------
    // INIT THINGY
    // -------------------------
    public SwerveSS(HardwareMap hardwareMap) {
        leftFront  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        leftRear   = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        rightRear  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear"));

        lfServo = new CachingCRServo(hardwareMap.get(CRServo.class, "lf"));
        rfServo = new CachingCRServo(hardwareMap.get(CRServo.class, "rf"));
        lrServo = new CachingCRServo(hardwareMap.get(CRServo.class, "lr"));
        rrServo = new CachingCRServo(hardwareMap.get(CRServo.class, "rr"));

        rfA = hardwareMap.get(AnalogInput.class, "rfA");
        lfA = hardwareMap.get(AnalogInput.class, "lfA");
        rrA = hardwareMap.get(AnalogInput.class, "rrA");
        lrA = hardwareMap.get(AnalogInput.class, "lrA");

        // match your original setup
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // -------------------------------------------------------------
    // METHODSSSS
    // -------------------------------------------------------------
    public void drive(double forward, double strafe, double rotate) {
        if (xLockEnabled) {
            // any input? disable X lock
            if (Math.abs(forward) > 0.01 || Math.abs(strafe) > 0.01 || Math.abs(rotate) > 0.01) {
                disableXLock();
            } else {
                return; // stay locked
            }
        }

        doSwerve(forward, strafe, rotate);
    }

    public void setXLock() {
        xLockEnabled = true;

        // classic X-lock angles
        targetAngleLF = 45;
        targetAngleRF = 315;
        targetAngleLR = 315;
        targetAngleRR = 45;

        // zero drive motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // immediately drive servos to target using PID
        updateSteeringServos();
    }


    public void disableXLock() {
        xLockEnabled = false;
    }

    public boolean isXLocked() {
        return xLockEnabled;
    }

    // -------------------------------------------------------------
    // FULL SWERVE MATH STUFF AND LIKE LOGIC
    // -------------------------------------------------------------
    private void doSwerve(double forward, double strafe, double rotate) {
        double rawRfDeg = analogToDeg(rfA.getVoltage());
        double rawLfDeg = analogToDeg(lfA.getVoltage());
        double rawRrDeg = analogToDeg(rrA.getVoltage());
        double rawLrDeg = analogToDeg(lrA.getVoltage());

        double measuredRF = wrapDeg(rawRfDeg - rfOffset);
        double measuredLF = wrapDeg(rawLfDeg - lfOffset);
        double measuredRR = wrapDeg(rawRrDeg - rrOffset);
        double measuredLR = wrapDeg(rawLrDeg - lrOffset);

        forward = applyDeadband(forward, 0.03);
        strafe  = applyDeadband(strafe, 0.03);
        rotate  = applyDeadband(rotate, 0.03);

        double R = Math.hypot(L, W);

        double A = strafe - rotate * (L / R);
        double B = strafe + rotate * (L / R);
        double C = forward - rotate * (W / R);
        double D = forward + rotate * (W / R);

        double speedLF = Math.hypot(B, D);
        double speedRF = Math.hypot(B, C);
        double speedLR = Math.hypot(A, D);
        double speedRR = Math.hypot(A, C);

        double angleLF = wrapDeg(Math.toDegrees(Math.atan2(B, D)));
        double angleRF = wrapDeg(Math.toDegrees(Math.atan2(B, C)));
        double angleLR = wrapDeg(Math.toDegrees(Math.atan2(A, D)));
        double angleRR = wrapDeg(Math.toDegrees(Math.atan2(A, C)));

        boolean zero = (Math.abs(forward) < 1e-6 && Math.abs(strafe) < 1e-6 && Math.abs(rotate) < 1e-6);

        double max = Math.max(Math.max(speedLF, speedRF), Math.max(speedLR, speedRR));
        if (max > 1.0) {
            speedLF /= max; speedRF /= max; speedLR /= max; speedRR /= max;
        }

        speedLF *= mSpeed; speedRF *= mSpeed; speedLR *= mSpeed; speedRR *= mSpeed;

        SwerveModuleState optLF = optimize(angleLF, speedLF, measuredLF);
        SwerveModuleState optRF = optimize(angleRF, speedRF, measuredRF);
        SwerveModuleState optLR = optimize(angleLR, speedLR, measuredLR);
        SwerveModuleState optRR = optimize(angleRR, speedRR, measuredRR);

        targetAngleLF = smoothAngleTowards(targetAngleLF, optLF.angle, servoSmooth);
        targetAngleRF = smoothAngleTowards(targetAngleRF, optRF.angle, servoSmooth);
        targetAngleLR = smoothAngleTowards(targetAngleLR, optLR.angle, servoSmooth);
        targetAngleRR = smoothAngleTowards(targetAngleRR, optRR.angle, servoSmooth);

        pidOutLF = pidLF.calculate(measuredLF, targetAngleLF) * steerOutputScale * lfSteerSign;
        pidOutRF = pidRF.calculate(measuredRF, targetAngleRF) * steerOutputScale * rfSteerSign;
        pidOutLR = pidLR.calculate(measuredLR, targetAngleLR) * steerOutputScale * lrSteerSign;
        pidOutRR = pidRR.calculate(measuredRR, targetAngleRR) * steerOutputScale * rrSteerSign;

        lfServo.setPower(clamp(pidOutLF, -1, 1));
        rfServo.setPower(clamp(pidOutRF, -1, 1));
        lrServo.setPower(clamp(pidOutLR, -1, 1));
        rrServo.setPower(clamp(pidOutRR, -1, 1));

        leftFront.setPower(zero ? 0.0 : optLF.speed * lfDriveSign);
        rightFront.setPower(zero ? 0.0 : optRF.speed * rfDriveSign);
        leftRear.setPower(zero ? 0.0 : optLR.speed * lrDriveSign);
        rightRear.setPower(zero ? 0.0 : optRR.speed * rrDriveSign);

        if (isXLocked()) updateSteeringServos(); // keep wheels locked in X
    }

    // -------------------------
    // HEADING ALIGNMENT DAWG
    // -------------------------
    public boolean autoAlign(double targetHeadingDeg, double currentHeadingDeg) {

        if (xLockEnabled) return true; // can't auto align in X-lock

        // compute shortest-path heading error
        double error = angleError(targetHeadingDeg, currentHeadingDeg);

        // proportional rotation command
        double rot = error * HEADING_KP;
        rot = clamp(rot, -1, 1);
        lastRotCommand = rot;

        // tell existing swerve math to rotate
        doSwerve(0, 0, rot);

        return Math.abs(error) < HEADING_TOLERANCE;
    }


    // -------------------------
    // TELEMETRY FOR DEBUGGING MY AHH
    // -------------------------
    public String telemetry() {

        double rfDeg = analogToDeg(rfA.getVoltage()) - rfOffset;
        double lfDeg = analogToDeg(lfA.getVoltage()) - lfOffset;
        double rrDeg = analogToDeg(rrA.getVoltage()) - rrOffset;
        double lrDeg = analogToDeg(lrA.getVoltage()) - lrOffset;
        return("===== Swerve Telemetry =====\n" +
                "X-Lock: " + xLockEnabled + "\n" +
                "-- Measured Angles --\n" +
                "RF Measured: " + wrapDeg(rfDeg) + "\n" +
                "LF Measured: " + wrapDeg(lfDeg) + "\n" +
                "RR Measured: " + wrapDeg(rrDeg) + "\n" +
                "LR Measured: " + wrapDeg(lrDeg) + "\n" +
                "-- Target Angles --\n" +
                "RF Target: " + wrapDeg(targetAngleRF) + "\n" +
                "LF Target: " + wrapDeg(targetAngleLF) + "\n" +
                "RR Target: " + wrapDeg(targetAngleRR) + "\n" +
                "LR Target: " + wrapDeg(targetAngleLR) + "\n" +
                "-- Steering PID Outputs --\n" +
                "RF PID: " + pidOutRF + "\n" +
                "LF PID: " + pidOutLF + "\n" +
                "RR PID: " + pidOutRR + "\n" +
                "LR PID: " + pidOutLR + "\n" +
                "-- Raw Encoders (volt) --\n" +
                "RF Volt: " + rfA.getVoltage() + "\n" +
                "LF Volt: " + lfA.getVoltage() + "\n" +
                "RR Volt: " + rrA.getVoltage() + "\n" +
                "LR Volt: " + lrA.getVoltage() + "\n" +
                "-- Auto Align --\n" +
                "Last rot cmd: " + lastRotCommand + "\n");
    }

    // HELPER METHOD
    private void updateSteeringServos() {
        double measuredRF = wrapDeg(analogToDeg(rfA.getVoltage()) - rfOffset);
        double measuredLF = wrapDeg(analogToDeg(lfA.getVoltage()) - lfOffset);
        double measuredRR = wrapDeg(analogToDeg(rrA.getVoltage()) - rrOffset);
        double measuredLR = wrapDeg(analogToDeg(lrA.getVoltage()) - lrOffset);

        double pidOutLF = pidLF.calculate(measuredLF, targetAngleLF) * steerOutputScale * lfSteerSign;
        double pidOutRF = pidRF.calculate(measuredRF, targetAngleRF) * steerOutputScale * rfSteerSign;
        double pidOutLR = pidLR.calculate(measuredLR, targetAngleLR) * steerOutputScale * lrSteerSign;
        double pidOutRR = pidRR.calculate(measuredRR, targetAngleRR) * steerOutputScale * rrSteerSign;

        lfServo.setPower(clamp(pidOutLF, -1, 1));
        rfServo.setPower(clamp(pidOutRF, -1, 1));
        lrServo.setPower(clamp(pidOutLR, -1, 1));
        rrServo.setPower(clamp(pidOutRR, -1, 1));
    }


    // ----------------------------------------------------------------------
    // MATH FUNCTIONS BROOO
    // ----------------------------------------------------------------------
    private static class SwerveModuleState {
        double angle, speed;
        SwerveModuleState(double a, double s) { angle = a; speed = s; }
    }
    private SwerveModuleState optimize(double desiredAngle, double desiredSpeed, double currentAngle) {
        desiredAngle = wrapDeg(desiredAngle);
        currentAngle = wrapDeg(currentAngle);

        double delta = angleError(desiredAngle, currentAngle);
        if (Math.abs(delta) > 90.0) {
            desiredAngle = wrapDeg(desiredAngle + 180.0);
            desiredSpeed = -desiredSpeed;
        }
        return new SwerveModuleState(desiredAngle, desiredSpeed);
    }

    private static double analogToDeg(double voltage) {
        voltage = Math.max(0.0, Math.min(voltage, 3.3));
        return (voltage / 3.3) * 360.0;
    }
    private static double wrapDeg(double deg) {
        deg %= 360.0; if (deg < 0) deg += 360.0; return deg;
    }
    private static double angleError(double target, double current) {
        double err = target - current;
        return (err + 180) % 360 - 180;
    }
    private static double smoothAngleTowards(double current, double target, double s) {
        current = wrapDeg(current);
        target = wrapDeg(target);
        double err = angleError(target, current);
        return wrapDeg(current + err * Math.max(0, Math.min(1, s)));
    }
    private static double applyDeadband(double v, double db) {
        return Math.abs(v) < db ? 0 : v;
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
