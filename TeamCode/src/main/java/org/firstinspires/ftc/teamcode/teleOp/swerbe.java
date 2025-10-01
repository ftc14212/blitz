/***
 * SWERBE
 * @author David Grieas - 14212 MetroBotics
 * coded from scratch
 * started code at 9/29/25  @  11:08 am
 * finished recoding at 9/29/25 @ 3:42 pm
***/
package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.testCode.PIDTuneServo;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@Config("swerbe")
@TeleOp(name = "swerbe", group = "test_ftc14212")
public class swerbe extends LinearOpMode {
    /**
     * SWERBE BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     * @TODO: fix diagonal to work properly
     * @TODO: make rotation work when strafing
     * @TODO: make it more efficient
    **/
    boolean debugMode = true;
    // PID controller
    public static PIDController pid = new PIDController(Math.sqrt(PIDTuneServo.P), PIDTuneServo.I, PIDTuneServo.D);
    public static double mSpeed = 1;
    // pod offsets
    public static double rfOffset = 160;
    public static double lfOffset = 170;
    public static double rrOffset = 120;
    public static double lrOffset = 170;
    // wheel angles
    double wa1 = 180;   // RF straight
    double wa2 = 180; // RR reversed
    double wa3 = 180;   // LF straight
    double wa4 = 180; // LR reversed
    // angles forward
    double lfF = 180;
    double rfF = 180;
    double lrF = 180;
    double rrF = 180;
    // angles strafe
    double lfS = 85;
    double rfS = 92;
    double lrS = 88;
    double rrS = 90;
    @Override
    public void runOpMode() {
        // hardware
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        // motors
        CachingDcMotorEx leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        CachingDcMotorEx leftRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear"));
        CachingDcMotorEx rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        CachingDcMotorEx rightRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear"));
        // servos
        CachingCRServo lf = new CachingCRServo(hardwareMap.get(CRServo.class, "lf"));
        CachingCRServo rf = new CachingCRServo(hardwareMap.get(CRServo.class, "rf"));
        CachingCRServo lr = new CachingCRServo(hardwareMap.get(CRServo.class, "lr"));
        CachingCRServo rr = new CachingCRServo(hardwareMap.get(CRServo.class, "rr"));
        // analogs (steering encoders)
        AnalogInput rfA = hardwareMap.get(AnalogInput.class, "rfA");
        AnalogInput lfA = hardwareMap.get(AnalogInput.class, "lfA");
        AnalogInput rrA = hardwareMap.get(AnalogInput.class, "rrA");
        AnalogInput lrA = hardwareMap.get(AnalogInput.class, "lrA");
        // reverse motors
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        // reset PID
        pid.reset();
        // turn on servos
        lf.setPower(0);
        // telemetry
        telemetryM.addLine("Metrobotics Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryM.setDebug(debugMode);
                // raw encoder value
                double rfRaw = (rfA.getVoltage() / 3.3) * 360;
                double lfRaw = (lfA.getVoltage() / 3.3) * 360;
                double rrRaw = (rrA.getVoltage() / 3.3) * 360;
                double lrRaw = (lrA.getVoltage() / 3.3) * 360;
                // encoder + offsets
                double rfE = (rfRaw - rfOffset + 360) % 360;
                double lfE = (lfRaw - lfOffset + 360) % 360;
                double rrE = (rrRaw - rrOffset + 360) % 360;
                double lrE = (lrRaw - lrOffset + 360) % 360;
                // rotation pid
                rf.setPower(Math.abs(angleError(wa1, rfE)) > 2 ? -pid.calculate(rfE, wa1) : 0);
                rr.setPower(Math.abs(angleError(wa2, rrE)) > 2 ? -pid.calculate(rrE, wa2) : 0);
                lf.setPower(Math.abs(angleError(wa3, lfE)) > 2 ? -pid.calculate(lfE, wa3) : 0);
                lr.setPower(Math.abs(angleError(wa4, lrE)) > 2 ? -pid.calculate(lrE, wa4) : 0);
                // steering math stuff
                double servoSmooth = 0.2; // smoothing factor 0 = no movement, 1 = instant
                wa1 += angleError(lerp(rfF, rfS, joystickMix(gamepad1.left_stick_y, gamepad1.left_stick_x)), wa1) * servoSmooth;
                wa2 += angleError(lerp(rrF, rrS, joystickMix(gamepad1.left_stick_y, gamepad1.left_stick_x)), wa2) * servoSmooth;
                wa3 += angleError(lerp(lfF, lfS, joystickMix(gamepad1.left_stick_y, gamepad1.left_stick_x)), wa3) * servoSmooth;
                wa4 += angleError(lerp(lrF, lrS, joystickMix(gamepad1.left_stick_y, gamepad1.left_stick_x)), wa4) * servoSmooth;
                // drive
                double forward = -gamepad1.left_stick_y; // forward/back
                double strafe  = gamepad1.left_stick_x;  // left/right
                double rotate = gamepad1.right_stick_x;  // rotate
                // motor powers
                leftFront.setPower(forward - strafe + rotate);
                rightFront.setPower(forward - strafe - rotate);
                leftRear.setPower(forward - strafe + rotate);
                rightRear.setPower(forward - strafe - rotate);
                // scale powers if any exceeds 1
                double maxPower = Math.max(Math.max(Math.abs(leftFront.getPower()), Math.abs(rightFront.getPower())), Math.max(Math.abs(leftRear.getPower()), Math.abs(rightRear.getPower())));
                if (maxPower > 1.0) {
                    leftFront.setPower(leftFront.getPower() / maxPower * mSpeed);
                    rightFront.setPower(rightFront.getPower() / maxPower * mSpeed);
                    leftRear.setPower(leftRear.getPower() / maxPower * mSpeed);
                    rightRear.setPower(rightRear.getPower() / maxPower * mSpeed);
                } else {
                    leftFront.setPower(leftFront.getPower() * mSpeed);
                    rightFront.setPower(rightFront.getPower() * mSpeed);
                    leftRear.setPower(leftRear.getPower() * mSpeed);
                    rightRear.setPower(rightRear.getPower() * mSpeed);
                }
                // telemetry
                telemetryM.addLine("Metrobotics Team 14212!");
                telemetryM.addLine(true, "RAW Encoder Angles:");
                telemetryM.addData(true, "RF Raw", rfRaw);
                telemetryM.addData(true, "LF Raw", lfRaw);
                telemetryM.addData(true, "RR Raw", rrRaw);
                telemetryM.addData(true, "LR Raw", lrRaw);
                telemetryM.addLine(true, "\nOFFSET Encoder Angles:");
                telemetryM.addData(true, "RF OFF", rfE);
                telemetryM.addData(true, "LF OFF", lfE);
                telemetryM.addData(true, "RR OFF", rrE);
                telemetryM.addData(true, "LR OFF", lrE);
                telemetryM.addLine(true, "\nTarget Angles:");
                telemetryM.addData(true, "RF Target", wa1);
                telemetryM.addData(true, "RR Target", wa2);
                telemetryM.addData(true, "LF Target", wa3);
                telemetryM.addData(true, "LR Target", wa4);
                telemetryM.addLine(true, "\nErrors (deg):");
                telemetryM.addData(true, "RF error", angleError(wa1, rfE));
                telemetryM.addData(true, "RR error", angleError(wa2, rrE));
                telemetryM.addData(true, "LF error", angleError(wa3, lfE));
                telemetryM.addData(true, "LR error", angleError(wa4, lrE));
                telemetryM.addLine(true, "\nMotor powers:");
                telemetryM.addData(true, "RF Power", rightFront.getPower());
                telemetryM.addData(true, "LF Power", leftFront.getPower());
                telemetryM.addData(true, "RR Power", rightRear.getPower());
                telemetryM.addData(true, "LR Power", leftRear.getPower());
                telemetryM.update();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
        }
    }
    // functions
    /**
     * finds shortest signed angle difference between target and current
    **/
    private double angleError(double target, double current) {
        double error = target - current;
        error = (error + 180) % 360 - 180;
        return error;
    }
    /**
     * take 2 pos and combine em
    **/
    public static double lerp(double min, double max, double value) {
        // clamp value between 0 and 1
        value = Math.max(0, Math.min(1, value));
        return min + (max - min) * value;
    }
    /**
     * take 2 values and mix em
    **/
    public static double joystickMix(double forwardValue, double strafeValue) {
        double f = Math.abs(forwardValue);
        double s = Math.abs(strafeValue);
        if (f + s == 0) return 0;
        return s / (f + s);
    }

}
