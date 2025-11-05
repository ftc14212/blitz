/***
 * MAIN V1
 * @author David Grieas - 14212 MetroBotics
 * coding for qualifier 1 - nov 16
 * started coding at 11/1/25  @  6:15 pm
***/
package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.testCode.PIDTuneServo;
import org.firstinspires.ftc.teamcode.testCode.PIDTuneShooter;
import org.firstinspires.ftc.teamcode.testCode.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@Configurable
@TeleOp(name="Main v1", group=".ftc14212")
public class MainV1 extends LinearOpMode {
    /**
     * MAIN V6 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     **/
    // positions
    public static double pivotCpos = 0;
    public static double hoodCpos = 1;
    public static double indexerCpos = 0;
    public static double ledCpos = 0.611;
    public static double turretTpos = 0;
    // presets
    public static double blueShooterH = 45;
    public static double redShooterH = 275;
    // misc
    private double wheelSpeed = wheelSpeedMax;
    // timers
    ElapsedTime loopTime;
    // odometry
    public static boolean odoDrive = false;
    // config stuff
    public static boolean redSide = false;
    public static boolean debugMode = true;
    public static double wheelSpeedMax = 1;
    // heading lock
    @Override
    public void runOpMode() {
        // hardware
        PIDController shooterPID = new PIDController(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        PIDController turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        Follower follower = Constants.createFollower(hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        // limelight3A.setPollRateHz(50);
        LynxUtils.initLynx(hardwareMap);
        // gamepads
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        // motors
        CachingDcMotorEx leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront")); // 312 rpm --> 468 rpm
        CachingDcMotorEx leftRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear")); // 312 rpm --> 468 rpm
        CachingDcMotorEx rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront")); // 312 rpm --> 468 rpm
        CachingDcMotorEx rightRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear")); // 312 rpm --> 468 rpm
        CachingDcMotorEx turret = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "turret")); // 223 rpm
        CachingDcMotorEx shooter = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooter")); // 6000 rpm --> ~9100 rpm
        CachingDcMotorEx intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"));
        // servos
        CachingServo pivot = new CachingServo(hardwareMap.get(Servo.class, "intakePivot")); // 1x axon max
        CachingServo hood = new CachingServo(hardwareMap.get(Servo.class, "hood")); // 1x axon mini
        CachingCRServo leftIndexer = new CachingCRServo(hardwareMap.get(CRServo.class, "leftIndexer")); // 1x axon mini
        CachingCRServo rightIndexer = new CachingCRServo(hardwareMap.get(CRServo.class, "rightIndexer")); // 1x axon mini
        CachingServo led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        CombinedCRServo indexer = new CombinedCRServo(leftIndexer, rightIndexer); // 2x axon minis
        // limits
        pivot.scaleRange(0, 0.4);
        // hood.scaleRange(0.1, 0.86);
        // reset encoders
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pinpoint.resetPosAndIMU();
        // turn on motor
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0);
        led.setPosition(ledCpos = 0.611);
        hardwareMap.get(IMU.class, "imu").resetYaw();
        // misc
        loopTime = new ElapsedTime();
        follower.update();
        // reset
        loopTime.reset();
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            //
            // follower.startTeleopDrive();
            while (opModeIsActive()) {
                // variables
                follower.update();
                telemetryM.setDebug(debugMode);
                boolean moving = Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0;
                double turretCpos = (turret.getCurrentPosition() / (PIDTuneTurret.TPR * PIDTuneTurret.ratio)) * 360;
                double turretOffset = Math.toDegrees(follower.getHeading()) - (redSide ? redShooterH : blueShooterH);
                // gamepad stuff
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
                // servos
                pivot.setPosition(pivotCpos);
                hood.setPosition(hoodCpos);
                indexer.setPower(indexerCpos);
                led.setPosition(ledCpos);
                // field side
                if ((currentGamepad1.share && !previousGamepad1.share) || (currentGamepad2.share && !previousGamepad2.share)) redSide = !redSide;
                // toggle debug
                if ((currentGamepad1.options && !previousGamepad1.options) || (currentGamepad2.options && !previousGamepad2.options)) debugMode = !debugMode;
                // movements
                if (!odoDrive) {
                    // drive
                    double forward = -gamepad1.left_stick_y; // forward
                    double strafe = gamepad1.left_stick_x; // strafe
                    double turn = gamepad1.right_stick_x;  // rotation
                    // formula
                    double leftFrontPower = (forward + strafe + turn) * wheelSpeed;
                    double leftBackPower = (forward - strafe + turn) * wheelSpeed;
                    double rightFrontPower = (forward - strafe - turn) * wheelSpeed;
                    double rightBackPower = (forward + strafe - turn) * wheelSpeed;
                    // power
                    leftFront.setPower(leftFrontPower);
                    leftRear.setPower(leftBackPower);
                    rightFront.setPower(rightFrontPower);
                    rightRear.setPower(rightBackPower);
                } else {
                    follower.setMaxPower(wheelSpeed);
                    // follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                    follower.update();
                }
                if (gamepad1.left_trigger > 0.1) {
                    pivotCpos = 0.75;
                    indexerCpos = 1;
                    intake.setPower(1);
                } else if (gamepad1.right_trigger > 0.1) {
                    pivotCpos = 0.75;
                    indexerCpos = -1;
                    intake.setPower(-1);
                } else if (gamepad1.dpad_left) {
                    pivotCpos = 0.45;
                    indexerCpos = 1;
                    intake.setPower(0);
                } else if (gamepad1.dpad_right) {
                    pivotCpos = 0.75;
                    indexerCpos = 0;
                    intake.setPower(1);
                } else {
                    pivotCpos = 0.45;
                    indexerCpos = 0;
                    intake.setPower(0);
                }

                if (gamepad1.left_bumper) {
                    ledCpos = 0.388;
                    turretTpos = turretOffset;
                } else if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
                    turretTpos = 0;
                    ledCpos = 0.611;
                }

                if (Math.abs(turretCpos - turretTpos) > 2) {
                    turret.setPower(-Math.max(-1, Math.min(1, turretPID.calculate(turretCpos, turretTpos) + PIDTuneTurret.F)));
                } else {
                    turret.setPower(0);
                }

                if (!moving && !odoDrive) {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }
                // telemetry
                telemetryM.addLine("BEASTKIT Team 23403!");
                telemetryM.addData(true, "pivot", pivot.getPosition());
                telemetryM.addData(true, "hood", hood.getPosition());
                telemetryM.addData(true, "indexer", indexer.getPower());
                telemetryM.addData(true, "led", led.getPosition());
                telemetryM.addData(true, "turret", turretCpos);
                telemetryM.addData(true, "turretOffset", turretOffset);
                telemetryM.addData(true, "heading", Math.toDegrees(follower.getHeading()));
                telemetryM.update();
                loopTime.reset();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
            LynxUtils.setLynxColor(0, 255, 0);
        }
    }
}
