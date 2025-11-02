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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Config
@Configurable
@TeleOp(name="Main v1", group=".ftc23403")
public class MainV1 extends LinearOpMode {
    /**
     * MAIN V6 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     **/
    // servos
    public static double pivotCpos = 0;
    public static double hoodCpos = 1;
    public static double indexerCpos = 0;
    public static double ledCpos = 0.611;
    // misc
    private double wheelSpeed = wheelSpeedMax;
    // timers
    ElapsedTime loopTime;
    // odometry
    public static boolean odoDrive = false;
    // config stuff
    public static boolean redSide = true;
    public static boolean debugMode = true;
    public static double wheelSpeedMax = 1;
    // heading lock
    @Override
    public void runOpMode() {
        // hardware
        PIDController shooterPID = new PIDController(Math.sqrt(0), 0, 0);
        PIDController turretPID = new PIDController(0.7, 0, 0.05);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);

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
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0);
        hardwareMap.get(IMU.class, "imu").resetYaw();
        // misc
        loopTime = new ElapsedTime();
        // reset
        loopTime.reset();
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            // follower.startTeleopDrive();
            while (opModeIsActive()) {
                // variables
                telemetryM.setDebug(debugMode);
                boolean moving = Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0;
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
                    /*
                    follower.setMaxPower(wheelSpeed);
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, headingLock ? (Math.toDegrees(headingError) > 2 && Math.toDegrees(headingError) < 50 ? headingCalc : 0) : -gamepad1.right_stick_x, true);
                    follower.update();
                    */
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
                telemetryM.addData(true, "pivotCpos", pivotCpos);
                telemetryM.addData(true, "hoodCpos", hoodCpos);
                telemetryM.addData(true, "indexerCpos", indexerCpos);
                telemetryM.addData(true, "ledCpos", ledCpos);
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
