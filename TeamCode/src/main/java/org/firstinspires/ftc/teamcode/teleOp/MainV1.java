/***
 * MAIN V1
 * @author David Grieas - 14212 MetroBotics
 * coding for qualifier 2 - dec 6th
 * started coding at 11/1/25  @  6:15 pm
***/
package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.testCode.PIDTuneShooter;
import org.firstinspires.ftc.teamcode.testCode.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.utils.Variables;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

@Configurable
@TeleOp(name="Main v1", group=".ftc14212")
public class MainV1 extends LinearOpMode {
    /**
     * MAIN V1 BY DAVID
     * @author David Grieas - 14212 MetroBotics
    **/
    // positions
    public static double pivotCpos = 0.45;
    public static double hoodCpos = 1;
    public static double indexerCpos = 0;
    public static double ledCpos = 0.611;
    public static double turretTpos = 0;
    public static double shooterVelo = 0;
    // presets
    public static double blueShooter = 143;
    public static double redShooter = -49;
    // misc
    private double wheelSpeed = wheelSpeedMax;
    private boolean tReset = false;
    private boolean tReset2 = false;
    public static boolean shooterOn = true;
    public static int shooterT = 3500;
    // timers
    ElapsedTime loopTime;
    ElapsedTime tResetT;
    // odometry
    public static boolean odoDrive = true;
    // config stuff
    public static boolean redSide = false;
    public static boolean debugMode = true;
    public static double wheelSpeedMax = 1;
    public static double turretOffset = 0;
    public static boolean turretOn = true;
    @Override
    public void runOpMode() {
        // hardware
        PIDController shooterPID = new PIDController(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        PIDController turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        Follower follower = Constants.createFollower(hardwareMap);
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
        CachingDcMotorEx shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        CachingDcMotorEx shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        CachingDcMotorEx intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake")); // 1150 rpm --> 460 rpm
        CachingDcMotorEx indexer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        // servos
        CachingServo pivot = new CachingServo(hardwareMap.get(Servo.class, "intakePivot")); // 1x axon max
        CachingServo hood = new CachingServo(hardwareMap.get(Servo.class, "hood")); // 1x axon mini
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon max
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon max
        CachingServo led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        CombinedCRServo turret = new CombinedCRServo(turret1, turret2); // 2x axon maxs
        // limits
        pivot.scaleRange(0, 0.4);
        // reset encoders
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // turn on motor
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.45);
        led.setPosition(ledCpos = 0.611);
        follower.setStartingPose(Variables.lastAutoPos == null ? new Pose(56.5, 8.3, Math.toRadians(90)) : new Pose(Variables.lastAutoPos.getX(), Variables.lastAutoPos.getY(), Variables.lastAutoPos.getHeading()));
        Variables.lastAutoPos = null;
        // misc
        loopTime = new ElapsedTime();
        tResetT = new ElapsedTime();
        follower.update();
        boolean indexerOn = true;
        // reset
        loopTime.reset();
        tResetT.reset();
        // telemetry
        telemetryM.addLine("BLITZ Team 14212!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // poses
                Pose bluePos = new Pose(34.9, 121.9, Math.toRadians(blueShooter));
                Pose redPos = new Pose(124.9, -30.5, Math.toRadians(redShooter));
                Pose target = redSide ? redPos : bluePos;
                // variables
                telemetryM.setDebug(debugMode);
                turretPID.setPID(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
                double turretCpos = (intake.getCurrentPosition() / (PIDTuneTurret.TPR * PIDTuneTurret.ratio)) * 360;
                double turretOffsetXY = Math.atan(target.getY()/follower.getPose().getX());
                double turretOffset = (Math.toDegrees(follower.getHeading()) - (redSide ? Math.toDegrees(redPos.getHeading()) : Math.toDegrees(bluePos.getHeading()))) + turretOffsetXY;
                double distShooter = redSide ? Math.sqrt(Math.pow((redPos.getX() - follower.getPose().getX()), 2) + Math.pow((redPos.getY() - follower.getPose().getY()), 2)) : Math.sqrt(Math.pow((bluePos.getX() - follower.getPose().getX()), 2) + Math.pow((bluePos.getY() - follower.getPose().getY()), 2));
                // status
                boolean INTAKE = gamepad1.left_trigger > 0.1;
                boolean OUTTAKE = gamepad1.right_trigger > 0.1;
                boolean FEED = gamepad1.right_bumper;
                boolean ALIGN_SHOOT = gamepad1.left_bumper;
                boolean RESET_SHOOTER_TURRET = !currentGamepad1.left_bumper && previousGamepad1.left_bumper;
                boolean RESET_INTAKE = (!currentGamepad1.right_bumper && previousGamepad1.right_bumper) || gamepad1.dpadRightWasReleased() || (!(currentGamepad1.right_trigger > 0.1) && previousGamepad1.right_trigger > 0.1) || (!(currentGamepad1.left_trigger > 0.1) && previousGamepad1.left_trigger > 0.1);
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
                if ((currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b)) turretOn = !turretOn;
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
                    follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                }
                // controls
                if (INTAKE) {
                    pivotCpos = 0.75;
                    if (indexerOn) indexerCpos = 1;
                    intake.setPower(1);
                    shooterVelo = -75;
                    if (!indexerOn && shooterR.getCurrent(CurrentUnit.MILLIAMPS) <= shooterT) indexerCpos = 0;
                }
                if (INTAKE && shooterR.getCurrent(CurrentUnit.MILLIAMPS) >= shooterT && !ALIGN_SHOOT) {
                    indexerOn = false;
                    indexerCpos = -0.9;
                }
                if (OUTTAKE) {
                    indexerOn = true;
                    pivotCpos = 0.75;
                    indexerCpos = -1;
                    intake.setPower(-1);
                }
                if (FEED) {
                    indexerOn = true;
                    pivotCpos = 0.45;
                    indexerCpos = 1;
                    intake.setPower(1);
                }
                if (ALIGN_SHOOT) {
                    if (shooterR.getVelocity() >= shooterVelo && shooterR.getVelocity() <= shooterVelo + 80) ledCpos = 1;
                    else ledCpos = 0.388;
                    // turretTpos = turretOffset;
                    turretTpos = turretOn ? wrap(
                            alignTurret(
                                    follower.getPose().getX(),
                                    follower.getPose().getY(),
                                    Math.toDegrees(follower.getHeading()),
                                    target
                            )
                    ) : 0;
                    shooterVelo = getShooterVelo(distShooter);
                    hoodCpos = getHoodCpos(distShooter);
                    indexerOn = true;
                } else if (RESET_SHOOTER_TURRET) {
                    tReset = true;
                    tResetT.reset();
                    turretTpos = 0;
                    ledCpos = 0.611;
                    shooterVelo = 0;
                    hoodCpos = 0;
                }
                if (RESET_INTAKE) {
                    pivotCpos = 0.45;
                    indexerCpos = 0;
                    intake.setPower(0);
                    shooterVelo = 0;
                }
                // shooter code
                double sPower = shooterOn ? shooterPID.calculate(shooterR.getVelocity(), shooterVelo) + shooterVelo : 0;
                shooterR.setVelocity(sPower); // leader
                shooterL.setVelocity(sPower); // follower
                // turret code
                double error = turretTpos - turretCpos;
                double power = -turretPID.calculate(0, error) + PIDTuneTurret.F;
                power = Math.max(-1, Math.min(1, power));
                turret.setPower(power);
                follower.update();
                // telemetry
                telemetryM.addLine("BLITZ Team 14212!");
                telemetryM.addData(true, "pivot", pivot.getPosition());
                telemetryM.addData(true, "hood", hood.getPosition());
                telemetryM.addData(true, "indexer", indexer.getPower());
                telemetryM.addData(true, "led", led.getPosition());
                telemetryM.addData(true, "turret", turretCpos);
                telemetryM.addData(true, "turretOffset", turretOffset);
                telemetryM.addData(true, "bluePos", Math.toDegrees(bluePos.getHeading()));
                telemetryM.addData(true, "redPos", Math.toDegrees(redPos.getHeading()));
                telemetryM.addData(true, "turretPower", turret.getPower());
                telemetryM.addData(true, "tReset", tReset);
                telemetryM.addData(true, "tReset2", tReset2);
                telemetryM.addData(true,"turret error", Math.abs(turretTpos - turretCpos));
                telemetryM.addData(true, "PIDF", "P: " + PIDTuneTurret.P + " I: " + PIDTuneTurret.I + " D: " + PIDTuneTurret.D + " F: " + PIDTuneTurret.F);
                telemetryM.addData(true, "turretTpos", turretTpos);
                telemetryM.addData(true, "shooterR Current", shooterR.getCurrent(CurrentUnit.MILLIAMPS));
                telemetryM.addData(true, "indexerOn", indexerOn);
                telemetryM.addData(true, "distShooter", distShooter);
                telemetryM.addData(true, "getShooterVelo", getShooterVelo(distShooter));
                telemetryM.addData(true, "getHoodCpos", getHoodCpos(distShooter));
                telemetryM.addData(true, "redSide", redSide);
                telemetryM.addData(true, "\nFollower:\nX:", follower.getPose().getX());
                telemetryM.addData(true, "Y:", follower.getPose().getY());
                telemetryM.addData(true, "heading:", Math.toDegrees(follower.getHeading()));
                telemetryM.addData(true, "\nredPos:\nX:", redPos.getX());
                telemetryM.addData(true, "Y:", redPos.getY());
                telemetryM.addData(true, "heading:", Math.toDegrees(redPos.getHeading()));
                telemetryM.addData(true, "\nbluePos:\nX:", bluePos.getX());
                telemetryM.addData(true, "Y:", bluePos.getY());
                telemetryM.addData(true, "heading:", Math.toDegrees(bluePos.getHeading()));
                telemetryM.addData(true, "\nturret offset XY", turretOffsetXY);
                telemetryM.addData(true, "alignTurret", alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), target));
                telemetryM.update();
                loopTime.reset();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
            LynxUtils.setLynxColor(0, 255, 0);
        }
    }
    public double getShooterVelo(double distShooter) {
        InterpLUT lut = new InterpLUT();
        // add the data
        lut.add(15, 900);
        lut.add(25, 920);
        lut.add(35, 940);
        lut.add(45, 960);
        lut.add(55, 985);
        lut.add(65, 1030);
        lut.add(75, 1100);
        lut.add(85, 1150);
        lut.add(125, 1320);
        // finish
        lut.createLUT();
        return lut.get(Math.max(15.1, Math.min(124.9, distShooter)));
    }
    public double getHoodCpos(double distShooter) {
        InterpLUT lut = new InterpLUT();
        // add the data
        lut.add(15, 0.0);
        lut.add(25, 0.0);
        lut.add(35, 0.1);
        lut.add(45, 0.2);
        lut.add(55, 0.32);
        lut.add(65, 0.2);
        lut.add(75, 0.3);
        lut.add(85, 0.35);
        lut.add(125, 0.45);
        // finish
        lut.createLUT();
        return lut.get(Math.max(15.1, Math.min(124.9, distShooter)));
    }
    public double alignTurret(double x, double y, double headingDeg, Pose target) {
        double dx = target.getX() - x;
        double dy = target.getY() - y;
        // angle from robot to target
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // turret angle = angle to goal minus robot heading
        double turretAngle = angleToGoal - headingDeg;
        return turretAngle - turretOffset;
    }
    // wrap code
    public double wrap(double angle) {
        if (angle > 190) angle -= 360;
        if (angle < -210) angle += 360;
        return angle;
    }
}
