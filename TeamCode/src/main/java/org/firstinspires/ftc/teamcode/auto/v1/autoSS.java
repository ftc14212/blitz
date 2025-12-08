package org.firstinspires.ftc.teamcode.auto.v1;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
@Autonomous(name = "3 baller auto", group = ".ftc14212")
public class autoSS extends OpMode {
    TelemetryM telemetryM;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime timer;
    PIDController shooterPID;
    PIDController turretPID;
    // gamepads
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad1;
    Gamepad previousGamepad2;
    // motors
    CombinedCRServo turret; // 223 rpm
    CachingDcMotorEx shooterL; // 6000 rpm
    CachingDcMotorEx shooterR; // 6000 rpm
    CachingDcMotorEx intake; // 1150 rpm
    // servos
    private static CachingServo pivot; // 1x axon max
    private static CachingServo hood; // 1x axon mini
    private static CachingServo led; // 2x gobilda led lights RGB
    private static CachingDcMotorEx indexer; // 2x axon minis
    // positions
    public static double pivotCpos = 0.45;
    public static double hoodCpos = 1;
    public static double indexerCpos = 0;
    public static double ledCpos = 0.611;
    public static double turretTpos = 0;
    public static int shooterVelo = 0;
    private boolean tReset = false;
    private boolean tReset2 = false;
    public boolean SHOOTER_READY = false;
    public static boolean redSide = false;
    public static double blueShooter = 43;
    public static double redShooter = -49;
    public static boolean shooterOn = true;
    public static int intakeWait = 500;
    public static int shootWait = 1300;
    boolean ran2 = false;
    public static Pose target;
    public double distShooter;
    boolean ran = false;
    public static boolean indexerOn = true;
    public static int shooterT = 3500;
    private ElapsedTime tResetT;
    public static double speed = 1;
    public static boolean debugMode = true;
    private int pathState;
    public static final Pose startPose = new Pose(56.5, 8.39, Math.toRadians(90));
    public static final Pose parkPose = new Pose(56.5, 36, Math.toRadians(90));
    private PathChain park;
    /* preload lines */
    boolean parkStarted = false;

    public void buildPaths() {

        park = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        telemetryM = new TelemetryM(telemetry, debugMode);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        tResetT = new ElapsedTime();
        timer = new ElapsedTime();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        // hi
        shooterPID = new PIDController(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        // gamepads
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();
        // motors
        shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake")); // 1150 rpm
        indexer = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "indexer")); // 1150 rpm
        // servos
        pivot = new CachingServo(hardwareMap.get(Servo.class, "intakePivot")); // 1x axon max
        hood = new CachingServo(hardwareMap.get(Servo.class, "hood")); // 1x axon mini
        CachingCRServo turret1 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret1")); // 1x axon max
        CachingCRServo turret2 = new CachingCRServo(hardwareMap.get(CRServo.class, "turret2")); // 1x axon max
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        turret = new CombinedCRServo(turret1, turret2); // 2x axon maxs
        // directions
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        indexer.setDirection(DcMotorEx.Direction.REVERSE);
        // reset pos
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // limits
        pivot.scaleRange(0, 0.4);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.45);
        led.setPosition(ledCpos = 0.611);
        indexerCpos = 0;
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // telemetry
        telemetryM.addLine("BLITZ Team 14212!");
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!parkStarted) {
                    turretTpos = 24;
                    shooterVelo = 1280;
                    hoodCpos = 0.5;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (!shooterOn) {
                        RESET_SHOOTER_TURRET();
                        speed = 1;
                        follower.followPath(park, true);
                        parkStarted = true;
                    }
                    if (actionTimer.getElapsedTime() >= shootWait && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        follower.followPath(park, true);
                        parkStarted = true;
                    }
                }
                if (!follower.isBusy() && parkStarted) {
                    ran = false;
                    ran2 = false;
                    RESET_INTAKE();
                    setPathState(-1);
                }
                break;
        }
    }

    public void INTAKE() {
        pivotCpos = 0.75;
        indexerCpos = 1;
        intake.setPower(1);
        shooterVelo = -75;
        if (!indexerOn && shooterR.getCurrent(CurrentUnit.MILLIAMPS) <= shooterT) indexerCpos = 0;
        if (shooterR.getCurrent(CurrentUnit.MILLIAMPS) >= shooterT) {
            indexerOn = false;
            indexerCpos = -0.9;
        }
    }

    public void RESET_INTAKE() {
        pivotCpos = 0.45;
        indexerCpos = 0;
        intake.setPower(0);
        shooterVelo = 0;
        indexerOn = true;
    }
    public void FEED() {
        indexerOn = true;
        pivotCpos = 0.45;
        indexerCpos = 1;
        intake.setPower(1);
    }
    public void RESET_SHOOTER_TURRET() {
        tReset = true;
        tResetT.reset();
        turretTpos = 0;
        ledCpos = 0.611;
        shooterVelo = 0;
        hoodCpos = 0;
        indexerOn = true;
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        Pose bluePos = new Pose(114.9, 24.7, Math.toRadians(blueShooter));
        Pose redPos = new Pose(124.9, -62, Math.toRadians(redShooter));
        target = redSide ? redPos : bluePos;
        double turretCpos = (intake.getCurrentPosition() / (PIDTuneTurret.TPR * PIDTuneTurret.ratio)) * 360;
        double turretOffsetXY = Math.atan(target.getY()/follower.getPose().getX());
        double turretOffset = (Math.toDegrees(follower.getHeading()) - Math.toDegrees(target.getHeading())) + turretOffsetXY;
        distShooter = redSide ? Math.sqrt(Math.pow((redPos.getX() - follower.getPose().getX()), 2) + Math.pow((redPos.getY() - follower.getPose().getY()), 2)) : Math.sqrt(Math.pow((bluePos.getX() - follower.getPose().getX()), 2) + Math.pow((bluePos.getY() - follower.getPose().getY()), 2));
        SHOOTER_READY = shooterR.getVelocity() >= shooterVelo && shooterR.getVelocity() <= shooterVelo + 80;
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        follower.setMaxPower(speed);
        autonomousPathUpdate();
        // servos
        pivot.setPosition(pivotCpos);
        hood.setPosition(hoodCpos);
        indexer.setPower(indexerCpos);
        led.setPosition(ledCpos);
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
        telemetryM.addLine("Blitz Team 14212!");
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
        // telemetryM.addData(true, "getShooterVelo", getShooterVelo(distShooter));
        // telemetryM.addData(true, "getHoodCpos", getHoodCpos(distShooter));
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
        // telemetryM.addData(true, "alignTurret", alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), target));
        telemetryM.addData(true, "path state", pathState);
        telemetryM.addData(true, "x", follower.getPose().getX());
        telemetryM.addData(true, "y", follower.getPose().getY());
        telemetryM.addData(true, "heading", follower.getPose().getHeading());
        telemetryM.update();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        Variables.lastAutoPos = follower.getPose();
    }
}