package org.firstinspires.ftc.teamcode.auto.v1;

import com.arcrobotics.ftclib.controller.PIDController;
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

@Autonomous(name = "12 baller auto RED", group = ".ftc14212")
public class autoRS extends OpMode {
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
    CachingDcMotorEx turret; // 223 rpm
    CachingDcMotorEx shooterL; // 6000 rpm
    CachingDcMotorEx shooterR; // 6000 rpm
    CachingDcMotorEx intake; // 1150 rpm
    // servos
    private static CachingServo pivot; // 1x axon max
    private static CachingServo hood; // 1x axon mini
    private static CachingServo led; // 2x gobilda led lights RGB
    private static CombinedCRServo indexer; // 2x axon minis
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
    public static double turretOffset = -3;
    public static double blueShooter = 43;
    public static double redShooter = -49;
    public static boolean shooterOn = true;
    public static int intakeWait = 500;
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
    public static final Pose startPose = auto.startPose.mirror();
    public static final Pose intakeStart1Pose = auto.intakeStart1Pose.mirror();
    public static final Pose intakeEnd1Pose = auto.intakeEnd1Pose.mirror();
    public static final Pose shootFarPose = auto.shootFarPose.mirror();
    public static final Pose intakeStart2Pose = auto.intakeStart2Pose.mirror();
    public static final Pose intakeEnd2Pose = auto.intakeEnd2Pose.mirror();
    public static final Pose shootClosePose = auto.shootClosePose.mirror();
    public static final Pose intakeEnd3Pose = auto.intakeEnd3Pose.mirror();
    public static final Pose parkPose = auto.parkPose.mirror();
    private PathChain intake1, scoreFar, intake2, scoreClose1, intake3, scoreClose2, park;
    /* preload lines */
    boolean intake1Started = false;
    boolean shootFarStarted = false;
    boolean intake2Started = false;
    boolean scoreCloseStarted = false;
    boolean intake3Started = false;
    boolean shootClose2Started = false;
    boolean parkStarted = false;

    public void buildPaths() {
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, intakeStart1Pose))
                .setConstantHeadingInterpolation(intakeStart1Pose.getHeading())
                .addPath(new BezierLine(intakeStart1Pose, intakeEnd1Pose))
                .setConstantHeadingInterpolation(intakeEnd1Pose.getHeading())
                .build();
        scoreFar = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd1Pose, shootFarPose))
                .setConstantHeadingInterpolation(shootFarPose.getHeading())
                .build();
        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootFarPose, intakeStart2Pose))
                .setConstantHeadingInterpolation(intakeStart2Pose.getHeading())
                .addPath(new BezierLine(intakeStart2Pose, intakeEnd2Pose))
                .setConstantHeadingInterpolation(intakeEnd2Pose.getHeading())
                .build();
        scoreClose1 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd2Pose, shootClosePose))
                .setConstantHeadingInterpolation(shootClosePose.getHeading())
                .build();
        intake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, intakeEnd3Pose))
                .setConstantHeadingInterpolation(intakeEnd3Pose.getHeading())
                .build();
        scoreClose2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd3Pose, shootClosePose))
                .setConstantHeadingInterpolation(shootClosePose.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, parkPose))
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
        turret = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "turret")); // 223 rpm
        shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
        shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
        intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake")); // 1150 rpm
        // servos
        pivot = new CachingServo(hardwareMap.get(Servo.class, "intakePivot")); // 1x axon max
        hood = new CachingServo(hardwareMap.get(Servo.class, "hood")); // 1x axon mini
        CachingCRServo leftIndexer = new CachingCRServo(hardwareMap.get(CRServo.class, "leftIndexer")); // 1x axon mini
        CachingCRServo rightIndexer = new CachingCRServo(hardwareMap.get(CRServo.class, "rightIndexer")); // 1x axon mini
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        indexer = new CombinedCRServo(leftIndexer, rightIndexer); // 2x axon minis
        // directions
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        // reset pos
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // limits
        pivot.scaleRange(0, 0.4);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.45);
        led.setPosition(ledCpos = 0.611);
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
                if (!intake1Started) {
                    turretTpos = -68;
                    shooterVelo = 1300;
                    hoodCpos = 0.5;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (actionTimer.getElapsedTime() >= shooterT && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        INTAKE();
                        speed = 0.9;
                        follower.followPath(intake1, true);
                        intake1Started = true;
                    }
                }
                if (!follower.isBusy() && intake1Started) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(1);
                    }
                }
                break;
            case 1:
                if (!shootFarStarted) {
                    follower.followPath(scoreFar, true);
                    shootFarStarted = true;
                }
                if (!follower.isBusy() && shootFarStarted) {
                    turretTpos = -68;
                    shooterVelo = 1300;
                    hoodCpos = 0.5;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (actionTimer.getElapsedTime() >= shooterT && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        ran = false;
                        ran2 = false;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!intake2Started) {
                    speed = 0.9;
                    INTAKE();
                    follower.followPath(intake2, true);
                    intake2Started = true;
                }
                if (!follower.isBusy()) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (!scoreCloseStarted) {
                    follower.followPath(scoreClose1, true);
                    scoreCloseStarted = true;
                }
                if (!follower.isBusy() && scoreCloseStarted) {
                    turretTpos = -45;
                    shooterVelo = 1050;
                    hoodCpos = 0.2;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (actionTimer.getElapsedTime() >= shooterT && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        ran = false;
                        ran2 = false;
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!intake3Started) {
                    speed = 0.8;
                    INTAKE();
                    follower.followPath(intake3, true);
                    intake3Started = true;
                }
                if (!follower.isBusy()) {
                    if (!ran2) {
                        timer.reset();
                        ran2 = true;
                    }
                    if (timer.milliseconds() > intakeWait) {
                        speed = 1;
                        ran = false;
                        ran2 = false;
                        RESET_INTAKE();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!shootClose2Started) {
                    follower.followPath(scoreClose2, true);
                    shootClose2Started = true;
                }
                if (!follower.isBusy() && shootClose2Started) {
                    turretTpos = -45;
                    shooterVelo = 1050;
                    hoodCpos = 0.2;
                    if (shooterR.getVelocity() >= shooterVelo) {
                        if (!ran) {
                            actionTimer.resetTimer();
                            ran = true;
                        }
                        ledCpos = 1;
                        FEED();
                    }
                    if (actionTimer.getElapsedTime() >= shooterT && ledCpos == 1) {
                        RESET_SHOOTER_TURRET();
                        ran = false;
                        ran2 = false;
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(-1);
                }
                break;

        }
    }

    public boolean INTAKE() {
        pivotCpos = 0.75;
        if (indexerOn) indexerCpos = 1;
        intake.setPower(1);
        shooterVelo = -75;
        if (!indexerOn && shooterR.getCurrent(CurrentUnit.MILLIAMPS) <= shooterT) indexerCpos = 0;
        if (shooterR.getCurrent(CurrentUnit.MILLIAMPS) >= shooterT) {
            indexerOn = false;
            indexerCpos = -1;
            return true;
        }
        return false;
    }

    public void OUTTAKE() {
        indexerOn = true;
        pivotCpos = 0.75;
        indexerCpos = -1;
        intake.setPower(-1);
    }
    public void RESET_INTAKE() {
        pivotCpos = 0.45;
        indexerCpos = 0;
        intake.setPower(0);
        shooterVelo = 0;
    }
    public void FEED() {
        indexerOn = true;
        pivotCpos = 0.45;
        indexerCpos = 1;
        intake.setPower(1);
    }

    public void ALIGN_SHOOT() {
        // if (shooterR.getVelocity() >= shooterVelo && shooterR.getVelocity() <= shooterVelo + 80) ledCpos = 1;
        // else ledCpos = 0.388;
        // turretTpos = turretOffset;
        double turretO = alignTurret(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()), target);
        turretTpos = turretO;
        turretTpos += turretO > 200 ? -360 : turretO < -200 ? 360 : 0;
        shooterVelo = getShooterVelo(distShooter);
        hoodCpos = getHoodCpos(distShooter);
    }

    public void RESET_SHOOTER_TURRET() {
        tReset = true;
        tResetT.reset();
        turretTpos = 0;
        ledCpos = 0.611;
        shooterVelo = 0;
        hoodCpos = 0;
    }




    public int getShooterVelo(double distShooter) {
        int shooterVelo = 0;
        double hoodCpos = 0;

        // Table values
        double[] distances = {20, 50, 80, 120};
        int[] velocities = {900, 1000, 1150, 1350};
        double[] hoods = {0.0, 0.2, 0.3, 0.45};

        // If below or above range, clamp to min/max
        if (distShooter <= distances[0]) {
            shooterVelo = velocities[0];
            hoodCpos = hoods[0];
        } else if (distShooter >= distances[distances.length - 1]) {
            shooterVelo = velocities[velocities.length - 1];
            hoodCpos = hoods[hoods.length - 1];
        } else {
            // Linear interpolation between points
            for (int i = 0; i < distances.length - 1; i++) {
                if (distShooter >= distances[i] && distShooter <= distances[i + 1]) {
                    double t = (distShooter - distances[i]) / (distances[i + 1] - distances[i]);
                    shooterVelo = (int) (velocities[i] + t * (velocities[i + 1] - velocities[i]));
                    hoodCpos = hoods[i] + t * (hoods[i + 1] - hoods[i]);
                    break;
                }
            }
        }
        return shooterVelo;
    }
    public double getHoodCpos(double distShooter) {
        int shooterVelo = 0;
        double hoodCpos = 0;

        // Table values
        double[] distances = {20, 50, 80, 120};
        int[] velocities = {900, 1000, 1150, 1350};
        double[] hoods = {0.0, 0.2, 0.3, 0.45};

        // If below or above range, clamp to min/max
        if (distShooter <= distances[0]) {
            shooterVelo = velocities[0];
            hoodCpos = hoods[0];
        } else if (distShooter >= distances[distances.length - 1]) {
            shooterVelo = velocities[velocities.length - 1];
            hoodCpos = hoods[hoods.length - 1];
        } else {
            // Linear interpolation between points
            for (int i = 0; i < distances.length - 1; i++) {
                if (distShooter >= distances[i] && distShooter <= distances[i + 1]) {
                    double t = (distShooter - distances[i]) / (distances[i + 1] - distances[i]);
                    shooterVelo = (int) (velocities[i] + t * (velocities[i + 1] - velocities[i]));
                    hoodCpos = hoods[i] + t * (hoods[i + 1] - hoods[i]);
                    break;
                }
            }
        }
        return hoodCpos;
    }
    public double alignTurret(double x, double y, double heading, Pose target) {
        x = turretOffset + x;
        y = 0 + y;
        double goalX = target.getX();
        double goalY = target.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(goalX - x, goalY - y));
        return angleToGoal + heading - 90;
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
        double turretCpos = (turret.getCurrentPosition() / (PIDTuneTurret.TPR * PIDTuneTurret.ratio)) * 360;
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
        double sPower = shooterPID.calculate(shooterR.getVelocity(), shooterVelo) + shooterVelo;
        shooterR.setVelocity(sPower); // leader
        shooterL.setVelocity(sPower); // follower
        // turret code
        if (tResetT.milliseconds() > 1500) tReset = false;
        if (Math.abs(turretCpos - turretTpos) > 2) {
            double power = Math.max(-1, Math.min(1, turretPID.calculate(turretCpos, turretTpos) + PIDTuneTurret.F));
            if (tReset && turretCpos >= 130) tReset2 = true;
            if (tReset2) {
                turret.setPower(Math.abs(power));
                if (turretCpos < 20) tReset2 = false;
            }
            else turret.setPower(-power);
        } else turret.setPower(0);
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