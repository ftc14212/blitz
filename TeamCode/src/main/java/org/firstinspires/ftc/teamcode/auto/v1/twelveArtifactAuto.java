package org.firstinspires.ftc.teamcode.auto.v1;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.v1.paths.twelveArtifactAutoPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.testCode.PIDTuneShooter;
import org.firstinspires.ftc.teamcode.testCode.PIDTuneTurret;
import org.firstinspires.ftc.teamcode.utils.CombinedCRServo;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.utils.Variables;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

/**
 * BLITZ V1.3 auto
 * Started code  @  11/14/25  @  10:54 am
 * It is a 12 ball auto with park.
 * @author David Grieas - 14212 MetroBotics
 * @version 1.0, 11/14/25
 **/

@Configurable
@Autonomous(name = "12 Baller Auto", group = ".ftc23403")
public class twelveArtifactAuto extends OpMode {
    private Follower follower = Constants.createFollower(hardwareMap);
    private Timer pathTimer, opmodeTimer;
    public static double speed = 1;
    private ElapsedTime timer;
    private ElapsedTime tResetT;
    private ElapsedTime autoTimeE;
    private TelemetryM telemetryM;
    public static boolean debugMode = true;
    private boolean tReset = false;
    private boolean tReset2 = false;
    public boolean SHOOTER_READY = false;
    public static boolean redSide = false;
    public static double turretOffset = -3;
    public static double blueShooter = 43;
    public static double redShooter = -49;
    public static boolean shooterOn = true;
    public static boolean indexerOn = true;
    public static int shooterT = 3500;
    public static int shooting_time = 4500;
    public static Pose target;
    public double distShooter;
    boolean ran = false;
    /** store the state of our auto. **/
    private int pathState;
    PIDController shooterPID;
    PIDController turretPID;
    // gamepads
    Gamepad currentGamepad1;
    Gamepad currentGamepad2;
    Gamepad previousGamepad1;
    Gamepad previousGamepad2;
    // motors
    CachingDcMotorEx turret; // 223 rpm
    CachingDcMotorEx shooterL = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterL")); // 6000 rpm
    CachingDcMotorEx shooterR = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterR")); // 6000 rpm
    CachingDcMotorEx intake; // 1150 rpm
    // servos
    private static CachingServo pivot; // 1x axon max
    private static CachingServo hood; // 1x axon mini
    private static CachingCRServo leftIndexer; // 1x axon mini
    private static CachingCRServo rightIndexer; // 1x axon mini
    private static CachingServo led; // 2x gobilda led lights RGB
    private static CombinedCRServo indexer; // 2x axon minis
    // positions
    public static double pivotCpos = 0.45;
    public static double hoodCpos = 1;
    public static double indexerCpos = 0;
    public static double ledCpos = 0.611;
    public static double turretTpos = 0;
    public static int shooterVelo = 0;
    /* preload lines */
    boolean intake1Started = false;
    boolean shootFarStarted = false;
    boolean intake2Started = false;
    boolean hatchStarted = false;
    boolean shootClose1Started = false;
    boolean intake3Started = false;
    boolean shootClose2Started = false;
    boolean parkStarted = false;


    /***
     * PATHS
     ***/
    /* line1 */
    public PathChain intake1 =
            follower.pathBuilder()
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.startPos.getPose(),
                            twelveArtifactAutoPaths.intakePos1Points.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.intakePos1Points.getEndHeading()))
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.intakePos1Points.getEndPoint(),
                            twelveArtifactAutoPaths.intakePos2Points.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.intakePos2Points.getEndHeading()))
                    .setBrakingStrength(6)
                    .build();
    public PathChain shootFar =
            follower.pathBuilder()
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.intakePos2Points.getEndPoint(),
                            twelveArtifactAutoPaths.shootFarPoints.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.shootFarPoints.getEndHeading()))
                    .setBrakingStrength(6)
                    .build();
    public PathChain intake2 =
            follower.pathBuilder()
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.shootFarPoints.getEndPoint(),
                            twelveArtifactAutoPaths.intakePos3Points.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.intakePos3Points.getEndHeading()))
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.intakePos3Points.getEndPoint(),
                            twelveArtifactAutoPaths.intakePos4Points.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.intakePos4Points.getEndHeading()))
                    .setBrakingStrength(6)
                    .build();
    public PathChain hatch =
            follower.pathBuilder()
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.intakePos4Points.getEndPoint(),
                            twelveArtifactAutoPaths.openHatch.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.openHatch.getEndHeading()))
                    .setBrakingStrength(6)
                    .build();
    public PathChain shootClose1 =
            follower.pathBuilder()
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.openHatch.getEndPoint(),
                            twelveArtifactAutoPaths.shootClosePoints.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.shootClosePoints.getEndHeading()))
                    .setBrakingStrength(6)
                    .build();
    public PathChain intake3 =
            follower.pathBuilder()
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.shootClosePoints.getEndPoint(),
                            twelveArtifactAutoPaths.intakePos5Points.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.intakePos5Points.getEndHeading()))
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.intakePos5Points.getEndPoint(),
                            twelveArtifactAutoPaths.intakePos6Points.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.intakePos6Points.getEndHeading()))
                    .setBrakingStrength(6)
                    .build();
    public PathChain shootClose2 =
            follower.pathBuilder()
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.intakePos6Points.getEndPoint(),
                            twelveArtifactAutoPaths.shootClosePoints.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.shootClosePoints.getEndHeading()))
                    .setBrakingStrength(6)
                    .build();
    public PathChain park =
            follower.pathBuilder()
                    .addPath(new BezierLine(
                            twelveArtifactAutoPaths.shootClosePoints.getEndPoint(),
                            twelveArtifactAutoPaths.parkPoints.getEndPoint()
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(twelveArtifactAutoPaths.parkPoints.getEndHeading()))
                    .setBrakingStrength(6)
                    .build();





    /** movements **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1: /* line1 */
                if (!intake1Started) {
                    // ALIGN_SHOOT();
                    // if (SHOOTER_READY || !shooterOn) FEED();
                    if (timer.milliseconds() >= shooting_time) {
                        //RESET_SHOOTER_TURRET();
                        // RESET_INTAKE();
                        // INTAKE();
                        follower.followPath(intake1, true);
                        intake1Started = true;
                    }
                }
                /// IZAAAAAAAAAAAAAAAAAAAAA AND DAVIDA
                /// mwah
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2: /* line2 */
                // if (INTAKE()) RESET_INTAKE();
                // ALIGN_SHOOT();
                if (!shootFarStarted) {
                    follower.followPath(shootFar, true);
                    shootFarStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!ran) {
                        timer.reset();
                        ran = true;
                    }
                    // if (SHOOTER_READY || !shooterOn) FEED();
                    if (timer.milliseconds() >= shooting_time) {
                        // RESET_SHOOTER_TURRET();
                        // RESET_INTAKE();
                        // INTAKE();
                        setPathState(291);
                    }
                }
                break;

        }
    }
    // util
    public static double angleDiffDegrees(double a, double b) {
        double diff = ((a - b + 180) % 360 + 360) % 360 - 180;
        return Math.abs(diff);
    }

    /** change state of the paths and actions and reset the timer **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    // controls
    /*
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
        if (shooterR.getVelocity() >= shooterVelo && shooterR.getVelocity() <= shooterVelo + 80) ledCpos = 1;
        else ledCpos = 0.388;
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

     */

    /** init **/
    @Override
    public void init() {
        /*
        // hardware
        // telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM = new TelemetryM(telemetry, debugMode);
        shooterPID = new PIDController(Math.sqrt(PIDTuneShooter.P), PIDTuneShooter.I, PIDTuneShooter.D);
        turretPID = new PIDController(Math.sqrt(PIDTuneTurret.P), PIDTuneTurret.I, PIDTuneTurret.D);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
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
        leftIndexer = new CachingCRServo(hardwareMap.get(CRServo.class, "leftIndexer")); // 1x axon mini
        rightIndexer = new CachingCRServo(hardwareMap.get(CRServo.class, "rightIndexer")); // 1x axon mini
        led = new CachingServo(hardwareMap.get(Servo.class, "led")); // 2x gobilda led lights RGB
        indexer = new CombinedCRServo(leftIndexer, rightIndexer); // 2x axon minis
        // directions
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        // limits
        pivot.scaleRange(0, 0.4);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        // LynxUtils.setLynxColor(255, 0, 255);
        // starting pos
        hood.setPosition(hoodCpos = 0);
        pivot.setPosition(pivotCpos = 0.45);
        led.setPosition(ledCpos = 0.611);
        // movement
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        autoTimeE = new ElapsedTime();
        tResetT = new ElapsedTime();
        timer = new ElapsedTime();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(twelveArtifactAutoPaths.startPos);
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, twelveArtifactAutoPaths.startPos.getX(), twelveArtifactAutoPaths.startPos.getY(), AngleUnit.DEGREES, twelveArtifactAutoPaths.startPos.getHeading()));

        // Draw the robot on the dashboard
        Tuning.drawCurrentC(follower);
        // telemetry
        telemetryM.addLine("BLITZ Team 14212!");

         */
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //buildPaths();
        // follower.setStartingPose(startPose);

    }

    /** play loop **/
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
        // These loop the movements of the robot
        follower.update();
        follower.setMaxPower(speed);
        autonomousPathUpdate();
        // Draw the robot on the dashboard
        Tuning.drawCurrentC(follower);
        // misc
        telemetryM.setDebug(debugMode);
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
        telemetryM.addData(true, "path state", pathState);
        telemetryM.addData(true, "x", follower.getPose().getX());
        telemetryM.addData(true, "y", follower.getPose().getY());
        telemetryM.addData(true, "heading", follower.getPose().getHeading());
        telemetryM.update();
    }

    /** init loop **/
    @Override
    public void init_loop() {
        // gamepad stuff
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        // field side
        if ((currentGamepad1.share && !previousGamepad1.share) || (currentGamepad2.share && !previousGamepad2.share)) redSide = !redSide;
        telemetryM.addData(true, "redSide", redSide);
        telemetryM.update();
    }

    /** play not loop **/
    @Override
    public void start() {
        autoTimeE.reset();
        opmodeTimer.resetTimer();
        setPathState(1);
    }

    /** stop **/
    @Override
    public void stop() {
        LynxUtils.setLynxColor(true, true, 0, 255, 0);
        Variables.lastAutoPos = follower.getPose();
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
}