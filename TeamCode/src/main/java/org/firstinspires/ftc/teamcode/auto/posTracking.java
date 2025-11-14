package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

/**
 * Metrobotics pos tracking with odometry.
 * Track the robot's position using telemetry for ease of points.
 * @author David Grieas - 14212 MetroBotics
 * @version 1.3, 5/26/25
 **/

@Config("Auto pos tracking")
@Autonomous(name = "Position tracking odometry", group = "tools_ftc14212")
public class posTracking extends OpMode {
    public static double startPosX = 56.5;
    public static double startPosY = 8.39;
    public static double startPosRotation = 90;
    private final Pose startPos = new Pose(startPosX, startPosY, Math.toRadians(startPosRotation)); // start Pos
    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startPos.getX(), startPos.getY(), AngleUnit.RADIANS, startPos.getHeading()));
        follower.setStartingPose(startPos);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // telemetry for debugging
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("total heading", Math.toDegrees(follower.getTotalHeading()));
        Tuning.drawCurrentC(follower);
        telemetry.update();
        follower.update();
    }
}