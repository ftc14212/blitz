package org.firstinspires.ftc.teamcode.auto.v1.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utils.PPMP;
import org.firstinspires.ftc.teamcode.utils.PPPoint;

@Configurable()
public class twelveArtifactAutoPaths {
    public static Follower follower;
    /***
     * POINTS
     ***/
    /* start pos */
    public static Pose startPos = new Pose(56.5, 8.39, Math.toRadians(90));
    /* line1 */
    public static PPPoint.beizerLine intakePos1Points = new PPPoint.beizerLine(
            34.3,
            70,
            0
    );
    /* line2 */
    public static PPPoint.beizerCurve intakePos2Points = new PPPoint.beizerCurve(
            57.3,
            28,
            -90,
            new PPMP(22.3, 64.77),
            new PPMP(22.54, 14.24),
            new PPMP(59, 48.2)
    );
    /* line2a */
    public static PPPoint.beizerLine shootFarPoints = new PPPoint.beizerLine(
            23,
            28,
            -90
    );
    /* line3 */
    public static PPPoint.beizerCurve intakePos3Points = new PPPoint.beizerCurve(
            55.5,
            18.1,
            -90,
            new PPMP(63.8, 30.1)
    );
    /* line3a */
    public static PPPoint.beizerLine intakePos4Points = new PPPoint.beizerLine(
            23,
            18.1,
            -90
    );
    /* line4 */
    public static PPPoint.beizerCurve openHatch = new PPPoint.beizerCurve(
            55,
            12,
            -90,
            new PPMP(60.26, 20.64)
    );
    /* line4a */
    public static PPPoint.beizerLine shootClosePoints = new PPPoint.beizerLine(
            23,
            10,
            -90
    );
    /* line5 */
    public static PPPoint.beizerCurve intakePos5Points = new PPPoint.beizerCurve(
            8.3,
            29,
            -180,
            new PPMP(36.78, 11.87)
    );
    /* line6 */
    public static PPPoint.beizerLine intakePos6Points = new PPPoint.beizerLine(
            32.5,
            68,
            180
    );
    /* line7 */
    // shoot from close
    /* line8 */
    public static PPPoint.beizerLine parkPoints = new PPPoint.beizerLine(
            33.3,
            75.5,
            180
    );
}
