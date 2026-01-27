package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Prism.Color;
import org.firstinspires.ftc.teamcode.utils.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utils.Prism.PrismAnimations;

import java.util.concurrent.TimeUnit;

@TeleOp(name="strips colors", group="test_ftc14212")
public class stripsConfig extends LinearOpMode {
    GoBildaPrismDriver prism;

    PrismAnimations.Solid pink = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid red = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid blue = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Blink endgame = new PrismAnimations.Blink(Color.PINK, Color.TRANSPARENT);
    int mode = 1;

    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class,"strips");

        pink.setBrightness(100);
        pink.setStartIndex(0);
        pink.setStopIndex(255);

        red.setBrightness(100);
        red.setStartIndex(0);
        red.setStopIndex(255);

        blue.setBrightness(100);
        blue.setStartIndex(0);
        blue.setStopIndex(255);

        endgame.setBrightness(100);
        endgame.setStartIndex(0);
        endgame.setStopIndex(255);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            if(gamepad1.aWasPressed()){
                prism.clearAllAnimations();
                mode++;
                if (mode == 1) prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, pink);
                if (mode == 2) prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, red);
                if (mode == 3) prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blue);
                if (mode == 4) prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, endgame);
                if (mode == 5) mode = 1;
            }
            if(gamepad1.dpadDownWasPressed()){
                if (mode == 1) prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                if (mode == 2) prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
                if (mode == 3) prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
                if (mode == 4) prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_4);
            }
            telemetry.addLine("Press A to insert and update the created animations.");
            telemetry.addLine("Press D-Pad Down to save current animations to Artboard");
            telemetry.addLine();
            telemetry.addData("Run Time (Hours): ",prism.getRunTime(TimeUnit.HOURS));
            telemetry.addData("Run Time (Minutes): ",prism.getRunTime(TimeUnit.MINUTES));
            telemetry.addData("Number of LEDS: ", prism.getNumberOfLEDs());
            telemetry.addData("Current FPS: ", prism.getCurrentFPS());
            telemetry.addData("mode", mode);
            telemetry.update();
        }
    }
}
