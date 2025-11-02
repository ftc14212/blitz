/***
 * LynxUtils
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * Change the color of the Control Hub, Expansion Hub, and Servo Hub
 ***/
package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LynxUtils {
    private static LynxModule controlHub = null;
    private static LynxModule expansionHub = null;
    private static LynxModule servoHub = null;
    private static boolean validControlHub = false;
    private static boolean validExpansionHub = false;
    private static boolean validServoHub = false;
    // methods
    public static void initLynx(HardwareMap hardwareMap) {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            if (module.getRevProductNumber() != 0x311152 && module.getRevProductNumber() != 0x111855) {
                controlHub = module;
                validControlHub = true;
            } else if (module.getRevProductNumber() == 0x311152 && module.getRevProductNumber() != 0x111855) {
                expansionHub = module;
                validExpansionHub = true;
            } else if (module.getRevProductNumber() == 0x111855 && module.getRevProductNumber() != 0x311152) {
                servoHub = module;
                validServoHub = true;
            }
        }
    }
    public static void setLynxColor(int R, int G, int B) {
        @ColorInt int color = Color.rgb(R, G, B); // color int
        if (validControlHub) LynxUtils.controlHub.setConstant(color);
        if (validExpansionHub) LynxUtils.expansionHub.setConstant(color);
        if (validServoHub) LynxUtils.servoHub.setConstant(color);
    }
    public static void setLynxColor(boolean controlHub, boolean expansionHub, int R, int G, int B) {
        @ColorInt int color = Color.rgb(R, G, B); // color int
        if (validControlHub && controlHub) LynxUtils.controlHub.setConstant(color);
        if (validExpansionHub && expansionHub) LynxUtils.expansionHub.setConstant(color);
    }
    public static void setLynxColor(boolean controlHub, boolean expansionHub, boolean servoHub, int R, int G, int B) {
        @ColorInt int color = Color.rgb(R, G, B); // color int
        if (validControlHub && controlHub) LynxUtils.controlHub.setConstant(color);
        if (validExpansionHub && expansionHub) LynxUtils.expansionHub.setConstant(color);
        if (validServoHub && servoHub) LynxUtils.servoHub.setConstant(color);
    }
    public static void setLynxColor(int R1, int G1, int B1, int R2, int G2, int B2) {
        @ColorInt int color1 = Color.rgb(R1, G1, B1); // color int1
        @ColorInt int color2 = Color.rgb(R2, G2, B2); // color int2
        if (validControlHub) LynxUtils.controlHub.setConstant(color1);
        if (validExpansionHub) LynxUtils.expansionHub.setConstant(color2);
    }
    public static void setLynxColor(int R1, int G1, int B1, int R2, int G2, int B2, int R3, int G3, int B3) {
        @ColorInt int color1 = Color.rgb(R1, G1, B1); // color int1
        @ColorInt int color2 = Color.rgb(R2, G2, B2); // color int2
        @ColorInt int color3 = Color.rgb(R3, G3, B3); // color int3
        if (validControlHub) LynxUtils.controlHub.setConstant(color1);
        if (validExpansionHub) LynxUtils.expansionHub.setConstant(color2);
        if (validServoHub) LynxUtils.servoHub.setConstant(color3);
    }
    public static double getControlHubCurrent() {
        return validControlHub ? LynxUtils.controlHub.getCurrent(CurrentUnit.AMPS) : 0;
    }
    public static double getExpansionHubCurrent() {
        return validExpansionHub ? LynxUtils.expansionHub.getCurrent(CurrentUnit.AMPS) : 0;
    }
    public static double getServoHubCurrent() {
        return validServoHub ? LynxUtils.servoHub.getCurrent(CurrentUnit.AMPS) : 0;
    }
}