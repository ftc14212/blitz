package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryM {
    private Telemetry telemetry;
    private boolean debug;
    public TelemetryM(Telemetry telemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;
    }
    public void setDebug(boolean debug) {
        this.debug = debug;
    }
    public void updateTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void update() {
        telemetry.update();
    }
    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
    }
    public void addData(boolean debug, String caption, Object value) {
        if (this.debug && debug) telemetry.addData(caption, value);
    }
    public void addLine(String lineCaption) {
        telemetry.addLine(lineCaption);
    }
    public void addLine(boolean debug, String lineCaption) {
        if (this.debug && debug) telemetry.addLine(lineCaption);
    }
}
