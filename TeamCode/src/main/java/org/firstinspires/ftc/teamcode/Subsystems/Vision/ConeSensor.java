package org.firstinspires.ftc.teamcode.Subsystems.Vision;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ConeSensor {

    private final HardwareMap hardwareMap;
    private ColorRangeSensor coneSensor;

    //Minimum value of RED/BLUE presence needed to confidently say that a cone is in possession
    public static double THRESHOLD = 150;

    public ConeSensor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        coneSensor = hardwareMap.get(ColorRangeSensor.class, "coneSensor");
    }

    public boolean hasCone() {
        int blue = coneSensor.blue();
        int red = coneSensor.red();

        return ((blue >= THRESHOLD) || (red >= THRESHOLD));
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Cone Grabbed (Threshold >= 150):", hasCone());
        telemetry.addData("Blue Value", coneSensor.blue());
        telemetry.addData("Red Value", coneSensor.red());
    }

}