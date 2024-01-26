package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Box {

    private final HardwareMap hardwareMap;
    public DistanceSensor pixelDetector;
    public Box(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public enum boxInfo {
        EMPTY,
        ONE_PIXEL,
        FULL
    }

    public boxInfo Size;

    public void init() {
        pixelDetector = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    public boxInfo getCount() {
        double distance = pixelDetector.getDistance(DistanceUnit.CM);
        switch ((int) distance) {
            case (int) Constants.EMPTY_BOX:
                Size = boxInfo.EMPTY;
                break;
            case (int) Constants.ONE_PIXEL:
                Size = boxInfo.ONE_PIXEL;
                break;
            case (int) Constants.FULL_BOX:
                Size = boxInfo.FULL;
                break;
        }
        return Size;
    }

}