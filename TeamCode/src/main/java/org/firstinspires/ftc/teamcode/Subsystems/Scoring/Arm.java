package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Arm {

    private final HardwareMap hardwareMap;
    public ServoImplEx leftPivot, rightPivot;
    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
         leftPivot = hardwareMap.get(ServoImplEx.class, "leftPivot");
         rightPivot = hardwareMap.get(ServoImplEx.class, "rightPivot");
         dePower();
    }

    public void armIdle() {
        leftPivot.scaleRange(0, 0.1); //Decreasing (max) => Lifts Box Higher
        rightPivot.scaleRange(0, 0.1); //^

        leftPivot.setPosition(0);
        rightPivot.setPosition(1);
    }

    public void armScore() {
        rightPivot.scaleRange(0, 0.7);

        leftPivot.setPwmDisable();
        rightPivot.setPosition(1);
    }

    public void dePower() {
        leftPivot.setPwmDisable();
        rightPivot.setPwmDisable();
    }

    public void loop(Gamepad gamepad) {

         if(gamepad.cross) {
             armScore();
         }

    }
}