package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private final HardwareMap hardwareMap;
    public CRServo sweeper;
    public Servo intake;
    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        sweeper = hardwareMap.get(CRServo.class, "sweeper");
        intake = hardwareMap.get(Servo.class, "intake");
        initIdle(); // Retract intake so its not over 18'
    }



    public void retractIntake() { intake.setPosition(Constants.retractIntake); }
    public void extendIntake() { intake.setPosition(Constants.extendIntake); }
    public void Sweep() { sweeper.setPower(Constants.Sweep); }
    public void retractSweep()  { sweeper.setPower(Constants.retractSweep); }
    public void terminateSweep() { sweeper.setPower(Constants.terminateSweep); }


    public void initIdle() {
        retractIntake();
        terminateSweep();
    }

    public void intakeReady() {
        extendIntake();
        Sweep(); // Starts Sweeping
    }

    public void intakeIdle() {
        terminateSweep(); //Completely stops spinning
        retractIntake(); //Retracts the intake
    }


    public void loop(Gamepad gamepad) {

        if (gamepad.left_bumper) {
            intakeReady();
        } else if (gamepad.right_bumper) {
            intakeIdle();
        } else if(gamepad.dpad_down) {
            extendIntake(); //Force to extend outside
        } else if(gamepad.dpad_up) {
            retractIntake(); //Retracts (Useful for retracting and keep spinning to ensure pixel gets inside)
        } else if(gamepad.left_stick_y < 0) {
            retractSweep();
        }


    }
}