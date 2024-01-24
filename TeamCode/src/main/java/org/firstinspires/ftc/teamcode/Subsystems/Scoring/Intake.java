package org.firstinspires.ftc.teamcode.Subsystems.Scoring;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import kotlin.coroutines.jvm.internal.Boxing;

public class Intake {

    private final HardwareMap hardwareMap;
    public DcMotorEx sweeper;
    public CRServo boxSweeper;
    public Servo intake;
    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        boxSweeper = hardwareMap.get(CRServo.class ,"boxSweeper");
        intake = hardwareMap.get(Servo.class, "intake");
        sweeper = hardwareMap.get(DcMotorEx.class, "sweeper");
        sweeper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sweeper.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        intake.scaleRange(0, 0.65);
        initIdle();
        terminateBoxSweeper();
        retractIntake();
    }

//SWITCHED 1/13/24 MOTOR POSITIONS

    public void Sweep() { sweeper.setPower(Constants.Sweep); }
    public void reverseSweep() { sweeper.setPower(Constants.reverseSweep); }
    public void terminateSweep() { sweeper.setPower(Constants.terminatePower); }

    public void extendIntake() { intake.setPosition(Constants.intakeExtend); }

    public void retractIntake () { intake.setPosition(Constants.intakeRetract); }

    public void boxSweep() { boxSweeper.setPower(Constants.boxSweep); }

    public void boxReverseSweep() { boxSweeper.setPower(Constants.boxReverseSweep); }

    public void terminateBoxSweeper() { boxSweeper.setPower(Constants.terminatePower);}

    public void initIdle() {
        terminateSweep();
    }

    public void loop(Gamepad gamepad) {

      if(gamepad.dpad_up) { //Extend Intake + Spin Intake Pixels + Sweep inside Box
          extendIntake();
          Sweep();
          boxSweep();
      } else if(gamepad.dpad_down) { //Reverse Intake Spin + Reverse outside Box + Retracts Intake
          reverseSweep();
          boxReverseSweep();
          retractIntake();
      } else if(gamepad.left_stick_button) { //Terminate Intake Spin
          terminateSweep();
      } else if(gamepad.right_stick_button) { //Terminates Box spin
          terminateBoxSweeper();
      } else if(gamepad.dpad_left) { //Reverses Box Spin
          boxReverseSweep();
      } else if(gamepad.left_bumper) { //Default V4b Position
          retractIntake();
      } else if(gamepad.right_bumper) { //Outer V4b Position
          extendIntake();
      }


        //      } else if(gamepad.dpad_right) { //Spins Pixels in
//          boxSweeper.setPower(-1);

    }
}

