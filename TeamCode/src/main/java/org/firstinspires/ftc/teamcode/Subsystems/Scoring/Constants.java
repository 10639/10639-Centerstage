package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

public class Constants {

    /** ======= CONSTANTS FOR LIFT  ======= **/

    public static final double COUNTS_PER_MOTOR_REV    = 103.8 ;   // eg: GoBILDA 1620m RPM Yellow Jacket
    public static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double WHEEL_DIAMETER_INCHES   = 1.673 ;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //19.7498748746487

    static final double FIRST_LEVEL = 10;
    static final double SECOND_LEVEL = 12;
    static final double THIRD_LEVEL = 20;

    public static final int LIFT_LEVEL_ZERO = 0;
    public static final int LIFT_FIRST_LEVEL = (int) (FIRST_LEVEL * COUNTS_PER_INCH);
    public static final int LIFT_SECOND_LEVEL = (int) (SECOND_LEVEL * COUNTS_PER_INCH);
    public static final int LIFT_THIRD_LEVEL = (int) (THIRD_LEVEL * COUNTS_PER_INCH);

    //Proportional, Integral, Derivative gains.
    public static final double Kp = 0.1, Ki = 0, Kd = 0.00001;
    //P -> Ability to Reach Target Position
    //I -> Leave at 0
    //D -> Dampener to counteract any oscillations
    //F -> Ability to counteract gravity and maintain its position (going up, down, and in place)
    // Feedforward component (F) -> Since we're doing this for a lift; we'll do a G value (gravity).
    //Refer to https://www.ctrlaltftc.com/feedforward-control#slide-gravity-feedforward
    public static final double Kf = 0.12;


    /** ======= CONSTANTS FOR ARM  ======= **/

    public static final double closeArm = 1; //Claw closed position
    public static final double openArm = 0; //Claw opened position
    public static final double rotationScore = 0;
    public static final double rotationIdle = 1;
    public static final double leftArm_Idle = 0.7;
    public static final double rightArm_Idle = 0;
    public static final double leftArm_Score = 0;
    public static final double rightArm_Score = 1;



    /** ======= CONSTANTS FOR INTAKE  ======= **/
    public static final double Sweep = -1;
    public static final double reverseSweep = 1;
    public static final double intakeExtend = 1;
    public static final double intakeRetract = 0;
    public static final double boxSweep = -1;
    public static final double boxReverseSweep = 1;
    public static final double terminatePower = 0;



    /** ======= CONSTANTS FOR VISION  ======= **/
    public static final double CONFIDENCE = 0.10;

    /** ======= CONSTANTS FOR DISTANCE SENSOR (CM) ======= **/
    public static final double EMPTY_BOX = 200;
    public static final double ONE_PIXEL = 100;
    public static final double FULL_BOX = 10;





}