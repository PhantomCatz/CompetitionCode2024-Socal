// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot;

import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

/** Add your docs here. */
public class IntakePivotConstants {

    // Subsystem safety disable
    public final static boolean isIntakePivotDisabled = false;

    // Gearbox Definitions
    private static final double INTAKE_PIVOT_DRIVEN_GEAR = 52.0;
    private static final double INTAKE_PIVOT_DRIVING_GEAR = 30.0;
    private static final double INTAKE_PIVOT_DRIVEN_SPROCKET = 32.0;
    private static final double INTAKE_PIVOT_DRIVING_SPROCKET = 16.0;
    private static final double MAX_PLANETARY_RATIO = 5.0;
    public static final double FINAL_REDUCTION = (INTAKE_PIVOT_DRIVEN_GEAR / INTAKE_PIVOT_DRIVING_GEAR) *
                                                          (INTAKE_PIVOT_DRIVEN_SPROCKET / INTAKE_PIVOT_DRIVING_SPROCKET) *
                                                          (MAX_PLANETARY_RATIO);
    private static final double INTAKE_PIVOT_MTR_REV_PER_DEG = FINAL_REDUCTION / 360.0;
    private static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG = 164.09;

    public static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV = 0.0;// INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG
                                                                       // * INTAKE_PIVOT_MTR_REV_PER_DEG;
    
    // Motor Id and misc qualities
    public final static int PIVOT_MTR_ID =
      switch (CatzConstants.getRobotType()) {
        case SN2 -> 12;
        case SN1 -> 11;
        case SN_TEST -> 0;
      };    

    // Misc constants
    private static final double SEMI_MANUAL_STEP_COEFFICIENT = 2.0;
    private final static double INTAKE_TURRET_MAX_ANGLE_FOR_HANDOFF_DEG = 17.0;
    public final static double  INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV = 40.0;
    private final static double INTAKE_ELEV_MIN_HEIGHT_FOR_AMP_TRANS_REV = 32.0;    
    public final static double INTAKE_STOW_ELEV_CLEARED_DEG = 120.0;
    public static final double INTAKE_TURRET_CLEARANCE = 125.0;
    public static final double INTAKE_MIN_ELEV_CLEARANCE_DEG = 100.0;
    public static final double INTAKE_TRANSITION_CHECK_DEG = -47.0; 
    public static final double SOFTLIMIT_STOW = 150.0;
    public static final double SOFTLIMIT_GROUND = -23.0;

    // Initial PIDF and motion magic assignment
    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(9.0, 0.0, 0.27, 0.0, 0.0, 0.0, 0.437);
            case SN1 -> new Gains(75.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
            case SN_TEST -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
        };
    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(80, 100, 1000);
            case SN_TEST -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };
      
    // Adjustable Dashboard PIDF values
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/Gains/kP", gains.kP());
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("IntakePivot/Gains/kI", gains.kI());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/Gains/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("IntakePivot/Gains/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("IntakePivot/Gains/kV", gains.kV());
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("IntakePivot/Gains/kA", gains.kA());
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("IntakePivot/Gains/kG", gains.kG());
    public static final LoggedTunableNumber mmCruiseVelocity = new LoggedTunableNumber("IntakePivot/Gains/kV", motionMagicParameters.mmCruiseVelocity());
    public static final LoggedTunableNumber mmAcceleration = new LoggedTunableNumber("IntakePivot/Gains/kA", motionMagicParameters.mmAcceleration());
    public static final LoggedTunableNumber mmJerk = new LoggedTunableNumber("IntakePivot/Gains/kG", motionMagicParameters.mmJerk());
    public static final LoggedTunableNumber lowerLimitRotations = new LoggedTunableNumber("IntakePivot/LowerLimitDegrees", 0.0);
    public static final LoggedTunableNumber upperLimitRotations = new LoggedTunableNumber("IntakePivot/UpperLimitDegrees", 0.0); //TODO add in limits

    // Adjustable Intake Pivot Positions
    public static final LoggedTunableNumber INTAKE_STOW_DEG           = new LoggedTunableNumber("IntakePivot/Stow",163.0);
    public static final LoggedTunableNumber INTAKE_SOURCE_LOAD_DN_DEG = new LoggedTunableNumber("IntakePivot/SourceLoad",30.0);
    public static final LoggedTunableNumber INTAKE_SOURCE_LOAD_UP_DEG = new LoggedTunableNumber("IntakePivot/SourceLoadUp", 97.0); //with drivetrain inner rail to the
                                                                                                                                                        // bottom inner rail 7 1/4 inches
    public static final LoggedTunableNumber INTAKE_AMP_SCORE_DN_DEG   = new LoggedTunableNumber("IntakePivot/ScoreDown", 95.6); //90.43; 
    public static final LoggedTunableNumber INTAKE_HOARD_DEG          = new LoggedTunableNumber("IntakePivot/",40.0);
    public static final LoggedTunableNumber INTAKE_AMP_SCORE_DEG      = new LoggedTunableNumber("IntakePivot/",80.0);
    public static final LoggedTunableNumber INTAKE_GROUND_PICKUP_DEG  = new LoggedTunableNumber("IntakePivot/",-22.0); //-25.0;

}
