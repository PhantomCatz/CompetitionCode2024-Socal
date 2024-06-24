// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import frc.robot.CatzConstants;

/** Add your docs here. */
public class ElevatorConstants {

    private static final double MAXPLANETARY_GEAR_RATIO = 4.0 * 4.0;

    private static final double ELEVATOR_DRIVING_PULLEY = 24.0;
    private static final double ELEVATOR_DRIVEN_PULLEY  = 18.0;
  
    private static final double ELEVATOR_RATIO_STAGE_ONE = ELEVATOR_DRIVING_PULLEY/ELEVATOR_DRIVEN_PULLEY;
    private static final double ELEVATOR_GEAR_RATIO      = MAXPLANETARY_GEAR_RATIO * ELEVATOR_RATIO_STAGE_ONE;
    
    public static final double reduction = ELEVATOR_GEAR_RATIO;

    public static final boolean leaderInverted = false;
    public static final double  minRotations = 0.0;
    public static final double  maxRotations = 117.0;

    public static final int leaderID =
        switch (CatzConstants.getRobot()) {
            case SN2 -> 25;
            default -> 11;
        };

    public static final int followerID =
        switch (CatzConstants.getRobot()) {
            case SN2 -> 25;
            default -> 11;
        };

    public static final Gains gains =
        switch (CatzConstants.getRobot()) {
            case SN2 -> new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);//TODO fix gains
            case SN1 -> new Gains(75.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
            case SIM -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
        };

    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobot()) {
            case SN2, SN1 -> new MotionMagicParameters(260, 400, 1600);
            case SIM -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };

    public record Gains(
        double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}

    public record MotionMagicParameters(
        double mmCruiseVelocity, double mmAcceleration, double mmJerk) {}

}