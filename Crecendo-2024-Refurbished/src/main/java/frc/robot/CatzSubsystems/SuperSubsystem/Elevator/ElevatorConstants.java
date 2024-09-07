// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.Elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

/** Add your docs here. */
public class ElevatorConstants {

    // Subsystem safety disable
    public static final boolean isElevatorDisabled = true;

    // Gearbox definitions
    public static final double MAXPLANETARY_GEAR_RATIO   = 4.0 * 4.0;
    public static final double ELEVATOR_DRIVING_PULLEY   = 24.0;
    public static final double ELEVATOR_DRIVEN_PULLEY    = 18.0;
    public static final double ELEVATOR_RATIO_STAGE_ONE  = ELEVATOR_DRIVING_PULLEY/ELEVATOR_DRIVEN_PULLEY;
    public static final double FINAL_REDUCATION          = MAXPLANETARY_GEAR_RATIO * ELEVATOR_RATIO_STAGE_ONE;

    // Motor ID
    public static final int leaderID =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> 25;
            default -> 11;
        };
    public static final int ID_FOLLOWER =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> 25;
            default -> 11;
        };
    public static final double elevatorLength =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> Units.inchesToMeters(24.8);
            default -> Units.inchesToMeters(25.866);
        };

    // Misc constants
    public static final boolean IS_LEADER_INVERTED = false;
    public static final double  MIN_ROTATIONS = 0.0;
    public static final double  MAX_ROTATIONS = 117.0;
    public static final Translation2d elevatorOrigin = new Translation2d(-0.238, 0.298);

    // Initial PIDF and motion magic assignment
    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);//TODO fix gains
            case SN1 -> new Gains(75.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
            case SN_TEST -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
        };
    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(260, 400, 1600);
            case SN_TEST -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };

    // Adjustable Dashboard PIDF values
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/Gains/kP", gains.kP());
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/Gains/kI", gains.kI());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/Gains/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/Gains/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/Gains/kV", gains.kV());
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/Gains/kA", gains.kA());
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/Gains/kG", gains.kG());
    public static final LoggedTunableNumber mmCruiseVelocity = new LoggedTunableNumber("Elevator/Gains/kV", motionMagicParameters.mmCruiseVelocity());
    public static final LoggedTunableNumber mmAcceleration = new LoggedTunableNumber("Elevator/Gains/kA", motionMagicParameters.mmAcceleration());
    public static final LoggedTunableNumber mmJerk = new LoggedTunableNumber("Elevator/Gains/kG", motionMagicParameters.mmJerk());
    public static final LoggedTunableNumber lowerLimitRotations = new LoggedTunableNumber("Elevator/LowerLimitDegrees", MIN_ROTATIONS);
    public static final LoggedTunableNumber upperLimitRotations = new LoggedTunableNumber("Elevator/UpperLimitDegrees", MAX_ROTATIONS);

}