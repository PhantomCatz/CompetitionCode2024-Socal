package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;
import frc.robot.Utilities.MotorUtil.MotionMagicParameters;

public class ShooterPivotConstants {
    
    // Subsystem safety disable
    public static final boolean isShooterPivotDisabled = false;

    // Initial PIDF and motion magic assignment
    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(0.1, 0.1, 0.0005, 0.0, 0.0, 0.0, 0.0);//TODO fix gains
            case SN1 -> new Gains(75.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
            case SN_TEST -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
        };
    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(260, 400, 1600);
            case SN_TEST -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };

    // Adjustable Dashboard PIDF values
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("ShooterPivot/Gains/kP", gains.kP());
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("ShooterPivot/Gains/kI", gains.kI());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("ShooterPivot/Gains/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("ShooterPivot/Gains/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("ShooterPivot/Gains/kV", gains.kV());
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("ShooterPivot/Gains/kA", gains.kA());
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("ShooterPivot/Gains/kG", gains.kG());
}
