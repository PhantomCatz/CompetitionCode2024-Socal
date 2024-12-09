package frc.robot.CatzSubsystems.Shooter.ShooterFlywheels;

import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;

public class FlywheelConstants {

    // Subsystem safety disable
    public final static boolean isShooterFlywheelsDisabled = true;

    // motor id and misc qualities
    public static final FlywheelConfig flywheelConfig =
        switch (CatzConstants.getRobotType()) {
            case SN2 ->     new FlywheelConfig(21, 20, (1.0 / 2.0), 9000.0);
            case SN1 ->     new FlywheelConfig(5, 4, (1.0 / 2.0), 6000.0);
            case SN_TEST -> new FlywheelConfig(0, 0, (1.0 / 2.0), 9000.0);
        };

    // motor pidf constant assignment
     static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 ->     new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0.0, 0.0);
            case SN1 ->     new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0, 0.0 );
            case SN_TEST -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0.0);
        };

    // Adjustable Dashboard PIDF values
    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

    // Adjustable flywheel RPM values
    public static final LoggedTunableNumber shootingRpmLT          = new LoggedTunableNumber("Flywheels/ShootingLeftRpm", 5066.0);
    public static final LoggedTunableNumber shootingRpmRT          = new LoggedTunableNumber("Flywheels/ShootingRightRpm", 7733.0);
    public static final LoggedTunableNumber prepareShootMultiplier = new LoggedTunableNumber("Flywheels/PrepareShootMultiplier", 1.0);
    public static final LoggedTunableNumber intakingRpm            = new LoggedTunableNumber("Flywheels/IntakingRpm", -3000.0);
    public static final LoggedTunableNumber ejectingRpm            = new LoggedTunableNumber("Flywheels/EjectingRpm", 1000.0);
    public static final LoggedTunableNumber poopingRpm             = new LoggedTunableNumber("Flywheels/PoopingRpm", 3000.0);
    public static final LoggedTunableNumber maxAcceleration        = new LoggedTunableNumber( "Flywheels/MaxAccelerationRpmPerSec", flywheelConfig.maxAcclerationRpmPerSec());


    //-----------------------------------------------------------------------------------------------------
    //
    //    Flywheel specific record types
    //
    //-----------------------------------------------------------------------------------------------------
    public record FlywheelConfig(int leftID, 
                                 int rightID, 
                                 double reduction, 
                                 double maxAcclerationRpmPerSec
                                ) {}
}
