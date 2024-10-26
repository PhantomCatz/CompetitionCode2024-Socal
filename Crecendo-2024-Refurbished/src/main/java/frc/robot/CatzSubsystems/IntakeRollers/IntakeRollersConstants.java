package frc.robot.CatzSubsystems.IntakeRollers;

import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import frc.robot.Utilities.MotorUtil.Gains;


public class IntakeRollersConstants {
    // Subsystem safety disable
    public static final boolean isIntakeRollersDisabled = false;
    public static final int FRONT_BEAM_BREAK = 4;

    // motor ID assignment
    public static final int INTAKE_ROLLER_ID = 
        switch(CatzConstants.getRobotType()) {
            case SN2 -> 11;
            case SN1 -> 23;
            case SN_TEST -> 24;
        };

    public static final Gains gains = 
        switch (CatzConstants.getRobotType()) {
            case SN2 ->     new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0.0, 0.0);
            case SN1 ->     new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0, 0.0 );
            case SN_TEST -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0.0);
        };

    // Tunable states
    public static final LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/intakeRollersIn", 0.6);

}