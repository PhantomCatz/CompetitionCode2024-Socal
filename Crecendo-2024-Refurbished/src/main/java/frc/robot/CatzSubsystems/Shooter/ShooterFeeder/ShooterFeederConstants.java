package frc.robot.CatzSubsystems.Shooter.ShooterFeeder;

import frc.robot.CatzConstants;
import frc.robot.Utilities.MotorUtil.Gains;

public class ShooterFeederConstants {
    // Subsystem Safety Disable
    public static final boolean isShooterDisabledFeeder = false;

    // Motor ID Assignment
    public static final int FEEDER_ID = 
        switch(CatzConstants.getRobotType()) {
            case SN2 -> 23;
            case SN1 -> 23;
            case SN_TEST -> 24;
        };
    
    public static final Gains gains = 
        switch (CatzConstants.getRobotType()) {
            case SN2 ->     new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0.0, 0.0);
            case SN1 ->     new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0, 0.0 );
            case SN_TEST -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0.0);
        };

    // Feeder Ratio
    public static final double FEEDER_GEAR_RATIO = 3.0;
    public static final double FEEDER_COMPLIANT_WHEEL_DIAMETER_INCH = 2.25;

    // Feeder Roller Speeds //TODO constant speed changes so fwd = negative, bck = positive
    public static final double LOAD_MOTOR_SHOOTING_SPEED   = 1.0;
    public static final double LOAD_MOTOR_LOADING_SPEED    = 0.3;
    public static final double LOAD_MOTOR_BACKWARD_SPEED   = 0.8;
    public static final double LOAD_MOTOR_BWD_ADJUST_SPEED = 0.04;
    public static final double LOAD_MOTOR_TRANSFER_SPEED   = 0.4;
    public static final double LOAD_MOTOR_FWD_ADJUST_SPEED = 0.02;
}
