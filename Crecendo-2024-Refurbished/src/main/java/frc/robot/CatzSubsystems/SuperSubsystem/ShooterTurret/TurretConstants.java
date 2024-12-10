package frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret;
import frc.robot.CatzConstants;


public class TurretConstants {
    
    // Subsystem Safety Disable
    public static boolean isShooterTurretDisabled = true;

    public static final int TURRET_MOTOR_ID = 
        switch(CatzConstants.getRobotType()) {
            case SN2 -> 60;
            case SN1 -> 3;
            case SN_TEST -> 4;
        };

    // -----------------------------------------------------------------------------------------------
    //  Turret Gearbox Defs
    // -----------------------------------------------------------------------------------------------
    private static final double TURRET_PLANETARY_GEARBOX_RATIO =   9.0;         //TBD

    private static final double TURRET_GEARBOX_DRIVING_GEAR    =  10.0;        //Pinion Gear
    private static final double TURRET_GEARBOX_DRIVEN_GEAR     = 140.0;        //Turret Gear
    private static final double TURRET_GEARBOX_RATIO           = TURRET_GEARBOX_DRIVEN_GEAR / TURRET_GEARBOX_DRIVING_GEAR; 
    
    public static final double TURRET_GEAR_REDUCTION           = TURRET_PLANETARY_GEARBOX_RATIO * TURRET_GEARBOX_RATIO;    
    public static final double TURRET_MOTOR_SHAFT_REV_PER_DEG  = TURRET_GEAR_REDUCTION / 360.0;     //TBD 
    

    //------------------------------------------------------------------------------------------------
    //  Position Defs & Variables
    //------------------------------------------------------------------------------------------------
    public static final double TURRET_MAX_ANGLE_DEG =  120.0;
    public static final double HOME_POSITION_DEG    =    0.0;  
    public static final double TURRET_MIN_ANGLE_DEG = -120.0;

    public static final double TURRET_MAX_SERVO_LIMIT_DEG =  60.0;
    public static final double TURRET_MIN_SERVO_LIMIT_DEG = -60.0;  

    public static final double SERVO_TURRET_CONSTRAINT = 0.5;

    // -----------------------------------------------------------------------------------------------
    //  Turret Closed Loop Processing (PIDF, etc)
    // -----------------------------------------------------------------------------------------------
    public static final double TURRET_kP = 0.02;
    public static final double TURRET_kI = 0.0;
    public static final double TURRET_kD = 0.0;
    
    private static final double LIMELIGHT_kP = 0.01;//0.013;
    private static final double LIMELIGHT_kI = 0.0;
    private static final double LIMELIGHT_kD = 0.0;

    private final static double TURRET_ANGLE_THRESHOLD_DEG = 3.0;
    private final double TURRET_APRILTAG_OFFSET_THRESHOLD = 5.0;
}
