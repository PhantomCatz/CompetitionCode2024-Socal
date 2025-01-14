/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.*;
import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.driveConfig;
import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.moduleGainsAndRatios;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;
import frc.robot.util.Alert;
import frc.robot.util.CatzMathUtils;
import frc.robot.util.CatzMathUtils.Conversions;
import frc.robot.util.LoggedTunableNumber;

public class CatzSwerveModule {

    //Module delcaration block
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    // Module Strings for Logging
    private static final String[] moduleNames = new String[] {"FL", "BL", "BR", "FR"};

    // Global swerve module variables
    private int m_index;
    private SwerveModuleState m_swerveModuleState = null;

    // FeedFoward definment
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(moduleGainsAndRatios.ffkS(),  //TBD Added feedforward
                                                                   moduleGainsAndRatios.ffkV(), 
                                                                   0.0);
    // Alerts                                                                               
    private final Alert driveMotorDisconnected; // TBD added alerts
    private final Alert steerMotorDisconnected;


    //----------------------------------------------------------------------------------------------
    //
    //  CatzServeModule() - Constructor
    //
    //----------------------------------------------------------------------------------------------
    public CatzSwerveModule(ModuleConfig config, int index) { // TODO instead of index pass in the string
        this.m_index = index;

        // Run subsystem disconnect check
        if(DriveConstants.isDriveDisabled) { //TODO add extra robot enviroment //TODO have discussion on mode and states definement
                io = new ModuleIONull();
                System.out.println("Module " + moduleNames[m_index] + " Unconfigured");
        } else {
            // Run Robot Mode hardware assignment
            switch (CatzConstants.hardwareMode) {
                case REAL  : io = new ModuleIORealFoc(config);
                             System.out.println("Module " + moduleNames[m_index] + " Configured for Real");
                break;

                case REPLAY: io = new ModuleIORealFoc(config) {};
                             System.out.println("Module " + moduleNames[m_index] + " Configured for Replay simulation");
                break;
                
                case SIM: io = new ModuleIOSim(config);
                            System.out.println("Module " + moduleNames[m_index] + " Configured for WPILIB simulation");
                break;
                
                default : io = null;                                                                    //TBD - how is this diff from ModuleIONull()?
                          System.out.println("Module " + moduleNames[m_index] + " Unconfigured");
                break;
            }
        }

        // Disconnected Alerts
        driveMotorDisconnected = new Alert(moduleNames[index] + " drive motor disconnected!", Alert.AlertType.WARNING);
        steerMotorDisconnected = new Alert(moduleNames[index] + " steer motor disconnected!", Alert.AlertType.WARNING);

        resetDriveEncs();   //TBD - why here vs module constructor where all of the other encoder init is being done?
    
    } // -End of CatzSwerveModule Constructor



    //----------------------------------------------------------------------------------------------
    //
    //  periodic()          //TBD - if this is only called by Drivetrain subsys periodic should it go in module periodic?  
    //
    //----------------------------------------------------------------------------------------------
    public void periodic() {
    
        io.updateInputs(inputs);               // Process and Log Module Inputs
    
        Logger.processInputs("Drive/M " + moduleNames[m_index], inputs); 

    } // -End of CatzSwerveModule Periodic 



    public void debugLogsSwerve(){
        Logger.recordOutput("Module " + moduleNames[m_index] + "/drive mps",              m_swerveModuleState.speedMetersPerSecond);
        Logger.recordOutput("Module " + moduleNames[m_index] + "/current state",          getModuleState());
        Logger.recordOutput("Module " + moduleNames[m_index] + "/angle error deg",        Math.toDegrees(m_swerveModuleState.angle.getRadians() - getAbsEncRadians()));
        Logger.recordOutput("Module " + moduleNames[m_index] + "/currentmoduleangle rad", getAbsEncRadians());
        Logger.recordOutput("Module " + moduleNames[m_index] + "/targetmoduleangle rad",  m_swerveModuleState.angle.getRadians());


        SmartDashboard.putNumber("absenctorad" + moduleNames[m_index] , getAbsEncRadians());
        SmartDashboard.putNumber("angle"       + moduleNames[m_index] , getCurrentRotation().getDegrees());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setModuleAngleAndVelocity(SwerveModuleState state) { //TODO log variables actually used in calculations
    
        this.m_swerveModuleState       = state;
        double targetAngleRad          = state.angle.getRadians();
        double currentAngleRad         = getAbsEncRadians();            
        
        // Run closed loop drive control
        io.runDriveVelocityRPSIO(Conversions.MPSToRPS(state.speedMetersPerSecond),
                                 ff.calculate(state.speedMetersPerSecond / driveConfig.wheelRadius())
        );

        io.runSteerPositionSetpoint(currentAngleRad, targetAngleRad);           // Run Closed Loop Steer Control
    }

    //----------------------------------------------------------------------------------------------
    //  Drivetrain Power Setting methods
    //----------------------------------------------------------------------------------------------
    public void setSteerPower(double pwr) {
        io.runSteerPercentOutputIO(pwr);
    }

    public void setDriveVelocity(double velocity) {
        io.runDriveVelocityRPSIO(velocity, 0.0);
    }

    public void stopDriving() {
        io.runDrivePwrPercentIO(0.0);
    }

    //----------------------------------------------------------------------------------------------
    //  Module Util Methods
    //----------------------------------------------------------------------------------------------
    public void setBreakMode(boolean enable) {
        io.setSteerBrakeModeIO(enable);
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
    }

    /** optimze wheel angles before sending to setdesiredstate method for logging */
    public SwerveModuleState optimizeWheelAngles(SwerveModuleState unoptimizedState) 
    {
        SwerveModuleState optimizedState = CatzMathUtils.optimize(unoptimizedState, getCurrentRotation()); 
    
        return optimizedState;
    }


    //----------------------------------------------------------------------------------------------
    //  Module getters
    //----------------------------------------------------------------------------------------------
    public SwerveModuleState getModuleState() {
        double velocityMPS = CatzMathUtils.Conversions.RPSToMPS(inputs.driveVelocityRPS);
        
        return new SwerveModuleState(velocityMPS, getCurrentRotation());
    }

    public SwerveModuleState getModuleStateSetpoint() {
        return m_swerveModuleState;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters() {
        // seconds cancels out
        return CatzMathUtils.Conversions.RPSToMPS(inputs.drivePositionUnits);
    }

    public double getPositionRads() {
        return Units.rotationsToRadians(inputs.drivePositionUnits);
    }

    /** Get steer angle of module as {@link Rotation2d}. */
    public Rotation2d getAngle() {
        return inputs.steerAbsolutePosition;
    }

    /** Get velocity of drive wheel for characterization */
    public double getCharacterizationVelocityRadPerSec() {
        return Units.rotationsToRadians(getDrvVelocityRPS());
    }

    public double getDrvVelocityRPS() {
        return inputs.driveVelocityRPS;
    }

    /** Outputs the Rotation object of the module */
    public Rotation2d getCurrentRotation() {
        return new Rotation2d(getAbsEncRadians());
    }
    
    private double getAbsEncRadians() {
        //mag enc value should already have offset applied
        return inputs.steerAbsolutePosition.getRadians();
    }
}