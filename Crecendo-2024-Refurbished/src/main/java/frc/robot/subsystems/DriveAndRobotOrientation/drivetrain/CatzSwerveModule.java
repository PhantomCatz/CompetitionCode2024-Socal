/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;
import frc.robot.util.CatzMathUtils;

public class CatzSwerveModule {
    //Module delcaration block
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    //object variable declaration
    private PIDController m_PID;
                
    //steering pid constants
    private final double kP = 0.4; 
    private final double kI = 0.01;

    private final double kD = 0.000;

    //global swerve module constants
    private double m_wheelOffset;
    private int m_index;

    public CatzSwerveModule(ModuleConfig config) {

        switch (CatzConstants.currentMode) {
            case REAL: io = new ModuleIOReal(config);
                        System.out.println("Module " + config.driveID() + " Configured for Real");
            break;

            case REPLAY : io = new ModuleIOReal(config) {};
                        System.out.println("Module " + config.driveID() + " Configured for Replay simulation");
            break;

            case SIM: io = new ModuleIOSim(config);
            break;
            default : io = null;
                        System.out.println("Module " + config.driveID() + " Unconfigured");
            break;
        }

        m_PID = new PIDController(kP, kI, kD);
        m_index = config.driveID();
        System.out.println(m_index);
        m_wheelOffset = config.absoluteEncoderOffset();
        resetDriveEncs();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drv/M " + Integer.toString(m_index), inputs); 
    }

    //-----------------------------------------LOGS----------------------------------------------
    /*
    * For Debugging Purposes 
    * Keep them commmented ALWAYS if you are not using it 
    */
        public void debugLogsSwerve(){
            // Logger.recordOutput("Module/absenctorad" + Integer.toString(m_index) , getAbsEncRadians());
            // Logger.recordOutput("Module/angle" + Integer.toString(m_index) , getCurrentRotation().getDegrees());
            // Logger.recordOutput("Module " + Integer.toString(m_index) + "/drive applied volts", inputs.driveAppliedVolts);


            // SmartDashboard.putNumber("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());
            // SmartDashboard.putNumber("angle" + Integer.toString(m_index) , getCurrentRotation().getDegrees());
        }

    //----------------------------------------Setting pwr methods-------------------------------
    public void setSteerPower(double pwr) {
        io.setSteerPwrIO(pwr);
    }

    public void setDriveVelocity(double velocity) {
        io.setDriveVelocityIO(velocity, 0.0);
    }

    public double getAverageRawMagEnc(){
        double sum = 0;

        for(int i = 0; i < 100; i++){
            sum += inputs.magEncoderValue;
            Timer.delay(0.01);
        }

        return sum/100.0;
    }

    public void stopDriving() {
        io.setDrivePwrPercentIO(0.0);
    }

    //----------------------------------Util Methods catzswerve------------------------
    public double getDrvDistanceRaw() {
        return inputs.driveMtrSensorPosition;
    }

    public void setBreakMode(boolean enable) {
        io.setSteerBrakeModeIO(enable);
    }

    public double getPositionRads() {
        return Units.rotationsToRadians(inputs.driveMtrSensorPosition);
    }

    public double getDrvVelocity() {
        return inputs.driveMtrVelocity;
    }
    
    private double getAbsEncRadians() {
        return (inputs.magEncoderValue - m_wheelOffset) * 2 * Math.PI;
    }
    
    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        double unadjustedSpeedSetpoint = state.speedMetersPerSecond;
        double targetAngleRad          = state.angle.getRadians();
        double currentAngleRad         = getAbsEncRadians();
        // Run closed loop drive control

        //calculate drive pwr
        double driveRPS = CatzMathUtils.Conversions.MPSToRPS(unadjustedSpeedSetpoint);
        

        //set drive velocity
        setDriveVelocity(driveRPS);

        //calculate steer pwr
        //negative steer power because of coordinate system
        double steerPIDpwr = -m_PID.calculate(currentAngleRad, targetAngleRad); 

        if(CatzConstants.getRobot() == CatzConstants.RobotType.SIM) {
            io.runTurnPositionSetpoint(targetAngleRad);
        } else {
            setSteerPower(steerPIDpwr);
        }

        //DEBUG
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/angle error deg", Math.toDegrees(targetAngleRad-currentAngleRad));
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/drive rps", driveRPS);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/current state", getModuleState());
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/turn power", steerPIDpwr);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/currentmoduleangle rad", currentAngleRad);
        Logger.recordOutput("Module " + Integer.toString(m_index) + "/targetmoduleangle rad", targetAngleRad);
    }

    //optimze wheel angles before sending to setdesiredstate method for logging
    public SwerveModuleState optimizeWheelAngles(SwerveModuleState unoptimizedState) {
        SwerveModuleState optimizedState = CatzMathUtils.optimize(unoptimizedState, getCurrentRotation()); 
        return optimizedState;
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
    }

    //inputs the rotation object as radian conversion
    public Rotation2d getCurrentRotation() {
        return new Rotation2d(getAbsEncRadians());
    }

    public SwerveModuleState getModuleState() {
        double velocityMPS = CatzMathUtils.Conversions.RPSToMPS(inputs.driveMtrVelocity);
        
        return new SwerveModuleState(velocityMPS, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters() {
        // seconds cancels out
        return CatzMathUtils.Conversions.RPSToMPS(inputs.driveMtrSensorPosition);
    }
}
