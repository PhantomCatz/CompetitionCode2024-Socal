package frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret.TurretConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.CatzConstants;

public class TurretIOReal implements TurretIO{

    //Hardware IO declaration
    private final CANSparkMax turretNeo;

    //Control
    private final PIDController turretFeedback = new PIDController(TurretConstants.TURRET_kP, TurretConstants.TURRET_kI, TurretConstants.TURRET_kD, CatzConstants.LOOP_TIME);

    public TurretIOReal() {
        // Turret Neo hardware insantiation
        turretNeo = new CANSparkMax(TURRET_MOTOR_ID, MotorType.kBrushless);
        turretNeo.restoreFactoryDefaults();
        turretNeo.setSmartCurrentLimit(30);
        turretNeo.setIdleMode(IdleMode.kBrake);
        turretNeo.enableVoltageCompensation(12.0);
        turretNeo.getEncoder().setPositionConversionFactor((1/TURRET_GEAR_REDUCTION) * 360); //TODO
        turretNeo.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 32767);

        turretNeo.enableSoftLimit(SoftLimitDirection.kForward, true);
        turretNeo.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        turretNeo.setSoftLimit(SoftLimitDirection.kForward, 80);
        turretNeo.setSoftLimit(SoftLimitDirection.kReverse, -80);

        turretNeo.burnFlash(); //save configs so if pwr lost to be reapplied
        
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.velocityRps             = turretNeo.getEncoder().getVelocity()/60; // In rotations of Output shaft
        inputs.appliedVolts            = turretNeo.getBusVoltage();
        inputs.positionDegrees         = (turretNeo.getEncoder().getPosition());
        inputs.tempCelsius             = turretNeo.getMotorTemperature();

    }

    @Override
    public void runVolts(double volts) {
        turretNeo.setVoltage(volts);
    }

    @Override
    public void runPercentOutput(double outputPwr) {
        turretNeo.set(outputPwr);
    }

    @Override
    public void runSetpointDegrees(double currentAngleDegrees, double setpointAngleDegrees) {
        double percentOutput = turretFeedback.calculate(currentAngleDegrees, setpointAngleDegrees);
        runPercentOutput(percentOutput);
    }

    @Override 
    public void resetEncoder(double position){
        turretNeo.getEncoder().setPosition(position);
    }
    


}
