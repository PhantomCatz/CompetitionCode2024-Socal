package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.ShooterPivotConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret.TurretConstants;

public class ShooterPivotIOReal implements ShooterPivotIO {

    // Hardware
    private final CANSparkMax elevationNeoMtr;

    // Control
    private PIDController shooterPivotFeedback = new PIDController(gains.kP(), gains.kI(), gains.kD(), CatzConstants.LOOP_TIME);

    public ShooterPivotIOReal() {
        elevationNeoMtr = new CANSparkMax(61, MotorType.kBrushless);

        elevationNeoMtr.restoreFactoryDefaults();
        elevationNeoMtr.setSmartCurrentLimit(30);
        elevationNeoMtr.setIdleMode(IdleMode.kBrake);
        elevationNeoMtr.enableVoltageCompensation(12.0);
        elevationNeoMtr.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 32767);
        elevationNeoMtr.getEncoder().setPosition(33.95); //Starting position in relation to hardstopz

        elevationNeoMtr.burnFlash();
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {

        inputs.positionTicks = elevationNeoMtr.getEncoder().getPosition();
        inputs.appliedVolts = elevationNeoMtr.getBusVoltage();
        inputs.tempCelcius = elevationNeoMtr.getMotorTemperature();

        // System.out.println("pivot: " + inputs.positionTicks);
    }

    //-----------------------------------------------------------------------------------------
    //
    //    Pivot Power Output
    //
    //-----------------------------------------------------------------------------------------
    @Override
    public void runSetpointTicks(double currentPositionTicks,double setpointTicks) {
        double percentOutput = shooterPivotFeedback.calculate(currentPositionTicks, setpointTicks);
        elevationNeoMtr.set(percentOutput);
        Logger.recordOutput("shooterPivot/PercentOutput", percentOutput);
        Logger.recordOutput("shooterPivot/currentPositionTicks", currentPositionTicks);
        Logger.recordOutput("shooterPivot/setpointTicks", setpointTicks);
    }
  
    @Override
    public void runPercentOutput(double percentOutput) {
      elevationNeoMtr.set(percentOutput);
    }

    @Override
    public void setPID(double p, double i, double d) {
      shooterPivotFeedback = new PIDController(p, i, d, CatzConstants.LOOP_TIME);
    }
  
    @Override
    public void stop() {
      elevationNeoMtr.set(0);
    }
}
