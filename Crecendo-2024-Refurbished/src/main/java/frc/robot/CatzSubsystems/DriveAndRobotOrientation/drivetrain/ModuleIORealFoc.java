// 2637
// https://github.com/PhantomCatz/

package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.*;

import javax.swing.text.Position;

import org.littletonrobotics.junction.Logger;

public class ModuleIORealFoc implements ModuleIO {
  // Hardware
  private final TalonFX driveTalon;
  private final TalonFX steerTalon;
  private final MT6835 absEncoder;
  private final Rotation2d absoluteEncoderOffset;

  // Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveSupplyCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  private final StatusSignal<Double> steerPosition;
  private final StatusSignal<Double> steerVelocity;
  private final StatusSignal<Double> steerAppliedVolts;
  private final StatusSignal<Double> steerSupplyCurrent;
  private final StatusSignal<Double> steerTorqueCurrent;

  // Motor Configs
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration steerTalonConfig = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withUpdateFreqHz(0);
  private final PositionDutyCycle positionControl = new PositionDutyCycle(0).withUpdateFreqHz(0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);
  private final PIDController steerFeedback = new PIDController(moduleGainsAndRatios.steerkP(), 0.0, moduleGainsAndRatios.steerkD());

  // Status Code Initialization
  private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

  ModuleConfig m_config;
  public ModuleIORealFoc(ModuleConfig config) {
    m_config = config;
    // Init drive controllers from config constants
    driveTalon = new TalonFX(config.driveID());

    // Restore Factory Defaults
    driveTalon.getConfigurator().apply(new TalonFXConfiguration());

    // Config Motors Current Limits assume FOC is included with motors
    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0; 
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Gain Setting
    driveTalonConfig.Slot0.kP = moduleGainsAndRatios.drivekP();
    driveTalonConfig.Slot0.kD = moduleGainsAndRatios.drivekD();
    driveTalonConfig.Slot0.kS = moduleGainsAndRatios.driveFFkS();
    driveTalonConfig.Slot0.kV = moduleGainsAndRatios.driveFFkV();

    // Assign 100hz Signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    // Set Update Frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent
    );

    // Optimize bus utilization
    driveTalon.optimizeBusUtilization(0, 1.0);

    // Init steer controllers from config constants
    steerTalon = new TalonFX(config.steerID());
    absoluteEncoderOffset = Rotation2d.fromRotations(config.absoluteEncoderOffset());
    absEncoder = new MT6835(config.absoluteEncoderChannel(), false);

    // Restore Factory Defaults
    steerTalon.getConfigurator().apply(new TalonFXConfiguration());

    // Config Motors Current Limits assume FOC is included with motors
    steerTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0; 
    steerTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    steerTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    steerTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // Gain Setting
    steerTalonConfig.Slot0.kP = moduleGainsAndRatios.steerkP();
    steerTalonConfig.Slot0.kD = moduleGainsAndRatios.steerkD();

    // Assign 100hz Signals
    steerPosition = steerTalon.getPosition();
    steerVelocity = steerTalon.getVelocity();
    steerAppliedVolts = steerTalon.getMotorVoltage();
    steerSupplyCurrent = steerTalon.getSupplyCurrent();
    steerTorqueCurrent = steerTalon.getTorqueCurrent();

    // Set Update Frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        steerVelocity,
        steerAppliedVolts,
        steerSupplyCurrent,
        steerTorqueCurrent
    );

    steerTalon.optimizeBusUtilization(0, 1.0);


    //check if motors are initialized correctly
    for(int i=0;i<5;i++){
      initializationStatus = driveTalon.getConfigurator().apply(driveTalonConfig);
      if(!initializationStatus.isOK())
          System.out.println("Failed to Configure CAN ID" + config.driveID());

      initializationStatus = steerTalon.getConfigurator().apply(steerTalonConfig);
      if(!initializationStatus.isOK())
          System.out.println("Failed to Configure CAN ID" + config.steerID());
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh Drive Kraken status signals
    inputs.isDriveMotorConnected = BaseStatusSignal.refreshAll(
      drivePosition,
      driveVelocity,
      driveAppliedVolts,
      driveSupplyCurrent,
      driveTorqueCurrent)
    .isOK();

    inputs.isSteerMotorConnected = BaseStatusSignal.refreshAll(
      steerPosition,
      steerVelocity,
      steerAppliedVolts,
      steerSupplyCurrent,
      steerTorqueCurrent)
    .isOK();

    //Drive Input variable refresh
    inputs.drivePositionUnits     = drivePosition.getValueAsDouble();
    inputs.driveVelocityRPS       = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts      = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    // Refresh steer Motor Values
    inputs.rawAbsoluteEncPosition = Rotation2d.fromRotations(absEncoder.readAngle(0));
    inputs.steerAbsolutePosition   = Rotation2d.fromRotations(absEncoder.readAngle(0) - absoluteEncoderOffset.getRotations());
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
    inputs.steerSupplyCurrentAmps = steerSupplyCurrent.getValueAsDouble();
    inputs.steerTorqueCurrentAmps = steerTorqueCurrent.getValueAsDouble();

    inputs.odometryDrivePositionsMeters = new double[] {drivePosition.getValueAsDouble() * driveConfig.wheelRadius()};
    inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerAbsolutePosition};
  }

  public void runDriveVolts(double volts) {
    driveTalon.setControl(voltageControl.withOutput(volts));
  }

  public void runSteerVolts(double volts) {
    steerTalon.setVoltage(volts);
  }

  @Override
  public void runCharacterization(double input) {
    driveTalon.setControl(currentControl.withOutput(input));
  }

  @Override
  public void runDriveVelocityRPSIO(double velocityMetersPerSec) {
    driveTalon.setControl(velocityVoltage.withVelocity(velocityMetersPerSec));
  }

  public void runSteerPercentOutput(double percentOutput) {
    steerTalon.set(percentOutput);
  }

  @Override
  public void runSteerPositionSetpoint(double currentAngleRads, double targetAngleRads) {
		steerTalon.setControl(voltageControl.withOutput(steerFeedback.calculate(currentAngleRads, targetAngleRads)));

    Logger.recordOutput("Module " + m_config.driveID() + "/steer Target Angle", targetAngleRads);
    Logger.recordOutput("Module " + m_config.driveID() + "/steer current Angle", currentAngleRads);

  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
  }

  @Override
  public void setSteerPID(double kP, double kI, double kD) {
    steerFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveNeutralModeIO(NeutralModeValue type) {
    driveTalonConfig.MotorOutput.NeutralMode = type;
    driveTalon.getConfigurator().apply(driveTalonConfig);
  }

  @Override
  public void setSteerNeutralModeIO(NeutralModeValue type) {
    steerTalonConfig.MotorOutput.NeutralMode = type;
    steerTalon.getConfigurator().apply(steerTalonConfig);
  }
}