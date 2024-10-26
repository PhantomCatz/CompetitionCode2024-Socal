// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.Elevator;

import static frc.robot.CatzSubsystems.SuperSubsystem.Elevator.ElevatorConstants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class CatzElevator {

  // Hardware IO declaration
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Closed Loop Variable Declaration
  private ElevatorFeedforward ff;

  // MISC variables
  private ElevatorPosition m_targetPosition = ElevatorPosition.STOW;
  private static double targetElevatorRotations = 0.0;
  private boolean isCharacterizing = false;
  private BooleanSupplier coastSupplier = () -> false;
  private boolean brakeModeEnabled = true;

  // Motor Console alerts
  private final Alert disconnectedAlertLeader   = new Alert("Elevator leader motor disconnected!", Alert.AlertType.WARNING);

  @RequiredArgsConstructor
  public static enum ElevatorPosition {
      SCORE_AMP(new LoggedTunableNumber("Elevator/ScoreAmpSetpoint",50.0)),
      SCORE_AMP_PART_2(new LoggedTunableNumber("Elevator/ScoreSourceSetpoint",90.0)),
      STOW(() -> 0.0),
      WAIT(() -> targetElevatorRotations);

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getTargetPositionRotations() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  public CatzElevator() {
    if(isElevatorDisabled) {
      io = new ElevatorIONull();
      System.out.println("Elevator Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new ElevatorIOReal();
          System.out.println("Elevator Configured for Real");
        break;
        case REPLAY:
          io = new ElevatorIOReal() {};
          System.out.println("Elevator Configured for Replayed simulation");
        break;
        case SIM:
          io = new ElevatorIOSim();
          System.out.println("Elevator Configured for WPILIB simulation");
        break;
        default:
          io = null;
          System.out.println("Elevator Unconfigured");
        break;
      }
    }

    ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/Elevator ", inputs); 

    // Set alerts
    disconnectedAlertLeader.set(!inputs.isLeaderMotorConnected);

    // Update controllers when user specifies
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);
    LoggedTunableNumber.ifChanged(hashCode(), 
        ()-> io.setMotionMagicParameters(mmCruiseVelocity.get(), mmAcceleration.get(), mmJerk.get()), 
        mmCruiseVelocity,
        mmAcceleration,
        mmJerk);


    // Run Setpoint Control
    if(DriverStation.isDisabled() || m_targetPosition == null) {
      io.stop();
    } else {
      io.runSetpoint(m_targetPosition.getTargetPositionRotations(), ff.calculate(inputs.velocityRps));

    }

    Logger.recordOutput("Elevator/CurrentRotations", getElevatorPositionRotations());
    Logger.recordOutput("Elevator/isElevatorInPos", isElevatorInPosition());

  }

  //-----------------------------------------------------------------------------------------
  //
  //    Misc Methods
  //
  //-----------------------------------------------------------------------------------------
  public double getElevatorPositionRotations() {
    return inputs.leaderPositionRotations;
  }

  public boolean isElevatorInPosition() {
    return (Math.abs((getElevatorPositionRotations() - m_targetPosition.getTargetPositionRotations())) < 5.0);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void runCharacterization(double amps) {
    isCharacterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRps;
  }

  public void endCharacterization() {
    isCharacterizing = false;
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Command Flywheel State Access methods
  //
  //-----------------------------------------------------------------------------------------
  public void setTargetPosition(ElevatorPosition targetPosition) {
    this.m_targetPosition = targetPosition;
  }
}
