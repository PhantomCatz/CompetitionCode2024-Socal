// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.util.Alert;
import frc.robot.util.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class CatzElevator extends SubsystemBase {

  // Hardware Implementations
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Elevator Statemachine variables
  public static ElevatorState m_elevatorState;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Arm/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Arm/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber mmCruiseVelocity =
      new LoggedTunableNumber("Arm/Gains/kV", motionMagicParameters.mmCruiseVelocity());
  private static final LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Arm/Gains/kA", motionMagicParameters.mmAcceleration());
  private static final LoggedTunableNumber mmJerk =
      new LoggedTunableNumber("Arm/Gains/kG", motionMagicParameters.mmJerk());
  private static final LoggedTunableNumber lowerLimitRotations =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", minRotations);
  private static final LoggedTunableNumber upperLimitRotations =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", maxRotations);

  @RequiredArgsConstructor
  public static enum ElevatorState {
      SCORE_AMP(new LoggedTunableNumber("Elevator/ScoreAmpSetpoint",90.0)),
      SCORE_SOURCE(new LoggedTunableNumber("Elevator/ScoreSourceSetpoint",100.0)),
      STOW(()->0.0),
      WAIT(()->targetElevatorRotations);

    private final DoubleSupplier elevatorSetpointSupplier;

    private double getTargetPositionRotations() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  private final Alert leaderMotorDisconnected =
      new Alert("Elevator leader motor disconnected!", Alert.AlertType.WARNING);
  private final Alert followerMotorDisconnected =
      new Alert("Elevator follower motor disconnected!", Alert.AlertType.WARNING);


  // Elevator Class Variables
  private ElevatorFeedforward ff;

  private static double targetElevatorRotations = 0.0;
  private boolean isCharacterizing = false;
  private BooleanSupplier coastSupplier = () -> false;
  private boolean brakeModeEnabled = true;

  public CatzElevator() {
    if(ElevatorConstants.isElevatorDisabled) {
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/inputs ", inputs); 

    // Set alerts
    leaderMotorDisconnected.set(!inputs.isLeaderMotorConnected);
    followerMotorDisconnected.set(!inputs.isFollowerMotorConnected);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);
    LoggedTunableNumber.ifChanged(hashCode(), ()-> io.setMotionMagicParameters(mmCruiseVelocity.get(), mmAcceleration.get(), mmJerk.get()), 
        mmCruiseVelocity,
        mmAcceleration,
        mmJerk);

    if(DriverStation.isDisabled() || m_elevatorState == null) {
      io.stop();
    } else {
      // Run Softlimit check
      if(getElevatorPosition() > maxRotations) {
        io.stop();
      } else {
        // Run state check
        if(m_elevatorState == ElevatorState.STOW) {
          // Run Crossbar hit check
          if(getElevatorPosition() < 5.0) {
            io.stop();
          } else {
            // Run Setpoint Motion Magic    
            io.runSetpoint(m_elevatorState.getTargetPositionRotations(), ff.calculate(inputs.velocityRotPerSec));
          }
        } else {
          // Run Setpoint Motion Magic    
          io.runSetpoint(m_elevatorState.getTargetPositionRotations(), ff.calculate(inputs.velocityRotPerSec));
        }
      }
    }

  }

  public double getElevatorPosition() {
    return inputs.positionRotations;
  }

  public void setElevatorState(ElevatorState wantedstate) {
    m_elevatorState = wantedstate;
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
    return inputs.velocityRotPerSec;
  }

  public void endCharacterization() {
    isCharacterizing = false;
  }
}