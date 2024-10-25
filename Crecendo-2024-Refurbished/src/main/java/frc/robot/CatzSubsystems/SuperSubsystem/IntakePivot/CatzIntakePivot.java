// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot;

import static frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot.IntakePivotConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CatzConstants;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class CatzIntakePivot {

  // Hardware IO declaration
  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

  // Intake Statemachine variables
  private IntakePivotPosition m_targetPosition = IntakePivotPosition.STOW;

  // Cloased Loop variable declaration
  private ArmFeedforward ff = new ArmFeedforward(gains.kS(), gains.kG(), gains.kV());

  // MISC variables
  private static double targetDegree = 0.0;

  @RequiredArgsConstructor
  public static enum IntakePivotPosition {
    SCORE_AMP(new LoggedTunableNumber("Intake/Pivot Score Amp", 80.0)),
    PICKUP_SOURCE(new LoggedTunableNumber("Intake/Pivot Pickup Souce", 90.0)),
    PICKUP_GROUND(new LoggedTunableNumber("Intake/Pivot Pickup Ground", -24.0)),
    HOLD(new LoggedTunableNumber("Intake/Pivot Holding Position", 90.0)),
    STOW(new LoggedTunableNumber("Intake/Pivot Stow Position", 164)),
    ANTI_STUCK(new LoggedTunableNumber("Intake/Pivot Anti Stuck Position", 110)),
    WAIT(()-> targetDegree);
    
    private final DoubleSupplier intakePivotSetpointSupplier;

    private double getTargetDegree() {
      return intakePivotSetpointSupplier.getAsDouble();
    }
  }

  /** Creates a new CatzIntake. */
  public CatzIntakePivot() {
    if(IntakePivotConstants.isIntakePivotDisabled) {
      io = new IntakePivotIONull();
      System.out.println("Intake Pivot Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new IntakePivotIOReal();
          System.out.println("Intake Pivot Configured for Real");
          break;
        case REPLAY:
          io = new IntakePivotIOReal() {};
          System.out.println("Intake Pivot Configured for Replayed simulation");
          break;
        case SIM:
          io = new IntakePivotIOSim();
          System.out.println("Intake Pivot Configured for WPILIB simulation");
          break;
        default:
          io = null;
          System.out.println("Intake Pivot Unconfigured");
          break;
      }
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/intakePivot", inputs);


    // Run Setpoint Control
    if(DriverStation.isDisabled() || m_targetPosition == null) {
      io.stop();
    } else {
      // Run Softlimit check
      if(getIntakePivotDegree() > SOFTLIMIT_STOW && m_targetPosition == IntakePivotPosition.STOW) {
        io.stop();
      } else {
         io.runSetpoint(m_targetPosition.getTargetDegree());
      }
    }
    
    Logger.recordOutput("IntakePivot/Target Degree Pivot", m_targetPosition.getTargetDegree());
    Logger.recordOutput("IntakePivot/Enum Intake Pivot", m_targetPosition.name());
    Logger.recordOutput("IntakePivot/isIntakeInPosition", isIntakeInPosition());
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Pivot Misc Methods
  //
  //-----------------------------------------------------------------------------------------
  public double getIntakePivotDegree() {
    return Units.radiansToDegrees(inputs.positionRads);
  }


  public boolean isIntakeInPosition() {
    double intakePosition = Math.abs(getIntakePivotDegree());
    double target = Math.abs(m_targetPosition.getTargetDegree());
    boolean isIntakeInPos = Math.abs(target - intakePosition) < 2.0;
    return isIntakeInPos;
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Command Flywheel State Access methods
  //
  //-----------------------------------------------------------------------------------------
  public void setIntakePivotState(IntakePivotPosition targetPosition) {
    m_targetPosition = targetPosition;
  }
}
