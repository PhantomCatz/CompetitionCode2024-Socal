// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret.TurretConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.CatzShooterPivot.ShooterPivotPositionType;
import frc.robot.Utilities.CatzMathUtils;
import lombok.RequiredArgsConstructor;


public class CatzShooterTurret {

  // Hardware IO declaration
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // Misc variables
  private TurretPosition currentMotionType = TurretPosition.HOME;
  private static double targetTurretPosition = 0.0;
  private static double manualPwr = 0.0;


  // State amchine
  @RequiredArgsConstructor
  public enum TurretPosition {
    AUTO_AIM(()-> CatzRobotTracker.getInstance().getAutoAimSpeakerParemeters()
                                          .turretHeading()
                                          .getDegrees()), 
    HOME(()->0.0),
    TEST_POSITION(()-> 90.0),
    IDLE(()->targetTurretPosition),
    MANUAL(()->manualPwr);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }
  
  /** Creates a new CatzShooterTurret. */
  public CatzShooterTurret() {
         if(TurretConstants.isShooterTurretDisabled) {
      io = new TurretIONull();
      System.out.println("Feeder Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new TurretIOReal();
          System.out.println("Turret Configured for Real");
        break;
        case REPLAY:
          io = new TurretIOReal() {};
          System.out.println("Turret Configured for Replayed simulation");
        break;
        case SIM:
          io = new TurretIOSim();
          System.out.println("Turret Configured for WPILIB simulation");
        break;
        default:
          io = null;
          System.out.println("Turret Unconfigured");
        break;
      }
    }
  }


  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/Turret", inputs);

    // Manual softlimits
    if((Math.abs(inputs.positionDegrees) > 11.0) && Math.abs(manualPwr) > 0) { //TODO test values
      manualPwr = 0;
       //currentMotionType = TurretPosition.HOME;
    }

    // Run Setpoint Control
    if(DriverStation.isDisabled()) {
      io.runPercentOutput(0.0);
    } else if(currentMotionType == TurretPosition.MANUAL) {
      io.runPercentOutput(manualPwr);
    } else {
      double targetPos = CatzMathUtils.Clamp(TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG, currentMotionType.getTargetMotionPosition());
      io.runSetpointDegrees(inputs.positionDegrees, targetPos);
    }

    Logger.recordOutput("Turret/ motion type", currentMotionType.getTargetMotionPosition());
    Logger.recordOutput("Turret/angle", CatzRobotTracker.getInstance().getAutoAimSpeakerParemeters()
                                          .turretHeading()
                                          .getDegrees());

  }

  //-----------------------------------------------------------------------------------------
  //
  //    Turret Misc Methods
  //
  //-----------------------------------------------------------------------------------------

  public double getTurretPosition() {
    return inputs.positionDegrees;
  }

  public boolean isTurretInPosition() {
    double currentPosition = Math.abs(getTurretPosition());
    double target = Math.abs(currentMotionType.getTargetMotionPosition());
    boolean isIntakeInPos = Math.abs(target - currentPosition) < 5.0;
    return isIntakeInPos;
  }

  public void setPercentOutput(Supplier<Double> percentOutput) {
    currentMotionType = TurretPosition.MANUAL;
    manualPwr = percentOutput.get();
  }


  //-----------------------------------------------------------------------------------------
  //
  //    State Setting method
  //
  //-----------------------------------------------------------------------------------------
  public void setTargetPosition(TurretPosition type) {
    currentMotionType = type;
  }
}
