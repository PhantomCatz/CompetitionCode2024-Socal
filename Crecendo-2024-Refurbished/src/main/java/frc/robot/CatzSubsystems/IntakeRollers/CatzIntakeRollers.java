// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.IntakeRollers;

import static frc.robot.CatzSubsystems.IntakeRollers.IntakeRollersConstants.*;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.LEDs.CatzLED;
import lombok.RequiredArgsConstructor;

public class CatzIntakeRollers extends SubsystemBase {

  // Hardware IO declaration
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

  // MISC variables
  public TargetSpeed recordTargetSpeed = TargetSpeed.IDLE;

  // State Machine Variables
  @RequiredArgsConstructor // TODO change intake speed to seperate
  public enum TargetSpeed { 
    IDLE(() ->  0.0),
    INTAKE(intakeSpeed),
    EJECT(() -> -0.6),
    HANDOFF_IN(() -> -0.1),
    HANDOFF_OUT(() -> 0.1);

    private final DoubleSupplier requestedRollerSpeed;
    private double getRollerSpeed() {
      return requestedRollerSpeed.getAsDouble();
    }
  }

  /** Creates a new CatzIntakeRollers. */
  public CatzIntakeRollers() {
    if(isIntakeRollersDisabled) {
      io = new IntakeRollersIONull();
      System.out.println("Intake Rollers Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new IntakeRollersIOReal();
          System.out.println("IntakeRollers Configured for Real");
        break;
        case REPLAY:
          io = new IntakeRollersIOReal() {};
          System.out.println("IntakeRollers Configured for Replayed simulation");
        break;
        case SIM:
          io = new IntakeRollersIOSim();
          System.out.println("IntakeRollers Configured for WPILIB simulation");
        break;
        default:
          io = null;
          System.out.println("IntakeRollers Unconfigured");
        break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/IntakeRollers", inputs);
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Intake hardware methods
  //
  //-----------------------------------------------------------------------------------------
  public boolean isNoteInIntake() {
    boolean isNoteInIntake = inputs.isFrontBeambreakBroken;
    CatzLED.getInstance().hasNoteAmp = isNoteInIntake;
    return isNoteInIntake;
  }

  private void setTargetRollerSpeed(TargetSpeed targetSpeed) {
    io.runDutycycle(targetSpeed.getRollerSpeed());
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Intake Roller commands
  //
  //-----------------------------------------------------------------------------------------
  public Command setRollersIn() {
    recordTargetSpeed = TargetSpeed.INTAKE;
    return startEnd(() -> setTargetRollerSpeed(TargetSpeed.INTAKE), 
                    () -> setTargetRollerSpeed(TargetSpeed.IDLE))
              .withName("Rollers Intake");
  }
  
  public Command setRollersOut() {
    recordTargetSpeed = TargetSpeed.EJECT;    
    return startEnd(() -> setTargetRollerSpeed(TargetSpeed.EJECT), 
                    () -> setTargetRollerSpeed(TargetSpeed.IDLE))
              .withName("Rollers Eject");
  }

  public Command setRollersOff() {
    recordTargetSpeed = TargetSpeed.IDLE;  
    return startEnd(() -> setTargetRollerSpeed(TargetSpeed.IDLE),
                    () -> setTargetRollerSpeed(TargetSpeed.IDLE))
              .withName("Rollers Off");
  }

  public Command setRollersHandofftoShooter() {
    recordTargetSpeed = TargetSpeed.HANDOFF_IN;  
    return startEnd(() -> setTargetRollerSpeed(TargetSpeed.HANDOFF_IN), 
                    () -> setTargetRollerSpeed(TargetSpeed.IDLE))
              .withName("Rollers HandoffIn");
  }
  
  public Command setRollersHandofftoIntake() {
    recordTargetSpeed = TargetSpeed.HANDOFF_OUT;  
    return startEnd(() -> setTargetRollerSpeed(TargetSpeed.HANDOFF_OUT), 
                    () -> setTargetRollerSpeed(TargetSpeed.IDLE))
              .withName("Rollers HandoffOut");
  }


}
