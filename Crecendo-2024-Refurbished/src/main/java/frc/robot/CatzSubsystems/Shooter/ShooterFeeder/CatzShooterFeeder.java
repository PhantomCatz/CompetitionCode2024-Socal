// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.Shooter.ShooterFeeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.LEDs.CatzLED;

public class CatzShooterFeeder extends SubsystemBase {

  // HardWare Implementations
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private ShooterFeederState currentShooterFeederState = ShooterFeederState.DISABLED;
  private ShooterFeederState previousShooterFeederState = ShooterFeederState.DISABLED;
  private AdjustState        determinedAdjustState = AdjustState.UNDETERMINED;

  // Flag Definement
  private boolean isAdjustStateDetermined;
  private boolean hasNoteClearedIntake;
  private boolean isNoteInPosition;

  // Counter delays
  private Timer stateTimerDelay = new Timer();

  // Access Feeder State Enums
  public enum ShooterFeederState{
    TO_SHOOTER,
    TO_INTAKE,
    SHOOT,
    DISABLED
  }

  // Inner Feeder Direction Enums
  private enum AdjustState{
    FWD,
    BCK,
    UNDETERMINED
  }
  
  /** Creates a new CatzShooterFeeder. */
  public CatzShooterFeeder() {
     if(ShooterFeederConstants.isShooterDisabledFeeder) {
      io = new FeederIONull();
      System.out.println("Feeder Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new FeederIOReal();
          System.out.println("Feeder Configured for Real");
          break;
        case REPLAY:
          io = new FeederIOReal() {};
          System.out.println("Feeder Configured for Replayed simulation");
          break;
        case SIM:
          io = new FeederIOSim();
          System.out.println("Feeder Configured for WPILIB simulation");
          break;
        default:
          io = null;
          System.out.println("Feeder Unconfigured");
          break;
      }
    }
    stateTimerDelay.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/ShooterFeeder ", inputs); 
    
    // Feeder State Change Init
    if(currentShooterFeederState != previousShooterFeederState) {
      switch(currentShooterFeederState) {
        case TO_SHOOTER: handleToShooterInit();
        break;
        case TO_INTAKE: handleToIntakeInit();
        break;
        case SHOOT: handleShootInit();
        break;
        case DISABLED: io.feedDisabled();
        break;
      }
    }

    // Feeder State Periodic
    switch(currentShooterFeederState) {
      case TO_SHOOTER: handleToShooterPeriodic();
      break;
      case TO_INTAKE: handleToIntakePeriodic();
      break;
      case SHOOT: handleShootPeriodic();
      break;
      case DISABLED:
      break;
    }

    previousShooterFeederState = currentShooterFeederState;

    Logger.recordOutput("ShooterFeeder/determinedAdjustState", determinedAdjustState);
  } //-End of feeder Periodic

  //-----------------------------------------------------------------------------------------
  //
  //    Feeder State Machine
  //
  //-----------------------------------------------------------------------------------------
  /** Feeder TOSHOOTER state on state change */
  private void handleToShooterInit() {
    stateTimerDelay.restart();
    io.loadFoward();
    isAdjustStateDetermined = false;
    isNoteInPosition = false;
    determinedAdjustState = AdjustState.UNDETERMINED;
  }

  /** Feeder TOSHOOTER state Periodic */
  private void handleToShooterPeriodic() {
    // Determine which direction to adjust note to
    if(isAdjustStateDetermined == false) {
      if(inputs.isLoadBeamBreakBroken) { 
        io.feedDisabled(); 
        if(stateTimerDelay.hasElapsed(0.06)) {
          if(inputs.isAdjustBeamBreakBroken) {
            io.fineAdjustBck();
            determinedAdjustState = AdjustState.BCK;
            isAdjustStateDetermined = true; //Logic exit
          } else {
            io.fineAdjustFwd();
            determinedAdjustState = AdjustState.FWD;
            isAdjustStateDetermined = true; //Logic exit
          }
        }
      }
    }

    // Adjust Note
    if(isAdjustStateDetermined == true) {
      if(determinedAdjustState == AdjustState.BCK) {
        if(inputs.isAdjustBeamBreakBroken == false) {
          io.feedDisabled();
          isNoteInPosition = true;
        }
      } else {
        if(inputs.isAdjustBeamBreakBroken == true) {
          io.feedDisabled();
          isNoteInPosition = true;
        }
      }
    }
  }

  /** Feeder TOINTAKE state state change */
  private void handleToIntakeInit() {
    stateTimerDelay.restart();
    io.resetLoadEnc();
    io.loadBackward();
    hasNoteClearedIntake = false;
    isNoteInPosition = false;
  }

  /** Feeder TOINTAKE state Periodic */
  private void handleToIntakePeriodic() {
    // Measure how far note travels
    if(hasNoteClearedIntake == false) {
      if(inputs.noteMovementUpInches > 2.0) { //TODO do we still need this?
        io.loadBackward();
        hasNoteClearedIntake = true; //Logic Exit
      }
    }
  }

  /** Feeder SHOOT state state change */
  private void handleShootInit() {
    io.feedShooter();
    isNoteInPosition = false;
  }

  /** Feeder SHOOT state Periodic */
  private void handleShootPeriodic() {
    // check if note has exited shooter
    if(!inputs.isAdjustBeamBreakBroken) {
      io.feedDisabled();
    }
  }

  
  //-----------------------------------------------------------------------------------------
  //    Feeder MISC
  //-----------------------------------------------------------------------------------------
  public boolean isNoteInRestingPosition() {
    return isNoteInPosition;
  }

  public boolean isNoteBeamBreakBroken() {
    boolean isNoteBeamBreakBroken = inputs.isAdjustBeamBreakBroken;
    CatzLED.getInstance().hasNoteSpeaker = isNoteBeamBreakBroken;
    return isNoteBeamBreakBroken;
  }

  //-----------------------------------------------------------------------------------------
  //    Command Feeder State Access methods
  //-----------------------------------------------------------------------------------------
  /** Shooter Feeder Access Method for Changing state */
  private void setShooterFeederState(ShooterFeederState wantedState) {
    currentShooterFeederState = wantedState;
  }

  public Command commandToShooter() {
    return startEnd(() -> setShooterFeederState(ShooterFeederState.TO_SHOOTER), 
                    () -> setShooterFeederState(ShooterFeederState.DISABLED))
        .withName("Feeder TO SHOOTER");
  }

  public Command commandToIntake() {
    return startEnd(() -> setShooterFeederState(ShooterFeederState.TO_INTAKE), 
                    () -> setShooterFeederState(ShooterFeederState.DISABLED))
        .withName("Feeder TOINTAKE");
  }

  public Command commandShootNote() {
    return startEnd(() -> setShooterFeederState(ShooterFeederState.SHOOT), 
                    () -> setShooterFeederState(ShooterFeederState.DISABLED))
        .withName("Feeder Shoot");
  }


}
