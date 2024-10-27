// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem;

import java.util.function.Supplier;

import javax.lang.model.element.ElementKind;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzSubsystems.Shooter.ShooterFeeder.CatzShooterFeeder.ShooterFeederState;
import frc.robot.CatzSubsystems.SuperSubsystem.Elevator.CatzElevator;
import frc.robot.CatzSubsystems.SuperSubsystem.Elevator.CatzElevator.ElevatorPosition;
import frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot.CatzIntakePivot;
import frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot.CatzIntakePivot.IntakePivotPosition;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.CatzShooterPivot;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.CatzShooterPivot.ShooterPivotPositionType;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret.CatzShooterTurret;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret.CatzShooterTurret.TurretPosition;

public class CatzSuperSubsystem extends SubsystemBase {

  private static SuperstructureState currentSuperstructureState = SuperstructureState.STOW;
  private static SuperstructureState previousSuperstructureState = SuperstructureState.STOW;
  private static SuperstructureState previousCommandedSuperstructureState = SuperstructureState.STOW;

  public static enum SuperstructureState {
    STOW,
    AUTO_AIM,
    SCORE_AMP,
    SCORE_AMP_PART_2,
    INTAKE_GROUND,
    INTAKE_SOURCE,
    SUBWOOFER
  }

  private final CatzElevator elevator;
  private final CatzShooterTurret turret;
  private final CatzShooterPivot shooterPivot;
  private final CatzIntakePivot intakePivot;


  private boolean isElevatorCausingDanger = false;
  private boolean isTurretCausingDanger = false;
  private boolean isShooterPivotCausingDanger = false;
  private boolean isIntakePivotCausingDanger = false;
  private boolean isIntakeCausingDanger = false; 

  /** Creates a new Superstructure. */
  public CatzSuperSubsystem(CatzElevator elevator, CatzShooterTurret turret, CatzShooterPivot shooterPivot, CatzIntakePivot intakePivot) {
    this.elevator = elevator;
    this.turret = turret;
    this.shooterPivot = shooterPivot;
    this.intakePivot = intakePivot;
  }

  @Override
  public void periodic() {
    //Run Danger Checks for stow
    isTurretCausingDanger = (Math.abs(turret.getTurretPosition()) > 20.0); //Turret turned too far to left or right
    isShooterPivotCausingDanger = shooterPivot.getPositionTicks() > 3.0; //Shooter extended to high
    isElevatorCausingDanger = elevator.getElevatorPositionRotations() > 30.0; // Elevator too high for intake to stow
    isIntakePivotCausingDanger = (intakePivot.getIntakePivotDegree() > 90.0); // Intake not extended out far enough to clear elevator

    // Init SuperstructureState change
    if(currentSuperstructureState != previousSuperstructureState) 
    {
      // Run initializes for each state
      switch(currentSuperstructureState) 
      {
        case STOW:
          if(!isElevatorCausingDanger && !isTurretCausingDanger && !isShooterPivotCausingDanger) 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.STOW);
          } else 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.HOLD);
          } 
          elevator.setTargetPosition(ElevatorPosition.STOW);
          turret.setTargetPosition(TurretPosition.HOME);
          shooterPivot.setTargetMotionMethod(ShooterPivotPositionType.HOME);
        
        break;

        case INTAKE_GROUND:
          elevator.setTargetPosition(ElevatorPosition.STOW);
          turret.setTargetPosition(TurretPosition.HOME);
          shooterPivot.setTargetMotionMethod(ShooterPivotPositionType.HOME);
          if(!isElevatorCausingDanger && !isTurretCausingDanger && !isShooterPivotCausingDanger) 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.PICKUP_GROUND);
          } else 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.HOLD);
          }

        break;

        case SCORE_AMP:
          intakePivot.setIntakePivotState(IntakePivotPosition.SCORE_AMP);
          turret.setTargetPosition(TurretPosition.HOME);
          shooterPivot.setTargetMotionMethod(ShooterPivotPositionType.HOME); //TODO Might change
          if(!isIntakePivotCausingDanger) 
          {
            elevator.setTargetPosition(ElevatorPosition.STOW);
          } else 
          {
            elevator.setTargetPosition(ElevatorPosition.SCORE_AMP);
          }
        break;

        case SCORE_AMP_PART_2:
          intakePivot.setIntakePivotState(IntakePivotPosition.ANTI_STUCK);
          elevator.setTargetPosition(ElevatorPosition.SCORE_AMP_PART_2);

        break;  

        case AUTO_AIM:
          intakePivot.setIntakePivotState(IntakePivotPosition.HOLD);
          elevator.setTargetPosition(ElevatorPosition.STOW);
          turret.setTargetPosition(TurretPosition.HOME);
          shooterPivot.setTargetMotionMethod(ShooterPivotPositionType.AUTO_AIM);
        break;

        case SUBWOOFER:
          intakePivot.setIntakePivotState(IntakePivotPosition.HOLD);
          elevator.setTargetPosition(ElevatorPosition.STOW);
          turret.setTargetPosition(TurretPosition.HOME);
          shooterPivot.setTargetMotionMethod(ShooterPivotPositionType.SUBWOOFER);
        break;

        default:
      }
      previousCommandedSuperstructureState = previousSuperstructureState;
      previousSuperstructureState = currentSuperstructureState;
    }

    // Periodic Superstructure State Change
    switch(currentSuperstructureState) 
    {
      case STOW:
        if(!isTurretCausingDanger) 
        {
          if(!isElevatorCausingDanger) 
          {
            if(!isShooterPivotCausingDanger) {
              intakePivot.setIntakePivotState(IntakePivotPosition.STOW);
            }
          }
        }
      break;

      case INTAKE_GROUND:
        if(!isTurretCausingDanger) 
        {
          if(!isElevatorCausingDanger) 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.PICKUP_GROUND);
          }
        }
      break;

      case SCORE_AMP:
        if(!isIntakePivotCausingDanger) 
        {
          elevator.setTargetPosition(ElevatorPosition.SCORE_AMP);
        }
      break;

      case SCORE_AMP_PART_2:
        if(elevator.isElevatorInPosition()) 
        {
          intakePivot.setIntakePivotState(IntakePivotPosition.SCORE_AMP);
        }
      break;

      case AUTO_AIM:
        if(!isIntakePivotCausingDanger)
        {
          turret.setTargetPosition(TurretPosition.AUTO_AIM);
        }
        
      break;
      default:
    }

    //----------------------------------------

    // End of Comment
    
    //----------------------------------------

    Logger.recordOutput("Superstructure/dangerCheck/elevator", isElevatorCausingDanger);
    Logger.recordOutput("Superstructure/dangerCheck/intake", isIntakePivotCausingDanger);
    Logger.recordOutput("Superstructure/dangerCheck/shooterpivot", isShooterPivotCausingDanger);
    Logger.recordOutput("Superstructure/dangerCheck/Turret", isTurretCausingDanger);


    Logger.recordOutput("Superstructure/currentState", currentSuperstructureState.toString());
    Logger.recordOutput("Superstructure/lastCommandedState", previousCommandedSuperstructureState.toString());
    Logger.recordOutput("Superstructure/previousLoopState", previousSuperstructureState.toString());

    Logger.recordOutput("Supersture/isElevatorInposition", isElevatorInPosition());

    // Run Member Mechanism periodics
    elevator.periodic();
    turret.periodic();
    shooterPivot.periodic();
    intakePivot.periodic();


    previousSuperstructureState = currentSuperstructureState;

  } // - End of Superstructure Periodic

  //-----------------------------------------------------------------------------------------
  //
  //    Superstructure Getters
  //
  //-----------------------------------------------------------------------------------------

  public boolean isIntakeInPosition() {
    return intakePivot.isIntakeInPosition();
  }

  public boolean isElevatorInPosition() {
    return elevator.isElevatorInPosition();
  }

  public boolean isTurretAndPivotInPosition() {
    return (shooterPivot.isShooterPivotInPosition() && turret.isTurretInPosition());
  }

  public boolean isPreviousSuperSubsystemStateScoreAmp() {
    return (previousCommandedSuperstructureState == SuperstructureState.SCORE_AMP);
  }
  //-----------------------------------------------------------------------------------------
  //
  //    Superstructure Instance Factory Command Wrapper
  //
  //-----------------------------------------------------------------------------------------
  public Command setSuperStructureState(SuperstructureState newState) {
    return runOnce(() -> currentSuperstructureState = newState);
  }

  public Command deployIntake(IntakePivotPosition state) {
    return runOnce(() -> intakePivot.setIntakePivotState(state));
  }

  public Command setElevatorTargetPos(ElevatorPosition position) {
    return runOnce(() -> elevator.setTargetPosition(position));
  }

  public Command moveTurretToHome() {
    return runOnce(() -> turret.setTargetPosition(TurretPosition.HOME));
  }

  public Command setTurretManual(Supplier<Double> power) {
    return runOnce(() -> turret.setPercentOutput(power));
  }

  public Command setShooterPosition(ShooterPivotPositionType position) {
    return runOnce(() -> shooterPivot.setTargetMotionMethod(position));
  }

  public Command setShooterPivotManualPower(Supplier<Double> power) {
    return run(() -> shooterPivot.setPercentOutput(power));
  }

  public Command cancelSuperStructureCommands() {
    Command cancel = new InstantCommand();
    cancel.addRequirements(this);
    return cancel;
  }
}
