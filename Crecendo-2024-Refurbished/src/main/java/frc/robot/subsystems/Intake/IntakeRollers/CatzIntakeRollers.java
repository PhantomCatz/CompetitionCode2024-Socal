// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeRollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatzIntakeRollers extends SubsystemBase {
  /** Creates a new CatzIntakeRollers. */
  public CatzIntakeRollers() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public Command setRollersIn() {
    return new InstantCommand();
  }
  
  public Command setRollersOut() {
    return new InstantCommand();
  }

  public Command setRollersHandoffIn() {
    return new InstantCommand();
  }
  
  public Command setRollersHandoffOut() {
    return new InstantCommand();
  }
}