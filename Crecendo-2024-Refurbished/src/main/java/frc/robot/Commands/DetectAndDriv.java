// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;

import org.littletonrobotics.junction.Logger;

public class DetectAndDriv extends Command {
    private final double kP_X = 0.1;
    private final double kP_Y = 0.1;
    private final double MIN_ERROR = 2; // degrees
    private final double TIMEOUT = 3;

    private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
    private CatzDrivetrain drivetrain;

    private double rotDif;
    private double yTargetMetersPerSecond;


  /** Creates a new detectAndDrive. */
  public DetectAndDriv(CatzDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d curPose = tracker.getEstimatedPose();

    rotDif = kP_X * tracker.getDetectorCameraOffsetX();
    yTargetMetersPerSecond = kP_Y * tracker.getDetectorCameraOffsetY();
    
    drivetrain.drive(new ChassisSpeeds(0, yTargetMetersPerSecond, rotDif));
    Logger.recordOutput("DetectAndDrive/rotDif", rotDif);
    Logger.recordOutput("DetectAndDrive/y target mps", yTargetMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
