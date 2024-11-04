// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision;

import frc.robot.CatzConstants;


/** Add your docs here. */
public class VisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.005;
  public static final double thetaStdDevCoefficient = 0.01;

  public static final double[] stdDevFactors =
      switch (CatzConstants.getRobotType()) {
        case SN2 -> new double[] {1.0, 0.6, 1.0};
        case SN1 -> new double[] {1.0, 1.0};
        default -> new double[] {};
      };

}
