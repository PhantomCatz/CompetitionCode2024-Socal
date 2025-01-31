
package frc.robot.Commands.CharacterizationCmds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.Utilities.LoggedTunableNumber;

import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

import org.littletonrobotics.junction.Logger;

public class WheelRadiusCharacterization extends Command {
  private static final LoggedTunableNumber characterizationSpeed =
      new LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 1);
  private static final double driveRadius = DriveConstants.driveConfig.driveBaseRadius();
  private static final DoubleSupplier gyroYawRadsSupplier =
      () -> CatzRobotTracker.getInstance().getEstimatedPose().getRotation().getRadians();

  @RequiredArgsConstructor
  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;
  }

  private final CatzDrivetrain drive;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(CatzDrivetrain drive, Direction omegaDirection) {
    this.drive = drive;
    this.omegaDirection = omegaDirection;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drive.runWheelRadiusCharacterization(
        omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get())
    );

    // Get yaw and wheel positions
    accumGyroYawRads += Math.abs(MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads));
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    // wheel radius (meters) = gyro delta (radians) * drive base radius (meters) / wheel position delta (radians)
    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    Logger.recordOutput(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopDriving();
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
