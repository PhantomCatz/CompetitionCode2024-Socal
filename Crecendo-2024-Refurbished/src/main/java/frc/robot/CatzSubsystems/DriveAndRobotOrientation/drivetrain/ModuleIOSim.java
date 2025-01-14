package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), DriveConstants.moduleGainsAndRatios.driveReduction(), 0.025);
  private final DCMotorSim steerSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), DriveConstants.moduleGainsAndRatios.steerReduction(), 0.004);

  private final PIDController driveFeedback =
      new PIDController(0.1, 0.0, 0.0, CatzConstants.LOOP_TIME);
  private final PIDController steerFeedback =
      new PIDController(10.0, 0.0, 0.0, CatzConstants.LOOP_TIME);

  private double driveAppliedVolts = 0.0;
  private double steerAppliedVolts = 0.0;
  private final Rotation2d steerAbsoluteInitPosition;

  private boolean driveCoast = false;
  private SlewRateLimiter driveVoltsLimiter = new SlewRateLimiter(2.5);

  public ModuleIOSim(ModuleConfig config) {
    steerAbsoluteInitPosition = Rotation2d.fromRadians(Units.rotationsToRadians(config.absoluteEncoderOffset()));
    steerFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    if (driveCoast && DriverStation.isDisabled()) {
      runDriveVolts(driveVoltsLimiter.calculate(driveAppliedVolts));
    } else {
      driveVoltsLimiter.reset(driveAppliedVolts);
    }

    driveSim.update(CatzConstants.LOOP_TIME);
    steerSim.update(CatzConstants.LOOP_TIME);

    inputs.driveVelocityRPS =   driveSim.getAngularVelocityRPM()/60; //Convert to RPS
    inputs.drivePositionUnits = driveSim.getAngularPositionRad()/(2*Math.PI) * 4; // Fudged number to get better result
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.steerAbsolutePosition = new Rotation2d(steerSim.getAngularPositionRad()).plus(steerAbsoluteInitPosition);
    inputs.steerPosition = Rotation2d.fromRadians(steerSim.getAngularPositionRad());
    inputs.steerVelocityRadsPerSec = steerSim.getAngularVelocityRadPerSec();
    inputs.steerSupplyCurrentAmps = steerAppliedVolts;
    inputs.steerSupplyCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());

    inputs.odometryDrivePositionsMeters =
        new double[] {driveSim.getAngularPositionRad() * DriveConstants.driveConfig.wheelRadius()};
    inputs.odometrySteerPositions =
        new Rotation2d[] {Rotation2d.fromRadians(steerSim.getAngularPositionRad())};

  }

  private void runDriveVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  private void runsteerVolts(double volts) {
    steerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    steerSim.setInputVoltage(steerAppliedVolts);
  }

  @Override
  public void runCharacterization(double input) {
    runDriveVolts(input);
  }

  @Override
  public void runDriveVelocityRPSIO(double velocityRPS) {
    double velocityRadsPerSec = Units.rotationsToRadians(velocityRPS);
    runDriveVolts(
        driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadsPerSec));
  }

  @Override
  public void runSteerPositionSetpoint(double currentAngleRad, double angleRads) {
    runsteerVolts(steerFeedback.calculate(steerSim.getAngularPositionRad(), angleRads));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setSteerPID(double kP, double kI, double kD) {
    steerFeedback.setPID(kP, kI, kD);
  }

}
