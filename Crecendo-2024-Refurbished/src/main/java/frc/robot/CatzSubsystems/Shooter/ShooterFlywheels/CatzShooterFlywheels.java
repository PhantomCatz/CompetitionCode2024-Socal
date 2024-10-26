package frc.robot.CatzSubsystems.Shooter.ShooterFlywheels;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.FlywheelConstants;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.LinearProfile;
import frc.robot.Utilities.LoggedTunableNumber;

import static frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.FlywheelConstants.*;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CatzShooterFlywheels extends SubsystemBase {

  // Hardware IO declaration
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  // Closed Loop Variable Declaration
  private final LinearProfile leftProfile;
  private final LinearProfile rightProfile;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());

  // MISC variables
  private boolean wasClosedLoop = false;
  private boolean isFlywheelClosedLoop = false;
  @Setter 
  private BooleanSupplier prepareShootSupplier = () -> false;
  @AutoLogOutput(key = "Flywheels/Goal")
  private ShooterSpeed targetSpeed = ShooterSpeed.IDLE;

  // Motor Console Alerts
  private final Alert disconnectedAlertLT = new Alert("Left flywheel disconnected!", Alert.AlertType.WARNING);
  private final Alert disconnectedAlertRT = new Alert("Right flywheel disconnected!", Alert.AlertType.WARNING);
  

  // Flywheel State machine
  @RequiredArgsConstructor
  public enum ShooterSpeed {
    IDLE(() -> 0.0, () -> 0.0),
    SHOOT(shootingRpmLT, shootingRpmRT),
    INTAKE(intakingRpm, intakingRpm),
    EJECT(ejectingRpm, ejectingRpm),
    POOP(poopingRpm, poopingRpm),
    HOARD(
        () -> 0,//RobotState.getInstance().getSuperPoopAimingParameters().flywheelSpeeds().leftSpeed(),
        () -> 0),//TODO add auto aim paremeters later
           // RobotState.getInstance().getSuperPoopAimingParameters().flywheelSpeeds().rightSpeed()),
    CHARACTERIZING(() -> 0.0, () -> 0.0);

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    private double getTargetSpeedLT() {
      return leftGoal.getAsDouble();
    }

    private double getTargetSpeedRT() {
      return rightGoal.getAsDouble();
    }
  }

  public CatzShooterFlywheels() {
     if(FlywheelConstants.isShooterFlywheelsDisabled) {
      io = new FlywheelsIONull();
      System.out.println("Feeder Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new FlywheelsIOReal();
          System.out.println("Flywheels Configured for Real");
        break;
        case REPLAY:
          io = new FlywheelsIOReal() {};
          System.out.println("Flywheels Configured for Replayed simulation");
        break;
        case SIM:
          io = new FlywheelsIOSim();
          System.out.println("Flywheels Configured for WPILIB simulation");
        break;
        default:
          io = null;
          System.out.println("Flywheels Unconfigured");
        break;
      }
    }

    leftProfile = new LinearProfile(maxAcceleration.get(), CatzConstants.LOOP_TIME);
    rightProfile = new LinearProfile(maxAcceleration.get(), CatzConstants.LOOP_TIME);

    setDefaultCommand(runOnce(() -> setTargetSpeed(ShooterSpeed.IDLE)).withName("Flywheels Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/Flywheels", inputs);

    // Set alerts
    disconnectedAlertLT.set(!inputs.isLeftMotorConnected);
    disconnectedAlertRT.set(!inputs.isRightMotorConnected);

    // Update controllers when user specifies
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), 
        kSVA -> ff = new SimpleMotorFeedforward(kSVA[0], kSVA[1], kSVA[2]), 
        kS, 
        kV, 
        kA
    );

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      setTargetSpeed(ShooterSpeed.IDLE);
    }

    // Get goal
    double leftGoal = targetSpeed.getTargetSpeedLT();
    double rightGoal = targetSpeed.getTargetSpeedRT();
    boolean idlePrepareShoot = targetSpeed == ShooterSpeed.IDLE && prepareShootSupplier.getAsBoolean();
    if (idlePrepareShoot) {
      leftGoal = ShooterSpeed.SHOOT.getTargetSpeedLT() * prepareShootMultiplier.get();
      rightGoal = ShooterSpeed.SHOOT.getTargetSpeedRT() * prepareShootMultiplier.get();
    }

    // Run to setpoint
    if (isFlywheelClosedLoop || idlePrepareShoot) {
      double leftSetpoint =  leftProfile.calculateSetpoint();
      double rightSetpoint = rightProfile.calculateSetpoint();
      io.runVelocity(leftSetpoint, 
                     rightSetpoint, 
                     ff.calculate(leftSetpoint), 
                     ff.calculate(rightSetpoint)
      );
      // CatzRobotTracker.getInstance().setFlywheelAccelerating(!atGoal() || isDrawingHighCurrent());
    } else if (targetSpeed == ShooterSpeed.IDLE) {
      // CatzRobotTracker.getInstance().setFlywheelAccelerating(false);
      io.stop();
    }

    Logger.recordOutput("Flywheels/SetpointLeftRpm", leftProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/SetpointRightRpm", rightProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/GoalLeftRpm", leftGoal);
    Logger.recordOutput("Flywheels/GoalRightRpm", rightGoal);
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Flywheel Misc Methods
  //
  //-----------------------------------------------------------------------------------------
  /** Runs flywheels at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    setTargetSpeed(ShooterSpeed.CHARACTERIZING);
    io.runCharacterizationLeft(input);
    io.runCharacterizationRight(input);
  }

  /** Get characterization velocity */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  }

  /** Get if velocity profile has ended */
  @AutoLogOutput(key = "Flywheels/AtGoal")
  public boolean atGoal() {
    return targetSpeed == ShooterSpeed.IDLE
        || (leftProfile.getCurrentSetpoint() == targetSpeed.getTargetSpeedLT()
            && rightProfile.getCurrentSetpoint() == targetSpeed.getTargetSpeedRT());
  }

  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.leftSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.rightSupplyCurrentAmps) > 50.0;
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Flywheel State Setting Method
  //
  //-----------------------------------------------------------------------------------------
  /** Set the current goal of the flywheel */
  private void setTargetSpeed(ShooterSpeed targetSpeed) {
    // Characterizing and idle flag
    if (targetSpeed == ShooterSpeed.CHARACTERIZING || targetSpeed == ShooterSpeed.IDLE) {
      wasClosedLoop = isFlywheelClosedLoop;
      isFlywheelClosedLoop = false;
      this.targetSpeed = targetSpeed;
      return; //don't set a goal
    }

    // Closed loop setting check
    if(this.targetSpeed == targetSpeed) {
      isFlywheelClosedLoop = true;
    }

    // Enable close loop
    if (!isFlywheelClosedLoop) {
      leftProfile.setGoal(-targetSpeed.getTargetSpeedLT(), inputs.leftVelocityRpm);
      rightProfile.setGoal(targetSpeed.getTargetSpeedRT(), inputs.rightVelocityRpm);
      isFlywheelClosedLoop = true;
    }
    this.targetSpeed = targetSpeed;
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Flywheel Instance Factory Commands
  //
  //-----------------------------------------------------------------------------------------
  public Command revCommand() {
    return startEnd(() -> setTargetSpeed(ShooterSpeed.SHOOT), 
                    () -> setTargetSpeed(ShooterSpeed.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command intakeCommand() {
    return startEnd(() -> setTargetSpeed(ShooterSpeed.INTAKE), () -> setTargetSpeed(ShooterSpeed.IDLE))
        .withName("Flywheels Intake");
  }

  public Command ejectCommand() {
    return startEnd(() -> setTargetSpeed(ShooterSpeed.EJECT), () -> setTargetSpeed(ShooterSpeed.IDLE))
        .withName("Flywheels Eject");
  }

  public Command hoardCommand() {
    return startEnd(() -> setTargetSpeed(ShooterSpeed.POOP), () -> setTargetSpeed(ShooterSpeed.IDLE))
          .withName("Flywheels Poop");
  }

  public Command autoHoardCommand() {
    return startEnd(() -> setTargetSpeed(ShooterSpeed.HOARD), () -> setTargetSpeed(ShooterSpeed.IDLE))
        .withName("Flywheels Super Poop");
  }
}
