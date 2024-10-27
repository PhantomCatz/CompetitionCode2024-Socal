
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.ShooterPivotConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class CatzShooterPivot {
  
  // Hardware IO Declaration
  private final ShooterPivotIO io;
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  // Misc variables
  private ShooterPivotPositionType m_targetPosition = ShooterPivotPositionType.HOME;
  private double targetShooterPivotPosition = 0.0;
  private double manualPwr = 0.0;
  private boolean isCharacterizing = false;

  // Motor Console alerts
  private final Alert disconnectedAlertShooterPivot   = new Alert("Shooter Pivot motor disconnected!", Alert.AlertType.WARNING);

  //Simulator Declaration
    Mechanism2d shooterPivotVisualizer = new Mechanism2d(3, 3);
    MechanismRoot2d root = shooterPivotVisualizer.getRoot("PivotPoint", 2.5, 0.5);
    MechanismLigament2d pivot = root.append(new MechanismLigament2d("pivotArm",-2, 180, 50, new Color8Bit(Color.kAliceBlue)));



  //State machine
  @RequiredArgsConstructor
  public enum ShooterPivotPositionType {
    AUTO_AIM(()-> CatzRobotTracker.getInstance().getAutoAimSpeakerParemeters()
                                                .shooterPivotTicks()), 
    MANUAL(() -> 0.0),
    HOME(new LoggedTunableNumber("shooterPivot/tunnable/home", 0.0)),
    SUBWOOFER(new LoggedTunableNumber("shooterPivot/Tunnable/subwoofer", 31.5)), //Was 33.95
    TEST(new LoggedTunableNumber("shooterPivot/Tunnable/TestingTicks", 33));

    private final DoubleSupplier motionType;
    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }


  public CatzShooterPivot() {
     if(ShooterPivotConstants.isShooterPivotDisabled) {
      io = new ShooterPivotIONull();
      System.out.println("Shooter Pivot Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new ShooterPivotIOReal();
          System.out.println("Shooter Pivot Configured for Real");
        break;
        case REPLAY:
          io = new ShooterPivotIOReal() {};
          System.out.println("Shooter Pivot Configured for Replayed simulation");
        break;
        case SIM:
          io = new ShooterPivotIOSim();
          System.out.println("Shooter Pivot Configured for WPILIB simulation");
        break;
        default:
          io = null;
          System.out.println("Shooter Pivot Unconfigured");
        break;
      }
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/ShooterPivot", inputs);
    
    // Update Controllers when User Specifies
    LoggedTunableNumber.ifChanged(
      hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

    // Set Alerts
    disconnectedAlertShooterPivot.set(!inputs.isElevationMotorConnected);

    // Manual softlimits
    // if(inputs.positionTicks > 11.0 && manualPwr > 0) { //TODO test values
    //   manualPwr = 0;
    //   m_targetPosition = ShooterPivotPositionType.SUBWOOFER;
    // } else if (inputs.positionTicks < 0.0 && manualPwr < 0) {
    //   manualPwr = 0;
    //   m_targetPosition = ShooterPivotPositionType.HOME;
    // }

    // Run Setpoint Control
    if(DriverStation.isDisabled()) {
      io.stop();
    } else if(m_targetPosition == ShooterPivotPositionType.MANUAL) {
      io.runPercentOutput(manualPwr);
    } else {
      io.runSetpointTicks(getPositionTicks(), m_targetPosition.getTargetMotionPosition());
    }

    //simulator logic
    pivot.setAngle(Rotation2d.fromDegrees(inputs.positionTicks));
    Logger.recordOutput("ShooterPivot/pivot", shooterPivotVisualizer);

    Logger.recordOutput("ShooterPivot/pivotstate", m_targetPosition.getTargetMotionPosition());

  }

  //-----------------------------------------------------------------------------------------
  //
  //    Misc Methods
  //
  //-----------------------------------------------------------------------------------------
  public double getPositionTicks() {
    return inputs.positionTicks;
  }

  public boolean isShooterPivotInPosition() {
    return inputs.positionTicks < 10; // DO math
  }

  public void setNeutralMode() {
    io.stop();
  }

  public void setPercentOutput(Supplier<Double> percentOutput) {
    m_targetPosition = ShooterPivotPositionType.MANUAL;
    manualPwr = percentOutput.get();
  }


  //-----------------------------------------------------------------------------------------
  //
  //    State Setting method
  //
  //-----------------------------------------------------------------------------------------
  public void setTargetMotionMethod(ShooterPivotPositionType type) {
    m_targetPosition = type;
  }
}
