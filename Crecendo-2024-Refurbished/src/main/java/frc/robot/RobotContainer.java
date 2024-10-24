// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.CatzAutonomous;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.RobotSenario;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.CatzVision;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.VisionIO;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.VisionIOLimeLight;
import frc.robot.CatzSubsystems.IntakeRollers.CatzIntakeRollers;
import frc.robot.CatzSubsystems.LEDs.CatzLED;
import frc.robot.CatzSubsystems.Shooter.ShooterFeeder.CatzShooterFeeder;
import frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.CatzShooterFlywheels;
import frc.robot.CatzSubsystems.SuperSubsystem.CatzSuperSubsystem;
import frc.robot.CatzSubsystems.SuperSubsystem.CatzSuperSubsystem.SuperstructureState;
import frc.robot.CatzSubsystems.SuperSubsystem.Elevator.CatzElevator;
import frc.robot.CatzSubsystems.SuperSubsystem.Elevator.CatzElevator.ElevatorPosition;
import frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot.CatzIntakePivot;
import frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot.CatzIntakePivot.IntakePivotPosition;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.CatzShooterPivot;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret.CatzShooterTurret;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.DriveAndRobotOrientationCmds.FaceTarget;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;

public class RobotContainer {

  // Subsystem Declaration
  private static CatzDrivetrain   drive                = new CatzDrivetrain();
  private static CatzShooterFeeder shooterFeeder       = new CatzShooterFeeder();
  private static CatzShooterFlywheels shooterFlywheels = new CatzShooterFlywheels();
  private static CatzIntakeRollers rollers             = new CatzIntakeRollers();

  // Superstructure member subsystem declaration
  private static CatzElevator     elevator             = new CatzElevator();
  private static CatzShooterTurret turret              = new CatzShooterTurret();
  private static CatzShooterPivot  shooterPivot        = new CatzShooterPivot();
  private static CatzIntakePivot   intakePivot         = new CatzIntakePivot();
  private static CatzSuperSubsystem superstructure = new CatzSuperSubsystem(elevator, turret, shooterPivot, intakePivot);

  // Assistance Subsystem declaration
  private static CatzLED          led = CatzLED.getInstance();
  private static CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private static CatzVision       vision       = new CatzVision(new VisionIO[] {
                                                              new VisionIOLimeLight("limelight-udon"),    //index 0 left
                                                              new VisionIOLimeLight("limelight-soba"),    //index 1 right
                                                              new VisionIOLimeLight("limelight-ramen")    //index 2 turret)
                                                              });
  // Drive Controller Declaration
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);

  // Alert Declaration
  private final Alert disconnectedAlertDrive = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert disconnectedAlertAux = new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final LoggedDashboardNumber endgameAlert1 = new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 = new LoggedDashboardNumber("Endgame Alert #2", 15.0);

  // Auto Declaration
  private CatzAutonomous auto = new CatzAutonomous(this);

  public RobotContainer() {

    // Drive And Aux Command Mapping
    configureBindings();

    // Endgame alert triggers
    new Trigger(
            () -> DriverStation.isTeleopEnabled()
                  && DriverStation.getMatchTime() > 0.0
                  && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get())
    ).onTrue(
        controllerRumbleCommand()
            .withTimeout(0.5)
            .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
            .finallyDo(() -> CatzLED.getInstance().endgameAlert = false)
    );
    new Trigger(
            () -> DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get())
    ).onTrue(
        controllerRumbleCommand()
            .withTimeout(0.2)
            .andThen(Commands.waitSeconds(0.1))
            .repeatedly()
            .withTimeout(0.9) // Rumble three times
            .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
            .finallyDo(() -> CatzLED.getInstance().endgameAlert = false)
    );
  }

  //---------------------------------------------------------------------------
  //
  //  Button mapping to commands
  //
  //---------------------------------------------------------------------------
  private void configureBindings() { // TODO organize by function
  
    xboxAux.rightBumper().whileTrue(rollers.setRollersIn());
    xboxAux.leftBumper().whileTrue(rollers.setRollersOut());

    // xboxAux.y().onTrue(superstructure.deployIntake());
    // xboxAux.leftTrigger().onTrue(superstructure.deployIntake());
    xboxAux.y().onTrue(superstructure.moveTurretToHome());

    xboxAux.a().onTrue(superstructure.setSuperStructureState(SuperstructureState.STOW));
    xboxAux.x().onTrue(superstructure.setSuperStructureState(SuperstructureState.INTAKE_GROUND));
    xboxAux.b().onTrue(superstructure.setSuperStructureState(SuperstructureState.SCORE_AMP));

    xboxDrv.b().onTrue(new FaceTarget(new Translation2d(0, 0), drive));

    xboxDrv.y().onTrue(auto.autoFindPathSpeakerLOT());

    xboxDrv.leftStick().onTrue(drive.cancelTrajectory());
    drive.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), 
                                               () -> xboxDrv.getLeftY(), 
                                               () -> xboxDrv.getRightX(), drive));
    //TODO add triggers to put default as priority    
  }

  /** Creates a controller rumble command with specified rumble and controllers */
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          xboxDrv.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          xboxAux.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          xboxDrv.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          xboxAux.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  //---------------------------------------------------------------------------
  //
  //  Misc methods
  //
  //---------------------------------------------------------------------------
  /** Updates dashboard data. */ //TODO if needed add values here
  public void updateDashboardOutputs() {

  }

  /** Updates the alerts. */
  public void updateAlerts() {

  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    disconnectedAlertDrive.set(
        !DriverStation.isJoystickConnected(xboxDrv.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxDrv.getHID().getPort())
    );
    disconnectedAlertAux.set(
        !DriverStation.isJoystickConnected(xboxAux.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxAux.getHID().getPort())
    );
    disconnectedAlertAux.setAlertOnloop(true, 1.0, 10.0);
  }

  //---------------------------------------------------------------------------
  //
  //      Subsystem getters
  //
  //---------------------------------------------------------------------------
  public CatzDrivetrain getCatzDrivetrain() {
    return drive;
  }

  public CatzSuperSubsystem getCatzSuperstructure() {
    return superstructure;
  }

  public CatzElevator getCatzElevator() {
    return elevator;
  }

  public CatzShooterFeeder getCatzShooterFeeder() {
    return shooterFeeder;
  }

  public CatzShooterFlywheels getCatzShooterFlywheels() {
    return shooterFlywheels;
  }

  public CatzIntakeRollers getCatzIntakeRollers() {
    return rollers;
  }

  public CatzIntakePivot getCatzIntakePivot() {
    return intakePivot;
  }

  public CatzAutonomous getCatzAutonomous(){
    return auto;
  }

  public Command getAutonomousCommand() {
    return auto.getCommand();
  }
}
