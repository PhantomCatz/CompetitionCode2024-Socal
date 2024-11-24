package frc.robot.Autonomous;

import java.nio.file.Path;
import java.util.Arrays;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.Shooter.ShooterFeeder.CatzShooterFeeder;
import frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.CatzShooterFlywheels;
import frc.robot.CatzSubsystems.SuperSubsystem.CatzSuperSubsystem;
import frc.robot.CatzSubsystems.SuperSubsystem.CatzSuperSubsystem.SuperstructureState;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.CharacterizationCmds.FeedForwardCharacterization;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization;
import frc.robot.Commands.CharacterizationCmds.WheelRadiusCharacterization.Direction;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.WaitUntilPassX;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.JSONUtil;

public class CatzAutonomous {
    private final int MAX_QUESTIONS = 5;

    private static LoggedDashboardChooser<Command> autoPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");
    private RobotContainer m_container;
    private boolean trajectoriesLoaded = false;
    private JSONParser parser = new JSONParser();

    private HashMap<String, DashboardCmd> dashboardCmds = new HashMap<>();
    private File choreoPathsDirectory = new File(Filesystem.getDeployDirectory(), "choreo");
    private File pathplannerPathsDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
    private File autosDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    private String lastAutoName = null;

    public CatzAutonomous(RobotContainer container) {

        this.m_container = container;

        // Path follwing setup
        CatzRobotTracker tracker = CatzRobotTracker.getInstance();
        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
            DriveConstants.driveConfig.maxLinearVelocity() / 2, 
            DriveConstants.driveConfig.driveBaseRadius(),   
            new ReplanningConfig()
        );
        BooleanSupplier shouldFlip = ()->AllianceFlipUtil.shouldFlipToRed();
        AutoBuilder.configureHolonomic(
            tracker::getEstimatedPose,
            tracker::resetPose,
            tracker::getRobotChassisSpeeds,
            container.getCatzDrivetrain()::drive,
            config,
            shouldFlip,
            container.getCatzDrivetrain()
        );
        //------------------------------------------------------------------------------------------------------------
        // Autonmous questionaire gui configurations
        // ORDER MATTERS! Register named commands first, configure questionaire second, and add autos to dashboard last
        //------------------------------------------------------------------------------------------------------------
        //------------------------------------------------------------------------------------------------------------
        // Path Configuration
        //------------------------------------------------------------------------------------------------------------
        for(File pathFile : choreoPathsDirectory.listFiles()){
            //to get rid of the extensions trailing the path names
            String pathName = pathFile.getName().replaceFirst("[.][^.]+$", ""); 
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
            NamedCommands.registerCommand(pathName, new TrajectoryDriveCmd(path, container.getCatzDrivetrain()));
        }
        for(File pathFile : pathplannerPathsDirectory.listFiles()){
            //to get rid of the extensions trailing the path names
            String pathName = pathFile.getName().replaceFirst("[.][^.]+$", ""); 
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            List<EventMarker> eventMarkers = path.getEventMarkers();
            NamedCommands.registerCommand(pathName, new ParallelCommandGroup(
                                                            new TrajectoryDriveCmd(path, container.getCatzDrivetrain()),
                                                            new EventMarkerHelperCmd(eventMarkers, path.getAllPathPoints())
                                                    ));
        }
        //----------------------------------------------------------------------------------------
        //  Named Command registration
        //----------------------------------------------------------------------------------------
        NamedCommands.registerCommand("TestPrint", new PrintCommand("Benchmark"));
        NamedCommands.registerCommand("ReturnToScore", autoFindPathSpeaker());
        NamedCommands.registerCommand("Intake", AutomatedSequenceCmds.noteDetectIntakeToShooter(container));
    
        //---------------------------------------------------------------------------
        // Far side auto Path Configuration
        //---------------------------------------------------------------------------
        HashMap<String, Command> farSideScoringChoices = new HashMap<>();

        farSideScoringChoices.put("Bottom GP", NamedCommands.getCommand("CollectGP1"));
        farSideScoringChoices.put("1 Up GP", NamedCommands.getCommand("ColletGP2"));
        farSideScoringChoices.put("2 Up GP", NamedCommands.getCommand("ColletGP3"));
        farSideScoringChoices.put("Do Nothing", new PrintCommand("Skipped"));
        dashboardCmds.put("Score1FarSide", new DashboardCmd("Bottom or 1 up GP?", farSideScoringChoices));
        dashboardCmds.put("Score2FarSide", new DashboardCmd("Bottom or 1 up GP?", farSideScoringChoices));
        dashboardCmds.put("Score3FarSide", new DashboardCmd("Score 1 More?", farSideScoringChoices));

        //---------------------------------------------------------------------------
        // Speaker side auto Path Configuration
        //---------------------------------------------------------------------------
        Command wingOptionTop = NamedCommands.getCommand("Wing Option Top");

        HashMap<String, Command> spSdScoringChoices = new HashMap<>();
        spSdScoringChoices.put("Top GP", wingOptionTop);
        spSdScoringChoices.put("Mid GP", NamedCommands.getCommand("Wing Option Mid"));
        spSdScoringChoices.put("Do Nothing", new PrintCommand("Skipped"));
        dashboardCmds.put("Score1", new DashboardCmd("Top or Mid GP?", spSdScoringChoices));
        dashboardCmds.put("Score2", new DashboardCmd("Top or Mid GP?", spSdScoringChoices));
        dashboardCmds.put("Score3", new DashboardCmd("Scoring Position 3?", spSdScoringChoices));

        //---------------------------------------------------------------------------
        //  Sping Auto Conifig
        //---------------------------------------------------------------------------
        HashMap<String, Command> moveOptions = new HashMap<>();
        moveOptions.put("Spin", NamedCommands.getCommand("TurnStraight"));
        moveOptions.put("Move", NamedCommands.getCommand("DriveStraight"));
        dashboardCmds.put("SpinOrMove", new DashboardCmd("Spin or Move?", moveOptions));

        //---------------------------------------
        HashMap<String, Command> pathOptions = new HashMap<>();
        pathOptions.put("CurveTurn", NamedCommands.getCommand("CurveTurn"));
        pathOptions.put("DriveStraight", NamedCommands.getCommand("DriveStraight"));
        dashboardCmds.put("CurveOrStraight", new DashboardCmd("Curve or Turn?", pathOptions));

        dashboardCmds.forEach((k, v) -> {
            NamedCommands.registerCommand(k, v);
        });
        
        for (File autoFile: autosDirectory.listFiles()){
            String autoName = autoFile.getName().replaceFirst("[.][^.]+$", "");
            autoPathChooser.addDefaultOption(autoName, new PathPlannerAuto(autoName));
        }

    }

    public void updateQuestionaire(){
        try {
            String autoName = autoPathChooser.get().getName() + ".auto";
            JSONObject json = (JSONObject) parser.parse(new FileReader(Filesystem.getDeployDirectory()+"/pathplanner/autos/" + autoName));

            if (!autoName.equals(lastAutoName)){
                lastAutoName = autoName;

                for(int i=1; i<=MAX_QUESTIONS; i++){
                    String questionName = "Question " + String.valueOf(i);
                    SmartDashboard.putString(questionName, "");
                    SmartDashboard.putData(questionName + " Response", new SendableChooser<Command>());
                }

                //im sorry
                ArrayList<Object> commands = JSONUtil.getCommandsFromPath(json);
                int questionCounter = 1;

                for(Object o : commands){
                    String commandName = JSONUtil.getCommandName(o);
                    DashboardCmd modifiableCommand = dashboardCmds.get(commandName);
                    
                    if(modifiableCommand != null){
                        String questionName = "Question " + String.valueOf(questionCounter);
                        SmartDashboard.putString(questionName, modifiableCommand.getQuestion());
                        SmartDashboard.putData(questionName + " Response", modifiableCommand.getChooser());
                        questionCounter += 1;
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    //---------------------------------------------------------------------------------------------------------
    //
    //          Characteration Routines
    //
    //---------------------------------------------------------------------------------------------------------
    public Command flywheelCharacterization() {
        CatzShooterFlywheels flywheels = m_container.getCatzShooterFlywheels();
        return new FeedForwardCharacterization(flywheels, flywheels::runCharacterization, flywheels::getCharacterizationVelocity)
                        .withName("Flywheels characterization");
    }
    

    //Automatic pathfinding command
    public Command autoFindPathAmp() {
        return Commands.either(
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(1.89, 7.76, Rotation2d.fromDegrees(90)), DriveConstants.autoPathfindingConstraints), 
            AutoBuilder.pathfindToPose(new Pose2d(1.89, 7.76, Rotation2d.fromDegrees(90)), DriveConstants.autoPathfindingConstraints), 
            ()->AllianceFlipUtil.shouldFlipToRed());

    }

    public Command autoFindPathSpeaker() {
        return Commands.either(
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(2.74, 6.14, Rotation2d.fromDegrees(180)), DriveConstants.autoPathfindingConstraints), 
            AutoBuilder.pathfindToPose(new Pose2d(2.74, 6.14, Rotation2d.fromDegrees(180)), DriveConstants.autoPathfindingConstraints), 
            ()->AllianceFlipUtil.shouldFlipToRed());
    }
    //---------------------------------------------------------------------------------------------------------
    //
    //          Trajectory Helpers
    //
    //---------------------------------------------------------------------------------------------------------
    private void preloadTrajectoryClass(PathPlannerPath segment) {
        // This is done because Java loads classes lazily. Calling this here loads the trajectory pathplanner class which
        // is used to follow paths and saves user code ms loop time at the start of auto.
        if (!trajectoriesLoaded) {
            trajectoriesLoaded = true;
            var trajectory = new PathPlannerTrajectory(
                segment,
                CatzRobotTracker.getInstance().getRobotChassisSpeeds(),
                CatzRobotTracker.getInstance().getEstimatedPose().getRotation());
        }
    }
    /** Getter for final autonomous routine */
    public Command getCommand() { 
        return autoPathChooser.get();
    }
}