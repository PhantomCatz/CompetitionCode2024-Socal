package frc.robot.Autonomous;

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
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.CatzShooterFlywheels;
import frc.robot.Commands.CharacterizationCmds.FeedForwardCharacterization;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.JSONUtil;

public class CatzAutonomous {
    private final int MAX_QUESTIONS = 5;

    private static LoggedDashboardChooser<Command> autoPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");
    private RobotContainer m_container;
    private boolean trajectoriesLoaded = false;
    private JSONParser parser = new JSONParser();

    private HashMap<String, ModifiableCmd> modifiableCmds = new HashMap<>();
    private File pathsDirectory = new File(Filesystem.getDeployDirectory(), "choreo");
    private File autosDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    private String lastAutoName = null;

    public CatzAutonomous(RobotContainer container) {
        this.m_container = container;

        // Path follwing setup
        CatzRobotTracker tracker = CatzRobotTracker.getInstance();
        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
            DriveConstants.driveConfig.maxLinearVelocity(), 
            DriveConstants.driveConfig.driveBaseRadius(),   
            new ReplanningConfig()
        );
        BooleanSupplier shouldFlip = ()->AllianceFlipUtil.shouldFlipToRed();
        AutoBuilder.configureHolonomic(
            tracker::getEstimatedPose,
            tracker::resetPosition,
            tracker::getRobotChassisSpeeds,
            container.getCatzDrivetrain()::drive,
            config,
            shouldFlip,
            container.getCatzDrivetrain()
        );

        // Questionaire configuration
        HashMap<String, Command> scoringPositions = new HashMap<>();
        scoringPositions.put("High", new PrintCommand("High"));
        scoringPositions.put("Mid", new PrintCommand("Mid"));
        scoringPositions.put("Low", new PrintCommand("Low"));
        modifiableCmds.put("Score1", new ModifiableCmd("Scoring Position 1?", scoringPositions));
        modifiableCmds.put("Score2", new ModifiableCmd("Scoring Position 2?", scoringPositions));
        modifiableCmds.put("Score3", new ModifiableCmd("Scoring Position 3?", scoringPositions));
        modifiableCmds.put("Score4", new ModifiableCmd("Scoring Position 4?", scoringPositions));

        modifiableCmds.forEach((k, v) -> {
            NamedCommands.registerCommand(k, v);
        });
        for(File pathFile : pathsDirectory.listFiles()){
            //to get rid of the extensions trailing the path names
            String pathName = pathFile.getName().replaceFirst("[.][^.]+$", ""); 
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
            NamedCommands.registerCommand(pathName, new TrajectoryDriveCmd(path, container.getCatzDrivetrain()));
        }
        
        HashMap<String, Command> moveOptions = new HashMap<>();
        moveOptions.put("Spin", NamedCommands.getCommand("Spin"));
        moveOptions.put("Move", NamedCommands.getCommand("Choreo"));
        modifiableCmds.put("SpinOrMove", new ModifiableCmd("Spin or Move?", moveOptions));
        
        modifiableCmds.forEach((k, v) -> {
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
                    ModifiableCmd modifiableCommand = modifiableCmds.get(commandName);
                    
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
    public Command autoFindPathSpeakerLOT() {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(180)),
                new Pose2d(1.50, 0.69, Rotation2d.fromDegrees(235))
                    );

        //send path info to trajectory following command
        return new TrajectoryDriveCmd(bezierPoints, 
                                      DriveConstants.autoPathfindingConstraints, 
                                      new GoalEndState(0.0, Rotation2d.fromDegrees(235)), m_container.getCatzDrivetrain());
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