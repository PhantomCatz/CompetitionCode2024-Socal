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
import com.pathplanner.lib.commands.FollowPathWithEvents;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
import frc.robot.Utilities.SwitchableChooser;
import frc.robot.Utilities.VirtualSubsystem;

public class CatzAutonomous extends VirtualSubsystem{
    private final int MAX_QUESTIONS = 5;
    private static final AutoRoutine defaultRoutine =
                new AutoRoutine("Do Nothing", List.of(), Commands.none());
    private static final String AUTO_STRING = "Auto";

    private static LoggedDashboardChooser<Command> autoPathChooser = new LoggedDashboardChooser<>("Choosen Auto Routine");
    private final LoggedDashboardChooser<AutoRoutine> routineChooser = new LoggedDashboardChooser<>(AUTO_STRING + "/Routine");
    private final List<StringPublisher> questionPublishers;
    private final List<SwitchableChooser> questionChoosers;
    private RobotContainer m_container;
    private boolean trajectoriesLoaded = false;
    private JSONParser parser = new JSONParser();

    private HashMap<String, Command> dashboardCmds = new HashMap<>();
    private HashMap<String, Command> dashboardSelectors = new HashMap<>();
    private File choreoPathsDirectory = new File(Filesystem.getDeployDirectory(), "choreo");
    private File pathplannerPathsDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
    private File autosDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    private String lastAutoName = null;

    private AutoRoutine lastRoutine;
    private List<AutoQuestionResponse> lastResponses;

    public CatzAutonomous(RobotContainer container) {
        this.m_container = container;
        routineChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
        lastRoutine = defaultRoutine;
        lastResponses = List.of();

        // Publish questions and choosers
        questionPublishers = new ArrayList<>();
        questionChoosers = new ArrayList<>();
        for (int i = 0; i < MAX_QUESTIONS; i++) {
        var publisher =
            NetworkTableInstance.getDefault()
                .getStringTopic("/SmartDashboard/" + AUTO_STRING + "/Question #" + Integer.toString(i + 1))
                .publish();
        publisher.set("NA");
        questionPublishers.add(publisher);
        questionChoosers.add(
            new SwitchableChooser(AUTO_STRING + "/Question #" + Integer.toString(i + 1) + " Chooser"));
        }

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
        //----------------------------------------------------------------------------------------
        //  Named Command registration
        //----------------------------------------------------------------------------------------
        NamedCommands.registerCommand("Intake", AutomatedSequenceCmds.noteDetectIntakeToShooter(container));

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
            NamedCommands.registerCommand(pathName, new TrajectoryDriveCmd(path, container.getCatzDrivetrain()));

            // Named Command registration
            NamedCommands.registerCommand("noteDetectIntakeToShooter", AutomatedSequenceCmds.noteDetectIntakeToShooter(container));
            NamedCommands.registerCommand("autonSpeakerShoot", AutomatedSequenceCmds.autonSpeakerShoot(container));

            NamedCommands.registerCommand("TestPrint", Commands.print("Bench"));

        }


    
        // //---------------------------------------------------------------------------
        // // Far side auto Path Configuration
        // //---------------------------------------------------------------------------
        // HashMap<String, Command> farSideScoringChoices = new HashMap<>();

        // farSideScoringChoices.put("Bottom GP", NamedCommands.getCommand("CollectGP1"));
        // farSideScoringChoices.put("1 Up GP", NamedCommands.getCommand("ColletGP2"));
        // farSideScoringChoices.put("2 Up GP", NamedCommands.getCommand("ColletGP3"));
        // farSideScoringChoices.put("Do Nothing", new PrintCommand("Skipped"));
        // dashboardCmds.put("Score1FarSide", new DashboardCmd("Bottom or 1 up GP?", farSideScoringChoices));
        // dashboardCmds.put("Score2FarSide", new DashboardCmd("Bottom or 1 up GP?", farSideScoringChoices));
        // dashboardCmds.put("Score3FarSide", new DashboardCmd("Score 1 More?", farSideScoringChoices));

        //---------------------------------------------------------------------------
        // Speaker side auto Path Configuration
        //---------------------------------------------------------------------------
        HashMap<String, Command> speakerSidefinalChoices = new HashMap<>();
        speakerSidefinalChoices.put("2 GP side", NamedCommands.getCommand("Wing Scoring 1"));
        speakerSidefinalChoices.put("2 GP subwoofer", NamedCommands.getCommand("Wing Option Mid"));
        speakerSidefinalChoices.put("3 GP side", NamedCommands.getCommand("Wing Option Mid"));
        speakerSidefinalChoices.put("3 GP subwoofer", NamedCommands.getCommand("Wing Option Mid"));


        dashboardCmds.put("Score1", speakerSideQuestionSort(speakerSidefinalChoices));


        // //---------------------------------------------------------------------------
        // //  Sping Auto Conifig
        // //---------------------------------------------------------------------------
        // HashMap<String, Command> moveOptions = new HashMap<>();
        // moveOptions.put("Spin", NamedCommands.getCommand("TurnStraight"));
        // moveOptions.put("Move", NamedCommands.getCommand("DriveStraight"));
        // dashboardCmds.put("SpinOrMove", new DashboardCmd("Spin or Move?", moveOptions));

        // //---------------------------------------
        // HashMap<String, Command> pathOptions = new HashMap<>();
        // pathOptions.put("CurveTurn", NamedCommands.getCommand("CurveTurn"));
        // pathOptions.put("DriveStraight", NamedCommands.getCommand("DriveStraight"));
        // dashboardCmds.put("CurveOrStraight", new DashboardCmd("Curve or Turn?", pathOptions));

        dashboardCmds.forEach((k, v) -> {
            NamedCommands.registerCommand(k, v);
        });
        
        for (File autoFile: autosDirectory.listFiles()){
            String autoName = autoFile.getName().replaceFirst("[.][^.]+$", "");
            if(autoName.contains("Questionaire")) {
                List<AutoQuestion> questions = determineAutoQuestions(autoName);
                addRoutine(autoName, questions, new PathPlannerAuto(autoName));
            } else {
                addRoutine(autoName, new PathPlannerAuto(autoName));
            }
            autoPathChooser.addDefaultOption(autoName, new PathPlannerAuto(autoName));
        }

    }

    @Override
    public void periodic() {
        // Skip updates when actively running in auto
        if (DriverStation.isAutonomousEnabled() && lastRoutine != null && lastResponses == null) {
            return;
        }
        // Update the list of questions
        var selectedRoutine = routineChooser.get();
        if (selectedRoutine == null) {
            return;
        }

        if (!selectedRoutine.equals(lastRoutine)) {
        var questions = selectedRoutine.questions();
        for (int i = 0; i < MAX_QUESTIONS; i++) {
            if (i < questions.size()) {
            questionPublishers.get(i).set(questions.get(i).question());
            questionChoosers
                .get(i)
                .setOptions(
                    questions.get(i).responses().stream()
                        .map((AutoQuestionResponse response) -> response.toString())
                        .toArray(String[]::new));
            } else {
            questionPublishers.get(i).set("");
            questionChoosers.get(i).setOptions(new String[] {});
            }
        }
        }

        // Update the routine and responses
        lastRoutine = selectedRoutine;
        lastResponses = new ArrayList<>();
        for (int i = 0; i < lastRoutine.questions().size(); i++) {
        String responseString = questionChoosers.get(i).get();
        lastResponses.add(
            responseString == null
                ? lastRoutine.questions().get(i).responses().get(0)
                : AutoQuestionResponse.valueOf(responseString));
        }

        // try {
        //     // collect autoname for this iteration
        //     String autoName = autoPathChooser.get().getName() + ".auto";
        //     JSONObject json = (JSONObject) parser.parse(new FileReader(Filesystem.getDeployDirectory()+"/pathplanner/autos/" + autoName));

        //     if (!autoName.equals(lastAutoName)){
        //         lastAutoName = autoName;

        //         // Determine question boxes
        //         for(int i=1; i<=MAX_QUESTIONS; i++){
        //             String questionName = "Question " + String.valueOf(i);
        //             SmartDashboard.putString(questionName, "");
        //             SmartDashboard.putData(questionName + " Response", new SendableChooser<Command>());
        //         }

        //         //Collect question boxes assiciated with auto
        //         ArrayList<Object> commands = JSONUtil.getCommandsFromPath(json);
        //         int questionCounter = 1;

        //         // Fill question Boxes
        //         for(Object o : commands){
        //             String commandName = JSONUtil.getCommandName(o);
        //             DashboardCmd modifiableCommand = dashboardCmds.get(commandName);
                    
        //             if(modifiableCommand != null){
        //                 String questionName = "Question " + String.valueOf(questionCounter);
        //                 SmartDashboard.putString(questionName, modifiableCommand.getQuestion());
        //                 SmartDashboard.putData(questionName + " Response", modifiableCommand.getChooser());
        //                 questionCounter += 1;
        //             }
        //         }
        //     }
        // } catch (Exception e) {
        //     e.printStackTrace();
        // }
    }
    //---------------------------------------------------------------------------------------------------------
    //
    //          Chooser helpers
    //
        //---------------------------------------------------------------------------------------------------------
    /** Registers a new auto routine that can be selected. */
    private void addRoutine(String name, Command command) {
        addRoutine(name, List.of(), command);
    }

    /** Registers a new auto routine that can be selected. */
    private void addRoutine(String name, List<AutoQuestion> questions, Command command) {
        if (questions.size() > MAX_QUESTIONS) {
        throw new RuntimeException(
            "Auto routine contained more than "
                + Integer.toString(MAX_QUESTIONS)
                + " questions: "
                + name);
        }
        routineChooser.addOption(name, new AutoRoutine(name, questions, command));
    }

    public List<AutoQuestion> determineAutoQuestions(String autoName) {
        List<AutoQuestion> autoQuestions = List.of();
        if(autoName.contains("SpeakerSide")) {
            autoQuestions = List.of(
                new AutoQuestion(
                    "Starting location?",
                    List.of(
                        AutoQuestionResponse.AMP,
                        AutoQuestionResponse.CENTER)),
                new AutoQuestion(
                    "How many spike notes?",
                    List.of(AutoQuestionResponse.TWO, AutoQuestionResponse.THREE)),
                new AutoQuestion(
                    "First center note?", List.of(AutoQuestionResponse.THINKING_ON_YOUR_FEET)),
                new AutoQuestion(
                    "Second center note?", List.of(AutoQuestionResponse.THINKING_ON_YOUR_FEET))
            );
        }

        return autoQuestions;
    }

    public Command speakerSideQuestionSort(HashMap<String, Command> choices) {
        return NamedCommands.getCommand("Wing Scoring 1");
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
    
    //---------------------------------------------------------------------------------------------------------
    //
    //          Auto Driving
    //
    //---------------------------------------------------------------------------------------------------------
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


    //---------------------------------------------------------------------------------------------------------
    //
    //          Record Types
    //
    //---------------------------------------------------------------------------------------------------------
    /** A customizable auto routine associated with a single command. */
    private static final record AutoRoutine(
        String name, List<AutoQuestion> questions, Command command) {}

    /** A question to ask for customizing an auto routine. */
    public static record AutoQuestion(String question, List<AutoQuestionResponse> responses) {}

    /** Responses to auto routine questions. */
    public static enum AutoQuestionResponse {
        AMP,
        CENTER,
        SOURCE,
        ONE,
        TWO,
        THREE,
        SOURCE_WALL,
        SOURCE_MIDDLE,
        MIDDLE,
        AMP_MIDDLE,
        AMP_WALL,
        SCORE_POOPED,
        FOURTH_CENTER,
        THINKING_ON_YOUR_FEET,
        IMMEDIATELY,
        SIX_SECONDS,
        LAST_SECOND,
        YES,
        NO
    }


}