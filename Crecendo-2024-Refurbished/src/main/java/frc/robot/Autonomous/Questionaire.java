package frc.robot.Autonomous;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;

public class Questionaire {
    //First key is the start pose, second key is the destination, and the value is the actual autonomous routine.
    private HashMap<String, Command> questionairePath;
    private String[] positions = {"left", "mid", "right"};

    private LoggedDashboardChooser<String> startPosChooser = new LoggedDashboardChooser<>("Choose Start Position");
    private LoggedDashboardChooser<String> endPosChooser = new LoggedDashboardChooser<>("Choose End Position");

    private RobotContainer container;

    public Questionaire(RobotContainer container){
        this.container = container;
        questionairePath = new HashMap<>();
        initializeMap();

        for(String s : positions){
            startPosChooser.addOption(s,s);
            endPosChooser.addOption(s,s);
        }
    }

    public Command getSelectedPath(){
        String pathName = startPosChooser.get() + " " + endPosChooser.get();
        return questionairePath.get(pathName);
    }

    public void initializeMap(){
        for(String p1: positions){
            for(String p2: positions){
                String pathName = p1.concat(" " + p2);
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                
               // questionairePath.put(pathName, new TrajectoryDriveCmd(path, container.getCatzDrivetrain())); //TODO fix later
            }
        }
    }
}
