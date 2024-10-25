package frc.robot.Commands.DriveAndRobotOrientationCmds;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.Utilities.AllianceFlipUtil;

public class TrajectoryDriveCmd extends Command {

    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = 5;

    private CatzDrivetrain m_driveTrain;
    private PathPlannerTrajectory trajectory;
    private HolonomicDriveController hocontroller;
    
    private final Timer timer = new Timer();
    private final double TIMEOUT_SCALAR = 5;
    private PathPlannerPath path;

    private boolean atTarget = false;
    public static double pathTimeOut = -999.0;


    //Command inside trajectory helper variables
    private List<Double> waypointsRatios = Arrays.asList();
    private List<Command> m_commands     = Arrays.asList();
    private int numConsecutiveWaypointCounter = 0;
    private double scaledWaypointTime = 0.0;
    private Command cmd = new InstantCommand();


    private boolean executing = false;
    private boolean done = false;

    //Constructor Logger
    private int m_constructorLogger = 1; // For determining if the command is auto path find or autonomous


    /**
     * @param drivetrain           The coordinator between the gyro and the swerve modules.
     * @param trajectory          The trajectory to follow.
     */
    public TrajectoryDriveCmd(PathPlannerPath newPath, CatzDrivetrain drivetrain, List<Double> waypoints, List<Command> commands, int constructorLogger) {
        waypointsRatios = waypoints;
        m_commands = commands;
        path = newPath;
        m_driveTrain = drivetrain;
        m_constructorLogger = constructorLogger;
        addRequirements(m_driveTrain);
    }
    

    //Auto Pathplanning trajectoreies
    public TrajectoryDriveCmd(List<Translation2d> bezierPoints, 
                              PathConstraints constraints, 
                              GoalEndState endRobotState,
                              CatzDrivetrain drivetrain,
                              int constructorLogger) {
        PathPlannerPath newPath = new PathPlannerPath(bezierPoints, constraints, endRobotState);
        path = newPath;
        m_driveTrain = drivetrain;
        m_constructorLogger = constructorLogger;
        addRequirements(m_driveTrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        //it is necessary to make a new instance of holonomic controller to clear the memory so kD doesn't explode in the first frame due to discontinuous function
        hocontroller = DriveConstants.getNewHolController();
        
        PathPlannerPath usePath = path;
        if(AllianceFlipUtil.shouldFlipToRed()) {
            usePath = path.flipPath();
        }

        if(m_constructorLogger == 1) {
            CatzRobotTracker.getInstance().resetPosition(usePath.getPreviewStartingHolonomicPose());
        }
        this.trajectory = new PathPlannerTrajectory(
            usePath, 
            DriveConstants.
                swerveDriveKinematics.
                    toChassisSpeeds(CatzRobotTracker.getInstance().getRobotSwerveModuleStates()),
            CatzRobotTracker.getInstance().getRobotRotation()
        );
                                               
        pathTimeOut = trajectory.getTotalTimeSeconds() * TIMEOUT_SCALAR; //TODO do we still need this
        numConsecutiveWaypointCounter = 0;
        scaledWaypointTime = 0.0;
        executing = false;
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();

        // Trajectory Executor
        if(!atTarget){
    
            // Getters from pathplanner and current robot pose
            PathPlannerTrajectory.State goal = trajectory.sample(Math.min(currentTime, trajectory.getTotalTimeSeconds()));
            Rotation2d targetOrientation     = goal.targetHolonomicRotation;
            Pose2d currentPose               = CatzRobotTracker.getInstance().getEstimatedPose();
                
            /* 
            * Convert PP trajectory into a wpilib trajectory type 
            * Only takes in the current robot position 
            * Does not take acceleration to be used with the internal WPILIB trajectory library
            */
            Trajectory.State state = new Trajectory.State(currentTime, 
                                                          goal.velocityMps,  //made the holonomic drive controller only rely on its current position, not its velocity because the target velocity is used as a ff
                                                          0.0, 
                                                          new Pose2d(goal.positionMeters, goal.heading  ),
                                                          goal.curvatureRadPerMeter);
    
            //construct chassisspeeds
            ChassisSpeeds adjustedSpeeds = hocontroller.calculate(currentPose, state, targetOrientation);

            //send to drivetrain
            m_driveTrain.drive(adjustedSpeeds, true);
            CatzRobotTracker.getInstance().addTrajectorySetpointData(goal.getTargetHolonomicPose());

            Logger.recordOutput("CatzRobotTracker/Desired Auto Pose", new Pose2d(state.poseMeters.getTranslation(), goal.targetHolonomicRotation));


        }else{
            m_driveTrain.stopDriving();
        }


        // Command Executer
        if(!waypointsRatios.isEmpty()) {
            if(numConsecutiveWaypointCounter < waypointsRatios.size()) {
                scaledWaypointTime = waypointsRatios.get(numConsecutiveWaypointCounter);  
                cmd = m_commands.get(numConsecutiveWaypointCounter);   
                if(executing == false) {
                    if(currentTime > scaledWaypointTime){
                        executing = true;
                        cmd.initialize();
                        System.out.println(scaledWaypointTime);

                    }
                }
            

                if(executing) {
                    m_commands.get(numConsecutiveWaypointCounter).execute();
                    if(cmd.isFinished() || (currentTime >= scaledWaypointTime + 2.0)){
                        done = true;
                        cmd.end(true);
                        executing = false;
                        numConsecutiveWaypointCounter++;
                    }
                }
            }
        }



    }

    /*
     * For Debugging Purposes 
     * Keep them commmented ALWAYS if you are not using it 
     */
    public void debugLogsTrajectory(){
        //Logger.recordOutput("Desired Auto Pose", new Pose2d(state.poseMeters.getTranslation(), goal.targetHolonomicRotation));
        //Logger.recordOutput("Adjusted Speeds X", adjustedSpeeds.vxMetersPerSecond);
        //Logger.recordOutput("Adjusted Speeds Y", adjustedSpeeds.vyMetersPerSecond);
        //Logger.recordOutput("Trajectory Goal MPS", state.velocityMetersPerSecond);
        //Logger.recordOutput("PathPlanner Goal MPS", goal.velocityMps);

        //System.out.println(goal.getTargetHolonomicPose());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop(); // Stop timer
        m_driveTrain.stopDriving();
        System.out.println("trajectory done");
    }

    @Override
    public boolean isFinished() {
        // Finish command if the total time the path takes is over
        double currentPosX =        CatzRobotTracker.getInstance().getEstimatedPose().getX();
        double currentPosY =        CatzRobotTracker.getInstance().getEstimatedPose().getY();
        double currentRotation =    CatzRobotTracker.getInstance().getEstimatedPose().getRotation().getDegrees();

        double desiredPosX =        trajectory.getEndState().positionMeters.getX();
        double desiredPosY =        trajectory.getEndState().positionMeters.getY();
        double desiredRotation =    trajectory.getEndState().targetHolonomicRotation.getDegrees();

        double xError =        Math.abs(desiredPosX - currentPosX);
        double yError =        Math.abs(desiredPosY - currentPosY);
        double rotationError = Math.abs(desiredRotation - currentRotation);
        if (rotationError > 180){
            rotationError = 360-rotationError;
        }

        atTarget = (xError < ALLOWABLE_POSE_ERROR && 
                    yError < ALLOWABLE_POSE_ERROR && 
                    rotationError < ALLOWABLE_ROTATION_ERROR);

        return atTarget || timer.hasElapsed(pathTimeOut);
    } 

}