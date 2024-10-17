package frc.robot.Commands.DriveAndRobotOrientationCmds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;

public class FaceTarget extends Command{
    private final double kP = 0.1;
    private final double MIN_ERROR = 2; // degrees
    private final double TIMEOUT = 3;

    private CatzRobotTracker tracker = CatzRobotTracker.getInstance();
    private CatzDrivetrain drivetrain;

    private Rotation2d rotDif = new Rotation2d();
    private Translation2d target;
    private Timer timer;

    public FaceTarget(Translation2d target, CatzDrivetrain drivetrain){
        this.target = target;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize(){
        timer.reset();
    }

    @Override
    public void execute(){
        Pose2d curPose = tracker.getEstimatedPose();
        Translation2d posDif = target.minus(curPose.getTranslation());
        rotDif = Rotation2d.fromRadians(Math.atan2(posDif.getY(), posDif.getX())).minus(curPose.getRotation());

        drivetrain.drive(new ChassisSpeeds(0, 0, rotDif.getDegrees() * kP));
    }
    
    @Override
    public boolean isFinished(){
        return Math.abs(rotDif.getDegrees()) < MIN_ERROR || timer.get() > TIMEOUT;
    }
}
