// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveAndRobotOrientation;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.CatzSwerveModule;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.CatzVision;
import frc.robot.util.CatzMathUtils;
import frc.robot.util.FieldRelativeAccel;
import frc.robot.util.FieldRelativeSpeed;

public class CatzRobotTracker {

  private static CatzRobotTracker instance;

  private static final double POSE_BUFFER_SIZE_SECONDS = 2.0;


  // Pose Estimation Members
  private Pose2d odometryPose = new Pose2d();

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
          TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SECONDS);
  // Swerve drive pose estimator for tracking robot pose
  private static SwerveDrivePoseEstimator m_poseEstimator;

  private double lastGyroAngularVelocity = 0.0;
  private double lastAccelerationXFromNavX = 0.0;
  private double lastAccelerationYFromNavX = 0.0;

  private Optional<Pose2d> speakerCameraPose = Optional.empty();
  private double speakerHorizontalDistance = Double.NaN;
  private Optional<Pose2d> robotPoseFromCameraPose = Optional.empty();
  private double visionHorizontalDistance = 0.0;

  private boolean hasTarget;

  protected Twist2d robotAccelerations = new Twist2d();
  protected SwerveDriveOdometry odometry;

  private SwerveDriveWheelPositions lastWheelPositions = new SwerveDriveWheelPositions(new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
  });

  private SwerveModuleState[] moduleStates = new SwerveModuleState[] {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
  };

  private Rotation2d lastGyroAngle = new Rotation2d();
  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
  private final SwerveDriveKinematics kinematics;

  private double xyStdDev;

  private CatzRobotTracker() {
        kinematics = DriveConstants.swerveDriveKinematics;

        // Initialize the swerve drive pose estimator
        m_poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(), 
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            }, 
            odometryPose, 
            VecBuilder.fill(1, 1, 0.7),  //odometry standard devs
            VecBuilder.fill(5, 5, 99999.0) //vision pose estimators standard dev are increase x, y, rotatinal radians values to trust vision less           
        ); 

        odometry = new SwerveDriveOdometry(
            kinematics, 
            new Rotation2d(),             
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            }, 
            odometryPose);
        
  }

  public static CatzRobotTracker getInstance() {
    if(instance == null) instance = new CatzRobotTracker();
    return instance;
  }




  public void addVisionObservation(List<CatzVision.PoseAndTimestamp> visionFrameUpdates) {
    //------------------------------------------------------------------------------------------------
    // Vison pose updating
    //------------------------------------------------------------------------------------------------
    var visionOdometry = visionFrameUpdates;   
    for (int i = 0; i < visionOdometry.size(); i++) {
        if(visionOdometry.get(i).getName().equals("limelight-ramen")){
            continue;
        } 
        //pose estimators standard dev are increase x, y, rotatinal radians values to trust vision less   
        xyStdDev = 0;

        //tag count
        if(visionOdometry.get(i).getNumOfTagsVisible() >= 2){

            //vision is trusted more with more tags visible
            xyStdDev = 5; 
        } else {
            xyStdDev = 1000;
        }
        
        if(visionOdometry.get(i).getAvgArea() <= 0.05){

            //Do not trust vision inputs if the tag size is extermely small
            xyStdDev = 1000; 
        } else {
            xyStdDev = 5;
        }

        m_poseEstimator.setVisionMeasurementStdDevs(
            VecBuilder.fill(xyStdDev, xyStdDev,99999999.0)
        ); //gyro can be purely trusted for pose calculations so always trust it more than vision
        
        if(visionOdometry.get(i).hasTarget()) { 
            m_poseEstimator.addVisionMeasurement(
                new Pose2d(visionOdometry.get(i).getPose().getTranslation(), visionOdometry.get(i).getPose().getRotation()), //only use vison for x,y pose, because gyro is already accurate enough
                visionOdometry.get(i).getTimestamp()
            );
        }
    }
  }

  /**************************************************************
   * 
   * Pose Estimation update setters
   * 
   ************************************************************/

  public void addVisionObservation(VisionFromAprilTagObservation observation) {
    hasTarget = observation.hasTarget;
    visionHorizontalDistance = observation.horizontalDistance;
    speakerHorizontalDistance = Double.NaN;
    speakerCameraPose = Optional.empty();
    Optional<Pose2d> visionPose = Optional.empty();

    if (observation.visionPose != null) { //TODO when is this ever null?
        if (observation.primaryTagId == FieldConstants.BLUE_SPEAKER_APRILTAG || observation.primaryTagId == FieldConstants.RED_SPEAKER_APRILTAG) {
            //Update Auto Aim Related Variables
            speakerCameraPose = Optional.of(observation.visionPose);
            speakerHorizontalDistance = observation.horizontalDistance;
        }

        // Get the difference between current and previous odometry pose.
        var previousRobotPose = poseBuffer.getSample(observation.timestamp).orElse(odometryPose);

        var newCameraPose = latencyCompensateVision(
                observation.visionPose, previousRobotPose, odometryPose);

        // Recalculate distance to account for robot movement.
        var poseVisionHorizontalDistance =
                observation.visionPose.getTranslation().getNorm();
        Logger.recordOutput("Vision/HorizontalDistanceFromPose", poseVisionHorizontalDistance);

        var latencyCompensatedVisionHorizontalDistance =
                newCameraPose.getTranslation().getNorm();
        Logger.recordOutput(
                "Vision/LatencyCompensatedHorizontalDistance", latencyCompensatedVisionHorizontalDistance);

        m_poseEstimator.addVisionMeasurement(observation.visionPose, observation.timestamp);
    }
  }
  


  public void addOdometryObservation(OdometryObservation observation) {
    //-------------------------------------------------------------------
    //  Gyro input collection
    //--------------------------------------------------------------------
    if (observation.gyroAngle != null) {
        //run REAL gyro input collection for pose estimation
        lastGyroAngle = observation.gyroAngle;
        lastGyroAngularVelocity = observation.gyroAngularVelocity;
    } else {
        // If gyro is not connected, simulate gyro using wheel position deltas
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions);
        lastGyroAngle = lastGyroAngle.plus(new Rotation2d(twist.dtheta));
        // simulate gyro drift.  +/- 0.25 degree.
        // var drift = Rotation2d.fromDegrees(0.0); // Rotation2d.fromDegrees(-0.25 + (Math.random() * 0.5));
        // lastGyroAngle = lastGyroAngle.plus(drift);
        lastGyroAngularVelocity = twist.dtheta;
    }

    //----------------------------------------------------------------------
    // Update module state information
    //----------------------------------------------------------------------
    lastWheelPositions = observation.wheelPositions;
    // update Class internal module states
    moduleStates = observation.moduleStates;
    var chassisSpeeds = kinematics.toChassisSpeeds(observation.moduleStates);
    robotAccelerations = new Twist2d(
            (chassisSpeeds.vxMetersPerSecond - lastChassisSpeeds.vxMetersPerSecond) / observation.timestamp,
            (chassisSpeeds.vyMetersPerSecond - lastChassisSpeeds.vyMetersPerSecond) / observation.timestamp,
            (chassisSpeeds.omegaRadiansPerSecond - lastChassisSpeeds.omegaRadiansPerSecond)
                    / observation.timestamp);
    lastChassisSpeeds = chassisSpeeds;


    //----------------------------------------------------------------------
    // Update Swerve Drive Odometry
    //----------------------------------------------------------------------
    odometryPose = odometry.update(lastGyroAngle, observation.wheelPositions);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp, odometryPose);


    //----------------------------------------------------------------------
    // Update Pose Estimation
    //----------------------------------------------------------------------
    m_poseEstimator.updateWithTime(observation.timestamp, lastGyroAngle, observation.wheelPositions);

    lastAccelerationXFromNavX = observation.accelerationX;
    lastAccelerationYFromNavX = observation.accelerationY;
  }


  /**
   * Applies latency compensation to a vision observation.
   *
   * @param cameraPose          The camera pose.
   * @param previousRobotPose   The previous robot pose.
   * @param currentRobotPose    The current robot pose.
   * @return The latency compensated vision pose.
   */
  protected Pose2d latencyCompensateVision(
          Pose2d cameraPose,
          Pose2d previousRobotPose,
          Pose2d currentRobotPose) {
      // Get the relative movement of the robot since the camera observation.
      var robotPoseChange = currentRobotPose.minus(previousRobotPose);

      // Apply the changes to the camera pose to get the current camera pose.
      return new Pose2d(
              cameraPose.getTranslation().plus(robotPoseChange.getTranslation()),
              cameraPose.getRotation());
  }



  public void resetPoseEstimator(Rotation2d resetRotation, SwerveModulePosition[] modulePositions, Pose2d pose) {
    m_poseEstimator.resetPosition(resetRotation, modulePositions, pose);
  }


  @AutoLogOutput
  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }


  /**************************************************************
   * 
   * Odometry and Vision Record types
   * 
   *******************************************************************/

  public record OdometryObservation(
        double timestamp,
        SwerveDriveWheelPositions wheelPositions,
        Rotation2d gyroAngle,
        SwerveModuleState[] moduleStates,
        double gyroAngularVelocity,
        double accelerationX,
        double accelerationY) {}

  public static record VisionObservation(        
        Pose2d visionPose,
        double timestamp,
        int numOfTagsVisible,
        double avgArea,
        String name,
        boolean hasTarget) {}

  /**
   * Represents an observation of where the camera is on the field as determined by an april tag.
   */
  public record VisionFromAprilTagObservation(
          double timestamp, 
          Pose2d visionPose, 
          double primaryTagId, 
          boolean hasTarget, 
          double horizontalDistance,
          double avgArea,
          String name) {}
}
