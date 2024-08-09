// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveAndRobotOrientation;

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
import frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain.CatzSwerveModule;
import frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.Subsystems.DriveAndRobotOrientation.vision.CatzVision;
import frc.robot.Utilities.CatzMathUtils;
import frc.robot.Utilities.FieldRelativeAccel;
import frc.robot.Utilities.FieldRelativeSpeed;
import lombok.Getter;
import lombok.Setter;

public class CatzRobotTracker {

  private static CatzRobotTracker instance;

  private static final double POSE_BUFFER_SIZE_SECONDS = 2.0;


  // Pose Estimation Members
  private Pose2d odometryPose = new Pose2d();

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
          TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SECONDS);
  // Swerve drive pose estimator for tracking robot pose
  private static SwerveDrivePoseEstimator m_poseEstimator;

  private double m_currentGyroAngularVelocity = 0.0;
  private double lastAccelerationXFromNavX = 0.0;
  private double lastAccelerationYFromNavX = 0.0;

  private Optional<Pose2d> speakerCameraPose = Optional.empty();
  private double speakerHorizontalDistance = Double.NaN;
  private Optional<Pose2d> robotPoseFromCameraPose = Optional.empty();
  private double visionHorizontalDistance = 0.0;

  private Twist2d robotVelocity;
  private Twist2d trajectoryVelocity;

  private boolean hasTarget;

  @AutoLogOutput @Getter @Setter private boolean flywheelAccelerating = false;

  protected Twist2d robotAccelerations = new Twist2d();
  protected SwerveDriveOdometry odometry;

  private SwerveDriveWheelPositions m_currentWheelPositions = new SwerveDriveWheelPositions(new SwerveModulePosition[] {
      new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
  });

  private SwerveModuleState[] m_currentModuleStates = new SwerveModuleState[] {
      new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
  };

  private Rotation2d m_currentGyroAngle = new Rotation2d();
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();
  private final SwerveDriveKinematics kinematics;


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
    if(instance == null) {
      instance = new CatzRobotTracker();
    }
    return instance;
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
        m_currentGyroAngle = observation.gyroAngle;
        m_currentGyroAngularVelocity = observation.gyroAngularVelocity;
    } else {
        // If gyro is not connected, simulate gyro using wheel position deltas
        Twist2d twist = kinematics.toTwist2d(m_currentWheelPositions, observation.wheelPositions);
        m_currentGyroAngle = m_currentGyroAngle.plus(new Rotation2d(twist.dtheta));
        // simulate gyro drift.  +/- 0.25 degree.
        // var drift = Rotation2d.fromDegrees(0.0); // Rotation2d.fromDegrees(-0.25 + (Math.random() * 0.5));
        // lastGyroAngle = lastGyroAngle.plus(drift);
        m_currentGyroAngularVelocity = twist.dtheta;
    }

    //----------------------------------------------------------------------
    // Update module state information
    //----------------------------------------------------------------------
    m_currentWheelPositions = observation.wheelPositions;
    // update Class internal module states
    m_currentModuleStates = observation.moduleStates;
    var chassisSpeeds = kinematics.toChassisSpeeds(observation.moduleStates);
    robotAccelerations = new Twist2d(
            (chassisSpeeds.vxMetersPerSecond - m_lastChassisSpeeds.vxMetersPerSecond) / observation.timestamp,
            (chassisSpeeds.vyMetersPerSecond - m_lastChassisSpeeds.vyMetersPerSecond) / observation.timestamp,
            (chassisSpeeds.omegaRadiansPerSecond - m_lastChassisSpeeds.omegaRadiansPerSecond)
                    / observation.timestamp);
    m_lastChassisSpeeds = chassisSpeeds;


    //----------------------------------------------------------------------
    // Update Swerve Drive Odometry
    //----------------------------------------------------------------------
    odometryPose = odometry.update(m_currentGyroAngle, observation.wheelPositions);
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp, odometryPose);


    //----------------------------------------------------------------------
    // Update Pose Estimation
    //----------------------------------------------------------------------
    m_poseEstimator.updateWithTime(observation.timestamp, m_currentGyroAngle, observation.wheelPositions);

    lastAccelerationXFromNavX = observation.accelerationX;
    lastAccelerationYFromNavX = observation.accelerationY;
  }

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

  public void addTrajectoryVelocityData(Twist2d robotVelocity) {
    this.trajectoryVelocity = robotVelocity;
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

  /**
   * Resets the robot pose estimator and odometry
   *
   * @param pose      The starting field-relative pose measurement.
   */
  public void resetPosition(Pose2d pose) {
    odometry.resetPosition(m_currentGyroAngle, m_currentWheelPositions, pose);
    odometryPose = odometry.getPoseMeters();
    m_poseEstimator.resetPosition(m_currentGyroAngle, m_currentWheelPositions, pose);
  }

  /**
   * Predicts what our pose will be in the future. Allows separate translation and rotation
   * lookaheads to account for varying latencies in the different measurements.
   *
   * @param translationLookaheadS The lookahead time for the translation of the robot
   * @param rotationLookaheadS The lookahead time for the rotation of the robot
   * @return The predicted pose.
   */
  public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
    Twist2d velocity = DriverStation.isAutonomousEnabled() ? trajectoryVelocity : robotVelocity;
    return getEstimatedPose()
        .transformBy(
            new Transform2d(
                velocity.dx * translationLookaheadS,
                velocity.dy * translationLookaheadS,
                Rotation2d.fromRadians(velocity.dtheta * rotationLookaheadS)));
  }


  @AutoLogOutput
  public Pose2d getEstimatedPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRobotRotation() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  public SwerveModuleState[] getRobotSwerveModuleStates() {
    return m_currentModuleStates;
  }

  public ChassisSpeeds getRobotChassisSpeeds() {
    return m_lastChassisSpeeds;
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
