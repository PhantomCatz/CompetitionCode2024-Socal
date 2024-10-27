package frc.robot;

import static edu.wpi.first.apriltag.AprilTagFields.k2024Crescendo;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
   * Contains various field dimensions and useful reference points. Dimensions are
   * in meters, and sets
   * of corners start in the lower left moving clockwise. <b>All units in
   * Meters</b> <br>
   * <br>
   *
   * <p>
   * All translations and poses are stored with the origin at the rightmost point
   * on the BLUE
   * ALLIANCE wall.<br>
   * <br>
   * Length refers to the <i>x</i> direction (as described by wpilib) <br>
   * Width refers to the <i>y</i> direction (as described by wpilib)
   */
  public class FieldConstants {

    public static final double SPEAKER_COORD_MTRS_Y = Units.inchesToMeters(219.277);
    public static final double HOARD_LOCATION_Y = Units.inchesToMeters(219.277) + 2.0;
    public static final double FIELD_LENGTH_MTRS = Units.inchesToMeters(651.223);
    public static final double fieldWidth = Units.inchesToMeters(323.277);
    public static final double wingX = Units.inchesToMeters(229.201);
    public static final double podiumX = Units.inchesToMeters(126.75);
    public static final double startingLineX = Units.inchesToMeters(74.111);

    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG  = 4;

    public static Translation2d ampCenter = new Translation2d(Units.inchesToMeters(72.455),
        Units.inchesToMeters(322.996));

    /** Staging locations for each note */
    public static final class StagingLocations {
      public static double centerlineX = FIELD_LENGTH_MTRS / 2.0;

      // need to update
      public static double centerlineFirstY = Units.inchesToMeters(29.638);
      public static double centerlineSeparationY = Units.inchesToMeters(66);
      public static double spikeX = Units.inchesToMeters(114);
      // need
      public static double spikeFirstY = Units.inchesToMeters(161.638);
      public static double spikeSeparationY = Units.inchesToMeters(57);

      public static Translation2d[] centerlineTranslations = new Translation2d[5];
      public static Translation2d[] spikeTranslations = new Translation2d[3];

      static {
        for (int i = 0; i < centerlineTranslations.length; i++) {
          centerlineTranslations[i] = new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
        }
      }

      static {
        for (int i = 0; i < spikeTranslations.length; i++) {
          spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
        }
      }
    }

    /** Each corner of the speaker * */
    public static final class Speaker {

      // corners (blue alliance origin)
      public static Translation3d topRightSpeaker = new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(238.815),
          Units.inchesToMeters(13.091));

      public static Translation3d topLeftSpeaker = new Translation3d(
          Units.inchesToMeters(18.055),
          Units.inchesToMeters(197.765),
          Units.inchesToMeters(83.091));

      public static Translation3d bottomRightSpeaker = new Translation3d(0.0, Units.inchesToMeters(238.815),
          Units.inchesToMeters(78.324));
      public static Translation3d bottomLeftSpeaker = new Translation3d(0.0, Units.inchesToMeters(197.765),
          Units.inchesToMeters(78.324));

      /** Center of the speaker opening (blue alliance) */
      public static Translation3d centerSpeakerOpening = new Translation3d(
          topLeftSpeaker.getX() / 2.0,
          fieldWidth - Units.inchesToMeters(104.0),
          (bottomLeftSpeaker.getZ() + bottomRightSpeaker.getZ()) / 2.0);
    }

    public static final class Stage {
      public static final Pose2d podiumLeg =
          new Pose2d(Units.inchesToMeters(126.75), Units.inchesToMeters(161.638), new Rotation2d());
      public static final Pose2d ampLeg =
          new Pose2d(
              Units.inchesToMeters(220.873),
              Units.inchesToMeters(212.425),
              Rotation2d.fromDegrees(-30));
      public static final Pose2d sourceLeg =
          new Pose2d(
              Units.inchesToMeters(220.873),
              Units.inchesToMeters(110.837),
              Rotation2d.fromDegrees(30));

      public static final Pose2d centerPodiumAmpChain =
          new Pose2d(
              podiumLeg.getTranslation().interpolate(ampLeg.getTranslation(), 0.5),
              Rotation2d.fromDegrees(120.0));
      public static final Pose2d centerAmpSourceChain =
          new Pose2d(
              ampLeg.getTranslation().interpolate(sourceLeg.getTranslation(), 0.5), new Rotation2d());
      public static final Pose2d centerSourcePodiumChain =
          new Pose2d(
              sourceLeg.getTranslation().interpolate(podiumLeg.getTranslation(), 0.5),
              Rotation2d.fromDegrees(240.0));
      public static final Pose2d center =
          new Pose2d(Units.inchesToMeters(192.55), Units.inchesToMeters(161.638), new Rotation2d());
      public static final double centerToChainDistance =
          center.getTranslation().getDistance(centerPodiumAmpChain.getTranslation());
    }

    public static final class Amp {
      public static final Translation2d ampTapeTopCorner =
          new Translation2d(Units.inchesToMeters(130.0), Units.inchesToMeters(305.256));
      public static final double ampBottomY = fieldWidth - Units.inchesToMeters(17.75);
    }

    public static double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final AprilTagFieldLayout aprilTags;

    static {
      try {
        aprilTags = AprilTagFieldLayout.loadFromResource(k2024Crescendo.m_resourceFile);
        aprilTags.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    }
  }