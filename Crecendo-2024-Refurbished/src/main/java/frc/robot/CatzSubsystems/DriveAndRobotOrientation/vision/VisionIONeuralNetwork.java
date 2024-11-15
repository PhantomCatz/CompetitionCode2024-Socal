package frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision;

import static frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Utilities.LimelightHelpers.RawDetection;

public class VisionIONeuralNetwork implements VisionIO {

    private String name;
    
     /**
     * Implements Limelight camera
     *
     * @param name Name of the limelight used, and should be configured in limelight software first
     */
    public VisionIONeuralNetwork(String name, Transform3d transform3d) {
        LimelightHelpers.setPipelineIndex(name, LIMELIGHT_PIPLINE_APRILTAG);

        LimelightHelpers.setCameraPose_RobotSpace(name, transform3d.getX(),
                                                        transform3d.getY(),
                                                        transform3d.getZ(),
                                                        transform3d.getRotation().getX(),
                                                        transform3d.getRotation().getY(),
                                                        transform3d.getRotation().getZ()

        );
        LimelightHelpers.setStreamMode_Standard(name);


        LimelightHelpers.setLEDMode_ForceBlink(name);
        this.name = name;
        System.out.println("Limeilight " + name + " instantiated" + LimelightHelpers.getCurrentPipelineType(name));
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Get raw neural detector results
        RawDetection[] detections = LimelightHelpers.getRawDetections(name);
        for (RawDetection detection : detections) {
            inputs.classID = detection.classId;
            inputs.txnc = detection.txnc;
            inputs.tync = detection.tync;
            inputs.ta = detection.ta;
            // Access corner coordinates if needed
            inputs.corner0X = detection.corner0_X;
            inputs.corner0Y = detection.corner0_Y;
            // ... corners 1-3 available similarly
        }


    }
}
