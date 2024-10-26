package frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision;
    
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzConstants.RobotHardwareMode;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker.VisionFromAprilTagObservation;

public class CatzVision extends SubsystemBase {

    // Hardware IO declaration
    private final VisionIO[] cameras;
    public final VisionIOInputsAutoLogged[] inputs;

    // MISC variables
    private double targetID;
    private int acceptableTagID;
    private boolean useSingleTag = false;
    public static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);

    public CatzVision(VisionIO[] cameras) {
        this.cameras = cameras;
        inputs = new VisionIOInputsAutoLogged[cameras.length];

        for(int i = 0; i < cameras.length; i++) {
            if(CatzConstants.hardwareMode == RobotHardwareMode.REPLAY ||
               CatzConstants.hardwareMode == RobotHardwareMode.SIM) {
               inputs[i] = new VisionIOInputsAutoLogged() {};
            } else {
                inputs[i] = new VisionIOInputsAutoLogged();
            }
        }
    }

    @Override
    public void periodic() {
        
        // For every limelight camera process vision with according logic
        for (int i = 0; i < inputs.length; i++) {
            // update and process new inputs[cameraNum] for camera
            cameras[i].updateInputs(inputs[i]);
            Logger.processInputs("inputs/Vision/" + cameras[i].getName() + "/Inputs", inputs[i]);
           
            // Check when to process Vision Info
            if(cameras[i].getName().equals("limelight-ramen")) { 
                // Continue
            } else {
                if (inputs[i].hasTarget &&
                    inputs[i].maxDistance < LOWEST_DISTANCE && // Process vision only when in teleop and in range of apriltag
                    !DriverStation.isAutonomous()) {
                    processVision(i);
                }
            }

        }        
        Pose2d sobaPose2d = new Pose2d(inputs[1].x, inputs[1].y, new Rotation2d());
        Logger.recordOutput("Vision/vision poses/soba", sobaPose2d);

        Pose2d udonPose2d = new Pose2d(inputs[0].x, inputs[0].y, new Rotation2d());
        Logger.recordOutput("Vision/vison poses/udon", udonPose2d); 

        //DEBUG

        //Logger.recordOutput("Vision/ResultCount", results.size());
    }

    static int camNum;
    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs[cameraNum
        Pose2d currentPose = new Pose2d(inputs[cameraNum].x, 
                                        inputs[cameraNum].y, 
                                        new Rotation2d(inputs[cameraNum].rotation)
                             );

        // A vision updates to robot tracker
        
        if(RobotContainer.updateLimelight){

            CatzRobotTracker.getInstance()
                                .addVisionObservation(
                                    new VisionFromAprilTagObservation(inputs[cameraNum].timestamp, 
                                                                      currentPose, 
                                                                      inputs[cameraNum].primaryApriltagID, 
                                                                      inputs[cameraNum].hasTarget,
                                                                      0.0, 
                                                                      inputs[cameraNum].ta, 
                                                                      cameras[cameraNum].getName())
                                ); //TODO it's not working. it drives in a spiral of archemedes occasionally
        }

        camNum = cameraNum;
    }


    //------------------------------------------------------------------------
    //
    //      Vision util methods 
    //
    //------------------------------------------------------------------------
    public void setUseSingleTag(boolean useSingleTag, int acceptableTagID) {
        this.useSingleTag = useSingleTag;
        this.acceptableTagID = acceptableTagID;
    }

    
    public double getOffsetX(int cameraNum) {
        return inputs[cameraNum].tx;
    }

    public double getOffsetY(int cameraNum) {
        return inputs[cameraNum].ty;
    }

    public double getAprilTagID(int cameraNum) {
        return inputs[cameraNum].primaryApriltagID;
    }

    public int getCameraNum() {
        return camNum;
    }


}