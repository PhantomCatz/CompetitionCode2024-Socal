// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;

public class EventMarkerHelperCmd extends Command {
  private List<EventMarker> eventMarkers;
  private List<PathPoint> pathPoints;
  private ArrayList<Boolean> alreadyInitializedList = new ArrayList<>();

  private boolean isCommandSkipped = false;

  /** Creates a new EventMarkerCmd. */
  public EventMarkerHelperCmd(List<EventMarker> eventMarkers, List<PathPoint> pathPoints) {
    this.eventMarkers = eventMarkers;
    this.pathPoints = pathPoints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(int i = 0; i < eventMarkers.size(); i++) {
      alreadyInitializedList.add(i, false);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for(int i = 0; i < eventMarkers.size(); i++) {
      boolean hasEventTriggered = hasEventTriggered(i);
      if(hasEventTriggered) {
        try {
          if(alreadyInitializedList.get(i) == false) {
            // forces all logic of the command to be located within this class
            // scheduling the command to run outside of this class will cause commands to finish too early
            eventMarkers.get(i).getCommand().initialize();
            alreadyInitializedList.set(i, true);
          } 

          if(alreadyInitializedList.get(i) == true) {
            eventMarkers.get(i).getCommand().execute();
          }

        } catch (Exception e) {
            System.out.println("Event Command Skipped due to uninitialization");
            isCommandSkipped = true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isCommandSkipped || eventMarkers.size() == 0) || eventMarkers.get(eventMarkers.size()-1).getCommand().isFinished(); // Stop when last marker command finishes
  }

  private boolean hasEventTriggered(int i) {
    boolean triggered = eventMarkers.get(i).getWaypointRelativePos()/pathPoints.size() >= CatzRobotTracker.getInstance().getTrajectoryAmtCompleted();
    System.out.println(eventMarkers.get(i).getWaypointRelativePos()/pathPoints.size());
    System.out.println(CatzRobotTracker.getInstance().getTrajectoryAmtCompleted());
    return triggered;
  }
}
