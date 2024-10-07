package com.peninsula.frc2023.behavior.routines.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.SubsystemBase;
import java.util.Set;

/** Follows a pathplanner trajectory */
public class DrivePathRoutine extends TimeoutRoutineBase {

  private PathPlannerTrajectory mPathPlannerTrajectory;
  private boolean start = false;

  public DrivePathRoutine(PathPlannerTrajectory trajectory) {
    this(trajectory, 1.15);
  }

  public DrivePathRoutine(PathPlannerTrajectory trajectory, double mult_constant) {
    mPathPlannerTrajectory = trajectory;
    mTimeout = mPathPlannerTrajectory.getTotalTimeSeconds() * mult_constant;
  }

  @Override
  public void start(Commands commands, @ReadOnly RobotState state) {
    // Required to start the timeout timer
    super.start(commands, state);
  }

  @Override
  public Set<SubsystemBase> getRequiredSubsystems() {
    return Set.of(mDrive);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    if (!start) {
      commands.setDriveFollowPath(mPathPlannerTrajectory);
      state.currentTrajectory = mPathPlannerTrajectory;
      start = true;
    }
  }

  @Override
  public boolean checkIfFinishedEarly(@ReadOnly RobotState state) {
    // TODO: possibly implement to see if we are within a tolerance of the end early
    return false;
  }
}
