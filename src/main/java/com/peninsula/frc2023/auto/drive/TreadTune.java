package com.peninsula.frc2023.auto.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.routines.drive.DrivePathRoutine;

public class TreadTune implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Thread Tune", 3.5, 3.5);

    DrivePathRoutine t1 = new DrivePathRoutine(traj1);

    return t1;
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
