package com.peninsula.frc2023.auto.drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.util.Flipper;
import edu.wpi.first.math.geometry.Pose2d;

public class FlipAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PathPlannerTrajectory traj1 = PathPlanner.loadPath("TestPath", 2, 2);

    Pose2d inital = traj1.getInitialHolonomicPose();

    if (Robot.blue) {
      inital = Flipper.flip(inital);
    }

    var setInitialOdometry =
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees());

    var t1 = new DrivePathRoutine(traj1);

    return new SequentialRoutine(setInitialOdometry, t1);
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
