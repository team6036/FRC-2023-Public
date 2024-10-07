package com.peninsula.frc2023.auto.drive.withClimb;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.drive.BalanceRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.util.Flipper;
import edu.wpi.first.math.geometry.Pose2d;

public class Climb implements AutoBase {

  @Override
  public RoutineBase getRoutine() {

    PathPlannerTrajectory traj1 = PathPlanner.loadPath("RunClimb", 3, 2);

    Pose2d inital = traj1.getInitialHolonomicPose();

    if (Robot.blue) {
      inital = Flipper.flip(inital);
    }

    var setInitialOdometry =
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees());

    var driveT1 = new DrivePathRoutine(traj1);

    return new SequentialRoutine(setInitialOdometry, driveT1, new BalanceRoutine(0.1));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
