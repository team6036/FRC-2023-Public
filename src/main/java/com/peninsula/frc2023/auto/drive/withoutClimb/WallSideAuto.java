package com.peninsula.frc2023.auto.drive.withoutClimb;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.behavior.ParallelRoutine;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.GripperSet;
import com.peninsula.frc2023.behavior.routines.arm.RunArmTrajectoryRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.subsystems.Arm;
import com.peninsula.frc2023.subsystems.Gripper;
import com.peninsula.frc2023.util.Flipper;
import edu.wpi.first.math.geometry.Pose2d;

public class WallSideAuto implements AutoBase {

  @Override
  public RoutineBase getRoutine() {

    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Test1", 2, 2);
    PathPlannerTrajectory traj2 = PathPlanner.loadPath("Test2", 3, 2);

    Pose2d inital = traj1.getInitialHolonomicPose();

    if (Robot.blue) {
      inital = Flipper.flip(inital);
    }

    var setInitialOdometry =
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees());

    var driveT1 = new DrivePathRoutine(traj1);

    var driveToNode =
        new ParallelRoutine(
            new ParallelRoutine(new GripperSet(Gripper.State.ON, 0.1), driveT1),
            new RunArmTrajectoryRoutine(Arm.Positions.PLACE_CUBE_H, 1.0));

    return new SequentialRoutine(
        setInitialOdometry, driveToNode, new GripperSet(Gripper.State.SPIT, 0.5));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
