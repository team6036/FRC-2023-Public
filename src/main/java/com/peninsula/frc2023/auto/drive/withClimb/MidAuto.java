package com.peninsula.frc2023.auto.drive.withClimb;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.behavior.ParallelRoutine;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.GripperSet;
import com.peninsula.frc2023.behavior.routines.TimedRoutine;
import com.peninsula.frc2023.behavior.routines.arm.RunArmTrajectoryRoutine;
import com.peninsula.frc2023.behavior.routines.drive.BalanceRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.subsystems.Arm;
import com.peninsula.frc2023.subsystems.Gripper;
import com.peninsula.frc2023.util.Flipper;
import edu.wpi.first.math.geometry.Pose2d;

public class MidAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Mid1", 2, 2);
    PathPlannerTrajectory traj2 = PathPlanner.loadPath("Mid2", 3, 2);

    Pose2d inital = traj1.getInitialHolonomicPose();

    if (Robot.blue) {
      inital = Flipper.flip(inital);
    }

    var setInitialOdometry =
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees());

    var t1 = new DrivePathRoutine(traj1);
    var t2 = new DrivePathRoutine(traj2);

    var moveToPlace =
        new SequentialRoutine(
            new GripperSet(Gripper.State.ON, 0.1),
            new RunArmTrajectoryRoutine(Arm.Positions.PLACE_CUBE_H, 1.0),
            t1);

    return new SequentialRoutine(
        setInitialOdometry,
        moveToPlace,
        new GripperSet(Gripper.State.SPIT, 0.5),
        new GripperSet(Gripper.State.OFF, 0.1),
        // NOTE, if arm disabled, must comment out arm SequentialRoutine in order for balance to
        // work.
        new ParallelRoutine(
            t2,
            new SequentialRoutine(
                new TimedRoutine(1), new RunArmTrajectoryRoutine(Arm.Positions.STOWED, 1.5))),
        new BalanceRoutine(0.1));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
