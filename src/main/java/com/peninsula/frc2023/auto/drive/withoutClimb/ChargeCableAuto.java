package com.peninsula.frc2023.auto.drive.withoutClimb;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.behavior.ParallelRoutine;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.GripperSet;
import com.peninsula.frc2023.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.subsystems.Gripper;
import com.peninsula.frc2023.util.Flipper;
import edu.wpi.first.math.geometry.Pose2d;

public class ChargeCableAuto implements AutoBase {

  @Override
  public RoutineBase getRoutine() {

    PathPlannerTrajectory traj1 = PathPlanner.loadPath("TestPathConeChargeCable", 3.5, 3.5);
    PathPlannerTrajectory traj2 = PathPlanner.loadPath("TestPathBackChargeCable", 3, 2.5);

    Pose2d inital = traj1.getInitialHolonomicPose();

    if (Robot.blue) {
      inital = Flipper.flip(inital);
    }

    var setInitialOdometry =
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees());

    //    var placeHigh =
    //        new SequentialRoutine(
    //            new ParallelRaceRoutine(
    //                new RunArmTrajectoryRoutine(Arm.Positions.PLACE_CUBE_H, 0.6),
    //                new GripperSet(Gripper.State.ON, 3.0)),
    //            new GripperSet(Gripper.State.SPIT, 0.05));

    var t1 = new DrivePathRoutine(traj1);

    var movePickAndTraj =
        new ParallelRoutine(
            // new ParallelRaceRoutine(
            // new RunArmTrajectoryRoutine(Arm.Positions.CUBE_PICK, 1.5),
            // new SequentialRoutine(
            // new TimedRoutine(0.5), new GripperSet(Gripper.State.ON, 3.0))),
            t1);

    var t2 = new DrivePathRoutine(traj2);

    var moveAndTraj =
        new ParallelRoutine(
            // new ParallelRaceRoutine(
            // new RunArmTrajectoryRoutine(Arm.Positions.PLACE_CUBE_H, 1.5),
            // new GripperSet(Gripper.State.ON, 1.5)),
            t2);

    return new SequentialRoutine(
        setInitialOdometry,
        // placeHigh,
        movePickAndTraj,
        moveAndTraj,
        new GripperSet(Gripper.State.SPIT, 0.1));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}