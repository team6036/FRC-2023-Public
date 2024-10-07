package com.peninsula.frc2023.auto.drive.withoutClimb;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.auto.MoveToCubeIntake;
import com.peninsula.frc2023.behavior.ParallelRaceRoutine;
import com.peninsula.frc2023.behavior.ParallelRoutine;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.GripperSet;
import com.peninsula.frc2023.behavior.routines.IntakeRetract;
import com.peninsula.frc2023.behavior.routines.TimedRoutine;
import com.peninsula.frc2023.behavior.routines.VisionSetOverride;
import com.peninsula.frc2023.behavior.routines.arm.RunArmTrajectoryRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.subsystems.Arm;
import com.peninsula.frc2023.subsystems.Gripper;
import com.peninsula.frc2023.util.Flipper;
import edu.wpi.first.math.geometry.Pose2d;

public class ThreePieceFlatAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("TestPathCone", 4, 3.5);
    PathPlannerTrajectory traj2 = PathPlanner.loadPath("TestPathBack", 4, 3.5);
    PathPlannerTrajectory traj3 = PathPlanner.loadPath("TestPathBackOffWithGrab", 4, 3.5);
    PathPlannerTrajectory traj4 = PathPlanner.loadPath("TestPathBackOffWithCube", 4, 3.5);
    PathPlannerTrajectory traj5 = PathPlanner.loadPath("MoveOutEndCopy", 4, 3.5);

    Pose2d inital = traj1.getInitialHolonomicPose();

    if (Robot.blue) {
      inital = Flipper.flip(inital);
    }

    var setInitialOdometry =
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees());

    var t1 = new DrivePathRoutine(traj1, 1.05);

    var movePickAndTraj =
        new ParallelRoutine(
            t1,
            new SequentialRoutine(new TimedRoutine(0.5), new VisionSetOverride(0.1, true)),
            new SequentialRoutine(
                new TimedRoutine(0.4),
                new GripperSet(Gripper.State.ON, 0.1),
                new MoveToCubeIntake().getRoutine()));

    var placeHigh =
        new SequentialRoutine(
            new ParallelRaceRoutine(
                new RunArmTrajectoryRoutine(Arm.Positions.PLACE_CONE_H, 1.0),
                new GripperSet(Gripper.State.ON, 2.0)),
            new ParallelRoutine(
                new SequentialRoutine(
                    new TimedRoutine(0.2), new GripperSet(Gripper.State.SPIT, 0.2)),
                new SequentialRoutine(new TimedRoutine(0.6), movePickAndTraj),
                new RunArmTrajectoryRoutine(Arm.Positions.STOWED, 0.6)));

    var t2 = new DrivePathRoutine(traj2, 1.05);

    var moveAndTraj =
        new ParallelRoutine(
            new VisionSetOverride(0.1, false),
            new ParallelRaceRoutine(
                new SequentialRoutine(new TimedRoutine(1.2), new IntakeRetract(0.2)),
                new SequentialRoutine(
                    new TimedRoutine(0.8),
                    new RunArmTrajectoryRoutine(Arm.Positions.PLACE_CUBE_H, 1.5)),
                new GripperSet(Gripper.State.ON, 1.5)),
            new SequentialRoutine(t2, new TimedRoutine(0.2)));

    var t3 = new DrivePathRoutine(traj3, 1.05);

    var t4 = new DrivePathRoutine(traj4, 1.05);

    var t5 = new DrivePathRoutine(traj5);

    var moveAndTraj2 =
        new ParallelRoutine(
            new VisionSetOverride(0.1, false),
            new ParallelRaceRoutine(
                new SequentialRoutine(new TimedRoutine(0.5), new IntakeRetract(0.2)),
                new RunArmTrajectoryRoutine(Arm.Positions.PLACE_CUBE_M, 1.5),
                new GripperSet(Gripper.State.ON, 1.5)),
            t4);

    return new SequentialRoutine(
        setInitialOdometry,
        placeHigh,
        moveAndTraj,
        new GripperSet(Gripper.State.SPIT, 0.1),
        new ParallelRoutine(
            t3,
            new SequentialRoutine(new TimedRoutine(0.5), new VisionSetOverride(0.1, true)),
            new SequentialRoutine(
                new TimedRoutine(0.2),
                new GripperSet(Gripper.State.ON, 0.1),
                new ParallelRoutine(
                    new VisionSetOverride(0.1, false), new MoveToCubeIntake().getRoutine()))),
        moveAndTraj2,
        new GripperSet(Gripper.State.SPIT, 0.1),
        new ParallelRoutine(t5, new RunArmTrajectoryRoutine(Arm.Positions.STOWED, 1.0)));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
