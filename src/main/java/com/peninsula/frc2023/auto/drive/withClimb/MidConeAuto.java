package com.peninsula.frc2023.auto.drive.withClimb;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.behavior.ParallelRaceRoutine;
import com.peninsula.frc2023.behavior.ParallelRoutine;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.GripperSet;
import com.peninsula.frc2023.behavior.routines.TimedRoutine;
import com.peninsula.frc2023.behavior.routines.VisionSetOverride;
import com.peninsula.frc2023.behavior.routines.arm.RunArmTrajectoryRoutine;
import com.peninsula.frc2023.behavior.routines.drive.BalanceRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2023.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.subsystems.Arm;
import com.peninsula.frc2023.subsystems.Gripper;
import com.peninsula.frc2023.util.Flipper;
import edu.wpi.first.math.geometry.Pose2d;

public class MidConeAuto implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    PathPlannerTrajectory traj1 = PathPlanner.loadPath("Mid2", 3.3, 2.5);

    Pose2d inital = traj1.getInitialHolonomicPose();

    if (Robot.blue) {
      inital = Flipper.flip(inital);
    }

    var setInitialOdometry =
        new DriveSetOdometryRoutine(
            inital.getX(), inital.getY(), inital.getRotation().getDegrees());

    var t1 = new DrivePathRoutine(traj1);

    var movePickAndTraj =
        new ParallelRoutine(
            new SequentialRoutine(new TimedRoutine(0.5), new VisionSetOverride(0.1, true)), t1);

    var placeHigh =
        new SequentialRoutine(
            new ParallelRaceRoutine(
                new RunArmTrajectoryRoutine(Arm.Positions.PLACE_CONE_H, 1.0),
                new GripperSet(Gripper.State.ON, 3.0)),
            new ParallelRoutine(
                new SequentialRoutine(new TimedRoutine(0.2), new GripperSet(Gripper.State.SPIT, 1)),
                new SequentialRoutine(new TimedRoutine(0.6), t1),
                new RunArmTrajectoryRoutine(Arm.Positions.STOWED, 0.6)));
    return new SequentialRoutine(
        setInitialOdometry, placeHigh, movePickAndTraj, new BalanceRoutine(0.1));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
