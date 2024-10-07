package com.peninsula.frc2023.behavior.routines.drive;

import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Set;

/** Resets the drive pose estimate to starting position */
public class DriveSetOdometryRoutine extends TimeoutRoutineBase {

  public static final double kTimeout = 0.1;
  private Pose2d mTargetPose;
  private Rotation2d mTargetPoseRotation;

  public DriveSetOdometryRoutine() {
    this(0.0, 0.0, 0.0);
  }

  public DriveSetOdometryRoutine(double xMeters, double yMeters, double yawDegrees) {
    super(kTimeout);
    mTargetPose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(yawDegrees));
    mTargetPoseRotation = Rotation2d.fromDegrees(yawDegrees);
  }

  public Pose2d getTargetPose() {
    return mTargetPose;
  }

  public Rotation2d getTargetPoseRotation() {
    return mTargetPoseRotation;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return state.poseEst.getEstimatedPosition().equals(mTargetPose);
  }

  @Override
  public void start(Commands commands, @ReadOnly RobotState state) {
    super.start(commands, state);
    state.initPose =
        new Pose2d(
            mTargetPose.getX(),
            mTargetPose.getY(),
            new Rotation2d(mTargetPoseRotation.getRadians()));
    state.lastVisionEstimatedPose =
        new Pose2d(
            mTargetPose.getX(),
            mTargetPose.getY(),
            new Rotation2d(mTargetPoseRotation.getRadians()));
    commands.driveWantedOdometryPose = mTargetPose;
    commands.driveWantedOdometryPoseRotation = mTargetPoseRotation;
    // state real pos is set in Robot.resetOdometryIfWanted()
  }

  @Override
  public Set<SubsystemBase> getRequiredSubsystems() {
    return Set.of(mDrive);
  }
}
