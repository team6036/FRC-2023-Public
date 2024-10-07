package com.peninsula.frc2023.subsystems.controllers.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.config.SwerveConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Swerve;
import com.peninsula.frc2023.util.Flipper;
import com.peninsula.frc2023.util.control.SwerveOutputs;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDriveController extends Swerve.SwerveController {

  private final Timer mTimer = new Timer();
  private HolonomicDriveController mTrajectoryController =
      new HolonomicDriveController(
          new PIDController(1.04, 0.0, 0.0),
          new PIDController(1.04, 0.0, 0.0),
          new ProfiledPIDController(
              2.0, 0.00, 0.00, new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI * 3)));

  PathPlannerTrajectory mPathPlannerTrajectory;

  public AutoDriveController(SwerveOutputs outputs) {
    super(outputs);
    mTimer.start();
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {

    PathPlannerTrajectory mPathPlannerTrajectoryWanted =
        commands.getDriveWantedPathPlannerTrajectory();

    if (mPathPlannerTrajectory != mPathPlannerTrajectoryWanted) {
      mPathPlannerTrajectory = mPathPlannerTrajectoryWanted;

      ProfiledPIDController thetaController =
          new ProfiledPIDController(
              SwerveConstants.Constants.AutoConstants.kPThetaController,
              0.00,
              0.00,
              SwerveConstants.Constants.AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      mTrajectoryController =
          new HolonomicDriveController(
              new PIDController(SwerveConstants.Constants.AutoConstants.kPXController, 0.0, 0.1),
              new PIDController(SwerveConstants.Constants.AutoConstants.kPYController, 0.0, 0.1),
              thetaController);
      mTimer.reset();
    }

    PathPlannerTrajectory.PathPlannerState targetPose =
        (PathPlannerTrajectory.PathPlannerState) mPathPlannerTrajectory.sample(mTimer.get());

    if (Robot.blue) {
      // On blue, create a copy and flip
      PathPlannerTrajectory.PathPlannerState targetPoseC =
          new PathPlannerTrajectory.PathPlannerState();
      targetPoseC.poseMeters = Flipper.flip(targetPose.poseMeters);
      targetPoseC.holonomicRotation = Flipper.flipRot(targetPose.holonomicRotation);
      targetPoseC.velocityMetersPerSecond = targetPose.velocityMetersPerSecond;

      Pose2d ghost =
          new Pose2d(targetPoseC.poseMeters.getTranslation(), targetPoseC.holonomicRotation);

      state.m_field.getObject("ghostPose").setPose(ghost);

      ChassisSpeeds wantedChassisSpeeds =
          mTrajectoryController.calculate(
              state.poseEst.getEstimatedPosition(), targetPoseC, targetPoseC.holonomicRotation);

      mOutputs.setOutputs(
          SwerveConstants.kKinematics.toSwerveModuleStates(wantedChassisSpeeds),
          targetPoseC.holonomicRotation);
    } else {
      Pose2d ghost =
          new Pose2d(targetPose.poseMeters.getTranslation(), targetPose.holonomicRotation);

      state.m_field.getObject("ghostPose").setPose(ghost);

      ChassisSpeeds wantedChassisSpeeds =
          mTrajectoryController.calculate(
              state.poseEst.getEstimatedPosition(), targetPose, targetPose.holonomicRotation);

      mOutputs.setOutputs(
          SwerveConstants.kKinematics.toSwerveModuleStates(wantedChassisSpeeds),
          targetPose.holonomicRotation);
    }

    SmartDashboard.putNumber("Target Y", targetPose.poseMeters.getY());
    SmartDashboard.putNumber("Current Y", state.poseEst.getEstimatedPosition().getY());
  }
}
