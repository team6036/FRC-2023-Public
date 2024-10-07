package com.peninsula.frc2023.subsystems.controllers.drive;

import com.peninsula.frc2023.config.SwerveConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Swerve;
import com.peninsula.frc2023.util.control.SwerveOutputs;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSDController extends Swerve.SwerveController {
  private static HolonomicDriveController mTrajectoryController;
  public static boolean resetThetaController =
      false; // Used to reset ProfiledPIDController when driver aligns.

  public FSDController(SwerveOutputs outputs) {
    super(outputs);

    mTrajectoryController =
        new HolonomicDriveController(
            new PIDController(SwerveConstants.p_xy, SwerveConstants.i_xy, SwerveConstants.d_xy),
            new PIDController(SwerveConstants.p_xy, SwerveConstants.i_xy, SwerveConstants.d_xy),
            new ProfiledPIDController(
                SwerveConstants.p_t,
                SwerveConstants.i_t,
                SwerveConstants.d_t,
                new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI * 3)));

    mTrajectoryController.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
    mTrajectoryController.getXController().setTolerance(0.01, 0.02);
    mTrajectoryController.getYController().setTolerance(0.01, 0.02);
    mTrajectoryController.getThetaController().setTolerance(0.02, 0.05);
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {
    double x =
        mTrajectoryController
            .getXController()
            .calculate(state.poseEst.getEstimatedPosition().getX(), commands.wantedFSDPose.getX());

    double y =
        mTrajectoryController
            .getYController()
            .calculate(state.poseEst.getEstimatedPosition().getY(), commands.wantedFSDPose.getY());

    if (resetThetaController) {
      mTrajectoryController
          .getThetaController()
          .reset(state.poseEst.getEstimatedPosition().getRotation().getRadians());
      resetThetaController = false;
    }
    double z =
        mTrajectoryController
            .getThetaController()
            .calculate(
                state.poseEst.getEstimatedPosition().getRotation().getRadians(),
                commands.wantedFSDPose.getRotation().getRadians());

    ChassisSpeeds wantedChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x, y, z, state.poseEst.getEstimatedPosition().getRotation());

    SmartDashboard.putNumber("ThetaFSDSet", commands.wantedFSDPose.getRotation().getRadians());
    SmartDashboard.putNumber(
        "ThetaFSDRead", state.poseEst.getEstimatedPosition().getRotation().getRadians());

    SmartDashboard.putNumber("zOut", z);

    mOutputs =
        TeleopController.outputGenerator.generateSetpoint(
            TeleopController.limits, mOutputs, state, wantedChassisSpeeds, 0.02);
  }

  /**
   * Calculates distance from current position and wanted position
   *
   * @param currentPosition the current position of the robot.
   * @param wantedPosition the wanted position of the robot.
   * @return euclidean distance between current position and wanted position.
   */
  public static double getRemainingDistance(Pose2d currentPosition, Pose2d wantedPosition) {
    return currentPosition.minus(wantedPosition).getTranslation().getNorm();
  }

  /**
   * Returns true if target is reached
   *
   * @return
   */
  public static boolean targetReached() {
    return mTrajectoryController.getXController().atSetpoint()
        && mTrajectoryController.getYController().atSetpoint()
        && mTrajectoryController.getThetaController().atSetpoint();
  }
}
