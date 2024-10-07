package com.peninsula.frc2023.robot;

import static com.peninsula.frc2023.config.VisionConstants.*;

import com.peninsula.frc2023.config.ArmConstants;
import com.peninsula.frc2023.config.SwerveConstants;
import com.peninsula.frc2023.config.VisionConstants;
import com.peninsula.frc2023.subsystems.Swerve;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A class that does the necessary calculations for the Robot State(RobotState.java) after the
 * hardware reads performed by the HardwareReader.java class.
 */
public class RobotStateEstimator {
  private static final RobotStateEstimator sInstance = new RobotStateEstimator();

  private RobotStateEstimator() {}

  public static RobotStateEstimator getInstance() {
    return sInstance;
  }

  public void updateSwervePoseEstimator(RobotState state) {

    // +x is shooter side, +y is side with PDP
    // TODO: Update the +x and +y side descriptions to match 2023 Robot
    state.chassisRelativeSpeeds =
        SwerveConstants.kKinematics.toChassisSpeeds(state.realModuleStates);

    // +x is towards alliance station, +y is right of driver station
    state.fieldRelativeSpeeds =
        new ChassisSpeeds(
            state.gyroHeading.getCos() * state.chassisRelativeSpeeds.vxMetersPerSecond
                - state.gyroHeading.getSin() * state.chassisRelativeSpeeds.vyMetersPerSecond,
            state.gyroHeading.getSin() * state.chassisRelativeSpeeds.vxMetersPerSecond
                + state.gyroHeading.getCos() * state.chassisRelativeSpeeds.vyMetersPerSecond,
            state.chassisRelativeSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber("Field Y", state.fieldRelativeSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Field X", state.fieldRelativeSpeeds.vxMetersPerSecond);

    SmartDashboard.putNumber("Chassis Y", state.chassisRelativeSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis X", state.chassisRelativeSpeeds.vxMetersPerSecond);

    state.gyroHeading = state.gyroHeading.plus(state.initPose.getRotation());

    if (!Robot.isRobotReal()) {
      state.poseEst.update(
          Swerve.getInstance().getOutputs().getRotation2d(), state.realModulePositions);
    } else state.poseEst.update(state.gyroHeading, state.realModulePositions); // On ground

    state.odometryDeadReckonUpdateTime = state.gameTimeS;

    state.pastPoses.addSample(
        state.odometryDeadReckonUpdateTime, state.poseEst.getEstimatedPosition());

    SmartDashboard.putNumber("gyro", state.gyroHeading.getDegrees());

    if (state.currentTrajectory != null) {

      /* Field 2D update */
      state.m_field.getObject("traj").setTrajectory(state.currentTrajectory);
    }

    SmartDashboard.putData(state.m_field);

    state.m_field.setRobotPose(state.poseEst.getEstimatedPosition());

    state
        .m_field3d
        .getObject("cameraRIO")
        .setPose(
            new Pose3d(state.poseEst.getEstimatedPosition())
                .transformBy(VisionConstants.robot_to_camera_RIO));

    state
        .m_field3d
        .getObject("cameraNETWORK")
        .setPose(
            new Pose3d(state.poseEst.getEstimatedPosition())
                .transformBy(VisionConstants.robot_to_camera_NETWORK));

    SmartDashboard.putData(state.m_field3d);

    ct++;
  }

  public static int ct = 0;

  public void updateArmViz(RobotState state) {

    Rotation2d angle1 = state.armState.getAngle1();
    Rotation2d angle2 = state.armState.getAngle2();

    //    state.m_field.setRobotPose(new Pose2d(new Translation2d(5, 5), new Rotation2d()));
    //    SmartDashboard.putData(state.m_field);

    Rotation3d td = new Rotation3d(0, -angle1.getRadians(), 0);
    Rotation3d td2 = new Rotation3d(0, -angle1.rotateBy(angle2).getRadians(), 0);

    SmartDashboard.putNumberArray(
        "armBaseRobotFrame",
        new double[] {
          ArmConstants.chassis_to_base.getY(),
          ArmConstants.chassis_to_base.getX(),
          ArmConstants.chassis_to_base.getZ(),
          td.getQuaternion().getX(),
          td.getQuaternion().getY(),
          td.getQuaternion().getZ(),
          td.getQuaternion().getW()
        });
    SmartDashboard.putNumberArray(
        "armBaseRobotFrame2",
        new double[] {
          ArmConstants.chassis_to_base.getY() + ArmConstants.l1 * angle1.getCos(),
          ArmConstants.chassis_to_base.getX(),
          ArmConstants.chassis_to_base.getZ() + ArmConstants.l1 * angle1.getSin(),
          td2.getQuaternion().getX(),
          td2.getQuaternion().getY(),
          td2.getQuaternion().getZ(),
          td2.getQuaternion().getW()
        });
  }

  public void updateVisionPoseEstimator(RobotState state) {
    var latestResultsNETWORK = state.photonPipelineResultNETWORK;
    var latestResultsRIO = state.photonPipelineResultRIO;

    if (state.visionOverrideOff) return;

    // RIO
    if (latestResultsNETWORK.hasTargets()) {
      if (VisionConstants.useMultiTag && latestResultsNETWORK.getTargets().size() > 1) {
        SmartDashboard.putBoolean("network multi", true);
        Optional<EstimatedRobotPose> estimatedRobotPose =
            state.photonPoseEstimatorNETWORK.update(state.photonPipelineResultNETWORK);
        estimatedRobotPose.ifPresent(
            robotPose ->
                state.poseEst.addVisionMeasurement(
                    robotPose.estimatedPose.toPose2d(), state.timestampLastResultNETWORK));
      } else {
        SmartDashboard.putBoolean("network multi", false);
        List<PhotonTrackedTarget> trackedTargets = latestResultsNETWORK.getTargets();
        updatePoseEstSingleTargets(
            trackedTargets,
            state,
            latestResultsNETWORK.getLatencyMillis(),
            VisionConstants.robot_to_camera_NETWORK,
            "NETWORK");
      }
    }

    // PDH
    if (latestResultsRIO.hasTargets()) {
      if (VisionConstants.useMultiTag && latestResultsRIO.getTargets().size() > 1) {
        SmartDashboard.putBoolean("rio multi", true);
        Optional<EstimatedRobotPose> estimatedRobotPose =
            state.photonPoseEstimatorRIO.update(state.photonPipelineResultRIO);
        estimatedRobotPose.ifPresent(
            robotPose ->
                state.poseEst.addVisionMeasurement(
                    robotPose.estimatedPose.toPose2d(), state.timestampLastResultRIO));
      } else {
        SmartDashboard.putBoolean("rio multi", false);
        List<PhotonTrackedTarget> trackedTargets = latestResultsRIO.getTargets();
        updatePoseEstSingleTargets(
            trackedTargets,
            state,
            latestResultsRIO.getLatencyMillis(),
            VisionConstants.robot_to_camera_RIO,
            "RIO");
      }
    }

    SmartDashboard.putData(state.m_field3d);
  }

  public void updatePoseEstSingleTargets(
      List<PhotonTrackedTarget> trackedTargets,
      RobotState state,
      double latency,
      Transform3d robot_to_camera,
      String name) {

    List<Pose2d> robotEstimates = new ArrayList<>();

    List<Pose3d> camEst = new ArrayList<>();
    List<Pose3d> robEst = new ArrayList<>();
    List<Pose3d> tags = new ArrayList<>();

    for (PhotonTrackedTarget trackedTarget : trackedTargets) {
      Transform3d camera_to_target = trackedTarget.getBestCameraToTarget();

      int fiducialId = trackedTarget.getFiducialId();

      if (VisionConstants.aprilTagFieldLayout.getTagPose(fiducialId).isEmpty()) continue;
      if (trackedTarget.getPoseAmbiguity() > VisionConstants.kMaxPoseAmbiguity) continue;
      if (hypot3d(camera_to_target.getX(), camera_to_target.getY(), camera_to_target.getZ())
          > VisionConstants.maxDistanceMeters) continue;

      Pose3d fiducial_pose_3d = VisionConstants.aprilTagFieldLayout.getTagPose(fiducialId).get();

      tags.add(fiducial_pose_3d);

      Pose3d camera_pose_3d_estimate = fiducial_pose_3d.transformBy(camera_to_target.inverse());

      camEst.add(camera_pose_3d_estimate);

      Pose3d robot_pose_3d = camera_pose_3d_estimate.transformBy(robot_to_camera.inverse());

      robEst.add(robot_pose_3d);

      robotEstimates.add(robot_pose_3d.toPose2d());
    }

    state.m_field3d.getObject("camera " + name).setPoses(camEst);
    state.m_field3d.getObject("robos " + name).setPoses(robEst);
    state.m_field3d.getObject("tags " + name).setPoses(tags);

    double latencySeconds = latency / 1000 + 0.011;

    for (Pose2d estimated_robot_pose_2d : robotEstimates) {
      state.poseEst.addVisionMeasurement(estimated_robot_pose_2d, state.gameTimeS - latencySeconds);
    }
  }

  public double hypot3d(double x, double y, double z) {
    return Math.sqrt(x * x + y * y + z * z);
  }
}
