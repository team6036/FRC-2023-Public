package com.peninsula.frc2023.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.subsystems.*;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectory;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectoryProfiledAngles;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/** Commands represent what we want the robot to be doing. */
@SuppressWarnings("java:S1104")
public class Commands {

  /* Routines */
  public List<RoutineBase> routinesWanted = new ArrayList<>();
  public boolean shouldClearCurrentRoutines;
  /* Swerve */
  public Swerve.State swerveWanted;
  public boolean boostWanted;
  public boolean robotCentricWanted;
  public double angleWanted;

  /* Vision */
  public Vision.State visionWanted;

  /* Intake */
  public Intake.State intakeWanted = Intake.State.OFF;

  /* Auto */
  public PathPlannerTrajectory wantedPathPlannerTrajectory;
  public Pose2d driveWantedOdometryPose = new Pose2d(0, 0, new Rotation2d(0));
  public Rotation2d driveWantedOdometryPoseRotation = new Rotation2d(0);

  public void addWantedRoutines(RoutineBase... wantedRoutines) {
    for (RoutineBase wantedRoutine : wantedRoutines) {
      addWantedRoutine(wantedRoutine);
    }
  }

  /* Gripper */
  public Gripper.State gripperWanted = Gripper.State.ON;

  public void addWantedRoutine(RoutineBase wantedRoutine) {
    routinesWanted.add(wantedRoutine);
  }

  /* Drive */
  public Pose2d wantedFSDPose = new Pose2d(new Translation2d(), new Rotation2d());

  @Override
  public String toString() {
    var log = new StringBuilder();
    log.append("Wanted routines: ");
    for (RoutineBase routine : routinesWanted) {
      log.append(routine).append(" ");
    }
    return log.append("\n").toString();
  }

  public PathPlannerTrajectory getDriveWantedPathPlannerTrajectory() {
    return wantedPathPlannerTrajectory;
  }

  public void setDriveFollowPath(PathPlannerTrajectory pathPlannerTrajectory) {
    wantedPathPlannerTrajectory = pathPlannerTrajectory;
    swerveWanted = Swerve.State.AUTO;
  }

  public void setDriveIdle() {
    swerveWanted = Swerve.State.NEUTRAL;
  }

  public void setClimberPosition(double per) {}

  /* Arm */
  public Arm.State wantedArmState = Arm.State.SET_ANGLES;

  public ArmTrajectoryProfiledAngles wantedArmTrajectoryProfiledAngles = null;
  public TwoDOFKinematics.ArmConfiguration wantedSetAngles =
      new TwoDOFKinematics.ArmConfiguration(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));
  public Pose2d wantedArmPosition = new Pose2d(new Translation2d(-0.9, 0.3), new Rotation2d());

  public ArmTrajectory wantedArmTrajectory = null;

  /* Lighting */
  public Lighting.State wantedLighting = Lighting.State.IDLE;

  /* FSD */
  public int wantedColumn = 0;
  public int wantedRow = 0;
}
