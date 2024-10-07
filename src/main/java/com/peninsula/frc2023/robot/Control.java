package com.peninsula.frc2023.robot;

import com.peninsula.frc2023.auto.MoveBackFromIntake;
import com.peninsula.frc2023.auto.MoveToCubeIntake;
import com.peninsula.frc2023.behavior.ParallelRoutine;
import com.peninsula.frc2023.behavior.SequentialRoutine;
import com.peninsula.frc2023.behavior.routines.GripperSet;
import com.peninsula.frc2023.behavior.routines.TimedRoutine;
import com.peninsula.frc2023.behavior.routines.arm.RunArmTrajectoryRoutine;
import com.peninsula.frc2023.config.ArmConstants;
import com.peninsula.frc2023.subsystems.*;
import com.peninsula.frc2023.subsystems.controllers.drive.FSDController;
import com.peninsula.frc2023.util.Flipper;
import com.peninsula.frc2023.util.TrajectoryReader;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;

/** Used to produce {@link Commands}'s from human input. Should only be used in robot package. */
public class Control {

  /** Modifies commands based on operator input devices. */
  void updateCommands(Commands commands, @ReadOnly RobotState state) throws IOException {
    updateDriveCommands(commands, state);
    updateSuperstructureCommands(commands, state);
  }

  private void updateDriveCommands(Commands commands, RobotState state) {
    if (!state.runningFSD) commands.swerveWanted = Swerve.State.TELEOP;
    commands.boostWanted = (state.driverRt > 0.2);
    commands.robotCentricWanted = state.driverLtPressed;

    // TODO, 2 is just an arbitrary
    if (state.runningFSD && FSDController.targetReached()) {

      state.runningFSD = false;
      commands.wantedLighting = Lighting.State.ALIGNED;
    }

    if (state.runningFSD)
      SmartDashboard.putBoolean("Target reached", FSDController.targetReached());
    else SmartDashboard.putBoolean("Target reached", true);

    if (Math.abs(state.driverRightX) > 0.1
        || Math.hypot(state.driverLeftX, state.driverLeftY) > 0.1) {
      state.runningFSD = false;
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 9; j++) {
        if (state.pressedGrid[i][j]) {
          commands.wantedColumn = j;
          if (Robot.blue) {
            commands.wantedColumn = 8 - commands.wantedColumn;
          }
          commands.wantedRow = i;
        }
      }
    }

    if (state.driverAPressed && !state.runningFSD) {
      commands.wantedLighting = Lighting.State.ALIGNING;
      state.runningFSD = true;

      Translation2d wanted = gridLoc(commands.wantedColumn, commands.wantedRow);
      Rotation2d wantedRot = new Rotation2d();

      Pose2d wantedPose = new Pose2d(wanted, wantedRot);

      if (Robot.blue) {
        wantedPose = Flipper.flip(wantedPose);
      }

      commands.wantedFSDPose = wantedPose;
      commands.swerveWanted = Swerve.State.FSD;
      FSDController.resetThetaController = true;

      trySetGoalFSD(commands.wantedRow, commands.wantedColumn, state, commands);
    }

    SmartDashboard.putNumber(
        "ODO_FSD diff",
        FSDController.getRemainingDistance(
            state.poseEst.getEstimatedPosition(), commands.wantedFSDPose));

    Pose2d ghost = commands.wantedFSDPose;

    state.m_field.getObject("ghostPose").setPose(ghost);
  }

  public void trySetGoalFSD(int row, int column, RobotState state, Commands command) {
    if (row == 2) {
      if (column == 1 || column == 4 || column == 7) {
        tryStartTraj(state, command, Arm.Positions.PLACE_CUBE_H);
      } else {
        tryStartTraj(state, command, Arm.Positions.PLACE_CONE_H);
      }
    }
    if (row == 1) {
      if (column == 1 || column == 4 || column == 7) {
        tryStartTraj(state, command, Arm.Positions.PLACE_CUBE_M);
      } else {
        tryStartTraj(state, command, Arm.Positions.PLACE_CONE_M);
      }
    }
    if (row == 0) {
      tryStartTraj(state, command, Arm.Positions.HYBRID_PLACE);
    }
  }

  public double p = 14.74;
  public double arm_skew_offset = 2 * 0.0254;

  public Translation2d gridLoc(int col, int row) {
    double y_off = (Robot.blue ? -1 : 1) * arm_skew_offset;

    if (col == 8) {
      return new Translation2d(p, 4.98 + y_off);
    }
    if (col == 7) {
      return new Translation2d(14.73, 4.4 + y_off);
    }
    if (col == 6) {
      return new Translation2d(p, 3.85 + y_off);
    }
    if (col == 5) {
      return new Translation2d(p, 3.3 + y_off);
    }
    if (col == 4) {
      return new Translation2d(14.73, 2.7 + y_off);
    }
    if (col == 3) {
      return new Translation2d(p, 2.17 + y_off);
    }
    if (col == 2) {
      return new Translation2d(p, 1.61 + y_off);
    }
    if (col == 1) {
      return new Translation2d(14.73, 1.07 + y_off);
    }
    return new Translation2d(p, 0.5 + y_off);
  }

  private void updateSuperstructureCommands(Commands commands, RobotState state)
      throws IOException {
    if (!Robot.isRobotReal()) {
      if (!state.armFollowingTrajectory) {
        commands.wantedArmState = Arm.State.FOLLOW_TRAJECTORY;
        state.armFollowingTrajectory = true;
        state.armTrajectoryTimeStart = Timer.getFPGATimestamp();
        TrajectoryReader read =
            new TrajectoryReader(Filesystem.getDeployDirectory().toPath() + "/video.json");
        state.simulationActiveArmTrajectory =
            new ArmTrajectory(read.getDeltaTime(), read.getTrajectoryP1(), read.getTrajectoryP2());
      }
      state.armState =
          state.simulationActiveArmTrajectory.samplePose(
              Timer.getFPGATimestamp() - state.armTrajectoryTimeStart);
    } else {
      SmartDashboard.putBoolean("state.intake", state.intake);
      SmartDashboard.putBoolean("intakeWanted", commands.intakeWanted == Intake.State.ON);
      SmartDashboard.putBoolean("armfollowing", state.armFollowingTrajectory);
      SmartDashboard.putNumber("routinewantedsize", commands.routinesWanted.size());
      if (state.cone) {
        tryStartTraj(state, commands, Arm.Positions.PLACE_CONE_M);
        commands.wantedLighting = Lighting.State.CONE;
      } else if (state.cube) {
        tryStartTraj(state, commands, Arm.Positions.PLACE_CONE_H);
        commands.wantedLighting = Lighting.State.CUBE;
      } else if (state.stow) {
        tryStartTraj(state, commands, Arm.Positions.STOWED);
        commands.wantedLighting = Lighting.State.IDLE;
      } else if (state.double_sub) {
        tryStartTraj(state, commands, Arm.Positions.DOUBLE_SUBSTATION);
        commands.wantedLighting = Lighting.State.CONE;
      } else if (state.intake) {
        if (!state.intakeDeployed
            && state.intakeState == Intake.State.OFF
            && !state.armFollowingTrajectory) {
          state.intakeDeployed = true;
          commands.routinesWanted.add(new MoveToCubeIntake().getRoutine());
        }
      } else {
        if (state.intakeDeployed
            && state.intakeState == Intake.State.ON
            && !state.armFollowingTrajectory) {
          state.intakeDeployed = false;
          commands.routinesWanted.add(new MoveBackFromIntake().getRoutine());
        }
      }
    }

    SmartDashboard.putNumber("bat", RobotController.getBatteryVoltage());

    if (state.driverBPressed) {
      if (state.currentArmPreset == Arm.Positions.PLACE_CONE_H) {
        commands.routinesWanted.add(
            new ParallelRoutine(
                new SequentialRoutine(new TimedRoutine(0.2), new GripperSet(Gripper.State.SPIT, 1)),
                new RunArmTrajectoryRoutine(Arm.Positions.STOWED, 1)));
      }
      if (state.currentArmPreset == Arm.Positions.PLACE_CONE_M) {
        commands.routinesWanted.add(
            new ParallelRoutine(
                new SequentialRoutine(
                    new TimedRoutine(0.1), new GripperSet(Gripper.State.SPIT, 0.65)),
                new RunArmTrajectoryRoutine(Arm.Positions.STOWED, 0.65)));
      }
    }

    if (state.grip) {
      commands.gripperWanted = Gripper.State.ON;
    } else {
      commands.gripperWanted = Gripper.State.OFF;
      if (commands.wantedRow == 0) {
        commands.gripperWanted = Gripper.State.SPIT;
      } else if ((commands.wantedRow == 2 || commands.wantedRow == 1)
          && (commands.wantedColumn == 1
              || commands.wantedColumn == 4
              || commands.wantedColumn == 7)) {
        commands.gripperWanted = Gripper.State.SPIT;
      }
    }
  }

  public void tryStartTraj(RobotState state, Commands commands, Arm.Positions goal) {
    if (!state.armFollowingTrajectory && goal != state.currentArmPreset) {
      commands.wantedArmState = Arm.State.FOLLOW_TRAJECTORY;
      state.armFollowingTrajectory = true;
      state.armTrajectoryTimeStart = Timer.getFPGATimestamp();
      commands.wantedArmTrajectory = ArmConstants.paths.getPath(state.currentArmPreset, goal);
      state.currentArmPreset = goal;
    }
  }

  public void reset(Commands commands) {
    commands.routinesWanted.clear();
    commands.swerveWanted = Swerve.State.NEUTRAL;
    commands.boostWanted = false;
    commands.visionWanted = Vision.State.ON;
  }
}
