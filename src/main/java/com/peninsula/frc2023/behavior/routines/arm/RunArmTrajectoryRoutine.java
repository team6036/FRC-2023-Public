package com.peninsula.frc2023.behavior.routines.arm;

import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.config.ArmConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Arm;
import edu.wpi.first.wpilibj.Timer;

public class RunArmTrajectoryRoutine extends TimeoutRoutineBase {

  public Arm.Positions goal;

  public RunArmTrajectoryRoutine(Arm.Positions goal, double timeout) {
    super(timeout);
    this.goal = goal;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    if (!state.armFollowingTrajectory && state.currentArmPreset != goal) {
      commands.wantedArmState = Arm.State.FOLLOW_TRAJECTORY;
      state.armFollowingTrajectory = true;
      state.armTrajectoryTimeStart = Timer.getFPGATimestamp();
      commands.wantedArmTrajectory = ArmConstants.paths.getPath(state.currentArmPreset, goal);
      state.currentArmPreset = goal;
    }
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}
}
