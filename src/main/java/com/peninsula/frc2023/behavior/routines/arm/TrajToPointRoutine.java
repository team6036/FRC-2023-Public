package com.peninsula.frc2023.behavior.routines.arm;

import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Arm;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectoryProfiledAngles;
import edu.wpi.first.wpilibj.Timer;

public class TrajToPointRoutine extends TimeoutRoutineBase {

  private final ArmTrajectoryProfiledAngles trajectoryProfiledAngles;
  private boolean running = false;

  public TrajToPointRoutine(ArmTrajectoryProfiledAngles trajectory, double timeout) {
    super(timeout);
    this.trajectoryProfiledAngles = trajectory;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    if (!running) {
      commands.wantedArmTrajectoryProfiledAngles = trajectoryProfiledAngles;
      commands.wantedArmState = Arm.State.FOLLOW_TRAJECTORY_PROFILED_ANGLES;
      state.armFollowingTrajectory = true;
      state.armTrajectoryTimeStart = Timer.getFPGATimestamp();

      running = true;
    }
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}
}
