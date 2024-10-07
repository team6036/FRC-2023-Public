package com.peninsula.frc2023.behavior.routines.drive;

import com.peninsula.frc2023.behavior.TimeoutRoutineBase;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.ReadOnly;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Swerve;

/** Points drivetrain at a vision target */
public class BalanceRoutine extends TimeoutRoutineBase {

  public BalanceRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.swerveWanted = Swerve.State.BALANCE;
    commands.angleWanted = Robot.blue ? 180 : 0;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}

  @Override
  public boolean checkIfFinishedEarly(@ReadOnly RobotState state) {
    return false;
  }
}
