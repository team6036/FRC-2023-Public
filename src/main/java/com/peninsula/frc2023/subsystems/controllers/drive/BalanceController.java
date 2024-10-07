package com.peninsula.frc2023.subsystems.controllers.drive;

import com.peninsula.frc2023.config.SwerveConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Swerve;
import com.peninsula.frc2023.util.control.SwerveOutputs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BalanceController extends Swerve.SwerveController {

  PIDController angleSetPIDController;
  PIDController pitchPIDController;

  public BalanceController(SwerveOutputs outputs) {
    super(outputs);
    angleSetPIDController =
        new PIDController(
            SwerveConstants.kSetAngleGains.p,
            SwerveConstants.kSetAngleGains.i,
            SwerveConstants.kSetAngleGains.d);

    pitchPIDController = new PIDController(0.028, 0, 0.013);
    angleSetPIDController.enableContinuousInput(-180, 180);
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {
    double x =
        state.driverLeftX
            * (commands.boostWanted
                ? SwerveConstants.kTeleopBoostMaxTransVel
                : SwerveConstants.kTeleopMaxTransVel);

    double y = pitchPIDController.calculate(state.gyroPitch, 0);

    if (Math.abs(state.gyroPitch) < 5) y = 0;

    double z =
        angleSetPIDController.calculate(
            state.poseEst.getEstimatedPosition().getRotation().getDegrees(), commands.angleWanted);

    if (Robot.blue) {
      y = -y;
    }

    ChassisSpeeds wantedChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            y, x, z, state.poseEst.getEstimatedPosition().getRotation());

    SmartDashboard.putNumber("y output", y);
    SmartDashboard.putNumber(
        "pose est ang", state.poseEst.getEstimatedPosition().getRotation().getDegrees());
    mOutputs =
        TeleopController.outputGenerator.generateSetpoint(
            TeleopController.limits, mOutputs, state, wantedChassisSpeeds, 0.02);
  }
}
