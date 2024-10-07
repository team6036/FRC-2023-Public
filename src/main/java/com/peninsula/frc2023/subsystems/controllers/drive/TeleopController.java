package com.peninsula.frc2023.subsystems.controllers.drive;

import com.peninsula.frc2023.config.SwerveConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.subsystems.Swerve;
import com.peninsula.frc2023.util.control.SwerveOutputs;
import com.peninsula.frc2023.util.swerveDrivers.SwerveOutputGenerator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TeleopController extends Swerve.SwerveController {
  // TODO, The following lines (limits and outputGenerator) should be put as SwerveConstants.
  public static final SwerveOutputGenerator.KinematicLimits limits =
      new SwerveOutputGenerator.KinematicLimits();

  public static final SwerveOutputGenerator.KinematicLimits slo_limits =
      new SwerveOutputGenerator.KinematicLimits();

  static {
    limits.kMaxDriveVelocity = SwerveConstants.Constants.Swerve.maxSpeed;
    limits.kMaxSteeringVelocity = SwerveConstants.Constants.Swerve.maxAngularVelocity;
    limits.kMaxDriveAcceleration = 10; // Arbitrary

    slo_limits.kMaxDriveVelocity = SwerveConstants.Constants.Swerve.maxSpeed * 0.5;
    slo_limits.kMaxSteeringVelocity = SwerveConstants.Constants.Swerve.maxAngularVelocity;
    slo_limits.kMaxDriveAcceleration = limits.kMaxDriveAcceleration * 0.5; // Arbitrary
  }

  public static final SwerveOutputGenerator outputGenerator =
      new SwerveOutputGenerator(SwerveConstants.kKinematics);

  public TeleopController(SwerveOutputs outputs) {
    super(outputs);
  }

  @Override
  public void updateSignal(Commands commands, RobotState state) {

    double x = state.driverLeftX * SwerveConstants.kTeleopBoostMaxTransVel;
    double y = state.driverLeftY * SwerveConstants.kTeleopBoostMaxTransVel;
    double z =
        Math.signum(state.driverRightX)
            * state.driverRightX
            * state.driverRightX
            * SwerveConstants.kTeleopBoostMaxRotVel
            * (commands.boostWanted ? 0.5 : 1);

    if (Robot.blue) {
      y = -y;
      x = -x;
    }

    ChassisSpeeds wantedChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            y, x, z, state.poseEst.getEstimatedPosition().getRotation());

    if (commands.boostWanted) {
      mOutputs =
          outputGenerator.generateSetpoint(slo_limits, mOutputs, state, wantedChassisSpeeds, 0.02);
    } else {
      mOutputs =
          outputGenerator.generateSetpoint(limits, mOutputs, state, wantedChassisSpeeds, 0.02);
    }
  }
}
