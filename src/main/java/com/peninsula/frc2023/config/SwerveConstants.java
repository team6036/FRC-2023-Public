package com.peninsula.frc2023.config;

import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.peninsula.frc2023.util.SwerveModuleConstants;
import com.peninsula.frc2023.util.control.Gains;
import com.peninsula.frc2023.util.input.TunableNumber;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
  public static final double kOffsetX = Units.inchesToMeters(20.75 / 2);
  public static final double kOffsetY = Units.inchesToMeters(20.75 / 2);

  public static double FLo = RobotConstants.kPracticeBot ? 359.6 - 180 : 193.007812 - 180; // enc 1
  public static double FRo = RobotConstants.kPracticeBot ? 194.3 : 311.308594 - 180; // enc 2
  public static double BLo = RobotConstants.kPracticeBot ? 122.3 : 318.955078 - 180; // enc 4
  public static double BRo = RobotConstants.kPracticeBot ? 202.9 : 170.332031 + 180; // enc 3

  public static Gains kSetAngleGains = new Gains(-0.032, 0.0001, 0.009, 0, 0, 0);

  static Translation2d kFLPos = new Translation2d(kOffsetX, kOffsetY);
  static Translation2d kFRPos = new Translation2d(kOffsetX, -kOffsetY);
  static Translation2d kBLPos = new Translation2d(-kOffsetX, kOffsetY);
  static Translation2d kBRPos = new Translation2d(-kOffsetX, -kOffsetY);

  public static final double kMaxVoltage = 12.0;

  public static final double kTeleopMaxTransVel = 4.5;
  public static final double kTeleopBoostMaxTransVel = 5.0;
  public static final double kTeleopBoostMaxRotVel = 2 * Math.PI;

  public static SwerveDriveKinematics kKinematics =
      new SwerveDriveKinematics(kFLPos, kFRPos, kBLPos, kBRPos);

  /* FSD */
  public static final TunableNumber p = new TunableNumber("FSD_P", 3.0);
  public static final TunableNumber i = new TunableNumber("FSD_I", 0.0);
  public static final TunableNumber d = new TunableNumber("FSD_D", 0.1);

  public static final double p_xy = 3;
  public static final double i_xy = 0.00001;
  public static final double d_xy = 0.3;

  public static final double p_t = -2.9;
  public static final double i_t = 0;
  public static final double d_t = -0.2;

  public static class Constants {

    public static final class Swerve {

      /* Drivetrain Constants */
      public static double wheelDiameter = Units.inchesToMeters(3.96);
      public static double wheelCircumference = wheelDiameter * Math.PI;

      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;

      public static final double driveGearRatio = 6.75;
      public static final double angleGearRatio = 21.43;

      /* Swerve Current Limiting */
      public static final int drivePeakCurrentLimit = 400;

      /* Angle Motor PID Values */
      public static final double angleKP = 0.75; // 0.75
      public static final double angleKI = 0.0;
      public static final double angleKD = 15;
      public static final double angleKF = 0.0;

      /* Drive Motor PID Values */
      public static final double driveKP = 0.002; // TODO, tune
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.0002;
      public static final double driveKF = 0.0;

      /* Drive Motor Characterization Values */
      public static final double driveKS =
          (0.022); // divide by 12 to convert from volts to percent output for CTRE
      public static final double driveKV = (0.1932052635);
      public static final double driveKA = (0);

      /* Swerve Module Constants */
      public static final double maxSpeed = 4.92; // meters per second
      public static final double maxAngularVelocity = 20;

      /* Neutral Modes Pro */
      public static final NeutralModeValue angleNeutralModePro = NeutralModeValue.Brake;
      public static final NeutralModeValue driveNeutralModePro = NeutralModeValue.Brake;

      /* Motor Inverts */
      public static final boolean driveMotorInvert = false;
      public static final boolean angleMotorInvert = true;

      /* Angle Encoder Invert */
      public static final boolean canCoderInvert = true;

      /* Module Specific Constants */
      /* Front Left Module - Module 0 */
      public static final class Mod0 {
        public static final int driveMotorID = 2;
        public static final int angleMotorID = 1;
        public static final int canCoderID = 1;
        public static final double angleOffset = SwerveConstants.FLo;
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 {
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 3;
        public static final int canCoderID = 2;
        public static final double angleOffset = SwerveConstants.FRo;
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Left Module - Module 2 */
      public static final class Mod2 {
        public static final int driveMotorID = 8;
        public static final int angleMotorID = 7;
        public static final int canCoderID = 4;
        public static final double angleOffset = SwerveConstants.BLo;
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3 {
        public static final int driveMotorID = 6;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 3;
        public static final double angleOffset = SwerveConstants.BRo;
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
    }

    public final class AutoConstants {
      public static final double kMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4 * Math.PI;

      public static final double kPXController = 1.5;
      public static final double kPYController = 1.5;
      public static final double kPThetaController = -5;

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
  }

  public static Translation2d[] getModulePos() {
    return new Translation2d[] {kFLPos, kFRPos, kBLPos, kBRPos};
  }
}
