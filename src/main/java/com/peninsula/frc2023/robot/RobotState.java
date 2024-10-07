package com.peninsula.frc2023.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.peninsula.frc2023.config.SwerveConstants;
import com.peninsula.frc2023.subsystems.Arm;
import com.peninsula.frc2023.subsystems.Intake;
import com.peninsula.frc2023.subsystems.Lighting;
import com.peninsula.frc2023.util.field3d.Field3d;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectory;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectoryProfiledAngles;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFTorques;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** Holds the current physical state of the robot from our sensors. */
@SuppressWarnings("java:S1104")
public class RobotState {

  public enum GamePeriod {
    AUTO,
    TELEOP,
    TESTING,
    DISABLED
  }

  public enum EEHolding {
    NONE,
    CONE,
    CUBE
  }

  /* Arm State */
  public TwoDOFKinematics.ArmConfiguration armState =
      new TwoDOFKinematics.ArmConfiguration(new Rotation2d(0), Rotation2d.fromDegrees(180));
  public TwoDOFTorques.State armQ = new TwoDOFTorques.State(0, 0, 0, 0);
  public ArmTrajectoryProfiledAngles simulationActiveArmTrajectoryProfiledAngles = null;
  public boolean armFollowingTrajectory = false;
  public double armTrajectoryTimeStart = 0;

  public ArmTrajectory simulationActiveArmTrajectory = null;

  /* End effector state */
  public EEHolding holdingPiece = EEHolding.NONE;

  /* Swerve */
  public Rotation2d gyroHeading = new Rotation2d();
  public double gyroPitch = 0;
  public double gyroRoll = 0;
  public double[] moduleEncoderPos = new double[4];

  public SwerveDrivePoseEstimator poseEst =
      new SwerveDrivePoseEstimator(
          SwerveConstants.kKinematics,
          new Rotation2d(0),
          new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
          },
          new Pose2d(),
          VecBuilder.fill(0.1, 0.1, 0.1),
          VecBuilder.fill(0.9, 0.9, 0.9));

  public SwerveModulePosition[] realModulePositions;
  public SwerveModuleState[] realModuleStates = new SwerveModuleState[4];

  {
    for (int i = 0; i < 4; i++) {
      realModuleStates[i] = new SwerveModuleState();
    }
  }

  public Pose2d lastVisionEstimatedPose = new Pose2d();
  public double odometryDeadReckonUpdateTime = 0;
  public Field2d m_field = new Field2d();
  public Field3d m_field3d = new Field3d();
  public ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
  public ChassisSpeeds chassisRelativeSpeeds = new ChassisSpeeds(0, 0, 0);

  public TimeInterpolatableBuffer<Pose2d> pastPoses = TimeInterpolatableBuffer.createBuffer(1.0);

  /* Joystick */
  public double driverLeftX, driverRightX, driverLeftY, driverRt = 0;
  public double driverRightY;
  public boolean operatorAPressed,
      operatorBPressed,
      operatorXPressed,
      operatorYPressed,
      operatorRtPressed,
      operatorLtPressed,
      operatorRbPressed,
      operatorLbPressed,
      operatorDPadLeftPressed,
      operatorDPadRightPressed;
  public double operatorLeftX, operatorLeftY, operatorRightY, operatorRightX;
  public boolean driverAPressed, driverRbPressed, driverBPressed;
  public boolean driverLtPressed;

  /* Vision */
  public PhotonPipelineResult photonPipelineResultRIO;
  public PhotonPipelineResult photonPipelineResultNETWORK;
  public double timestampLastResultRIO;
  public double timestampLastResultNETWORK;

  /* Miscellaneous */
  public GamePeriod gamePeriod = GamePeriod.DISABLED;
  public String gameData;
  public int cycles = 0;
  public double gameTimeS = 0;

  /* Auto */
  public PathPlannerTrajectory currentTrajectory;
  public Pose2d initPose = new Pose2d(0, 0, new Rotation2d(0));

  /* Intake */
  public Intake.State intakeState = Intake.State.ON;
  public boolean intakeDeployed = false;

  /* Gripper */
  public double currentTorqueGripper, gripperVelo;
  public boolean ir_reading = true;

  /* Arm Preset */
  public Arm.Positions currentArmPreset = Arm.Positions.STOWED;

  /* Lighting */
  public Lighting.State lastSet = Lighting.State.OFF;

  /* Vision */
  public PhotonPoseEstimator photonPoseEstimatorRIO = null;
  public PhotonPoseEstimator photonPoseEstimatorNETWORK = null;

  /* Intake */
  public double intakePosRotations = 0;

  /* FSD */
  public boolean runningFSD = false;

  /* Macro Pad */
  public boolean[][] pressedGrid = new boolean[3][9];

  public boolean visionOverrideOff = false;

  public boolean stow = false,
      grip = false,
      cone = false,
      cube = false,
      double_sub = false,
      intake = false,
      down = false;
}
