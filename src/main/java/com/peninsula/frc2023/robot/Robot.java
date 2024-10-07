package com.peninsula.frc2023.robot;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.peninsula.frc2023.auto.AutoBase;
import com.peninsula.frc2023.auto.drive.FlipAuto;
import com.peninsula.frc2023.auto.drive.None;
import com.peninsula.frc2023.auto.drive.withClimb.*;
import com.peninsula.frc2023.behavior.RoutineBase;
import com.peninsula.frc2023.behavior.RoutineManager;
import com.peninsula.frc2023.config.VisionConstants;
import com.peninsula.frc2023.subsystems.*;
import com.peninsula.frc2023.subsystems.SubsystemBase;
import com.peninsula.frc2023.util.LoopOverrunDebugger;
import com.peninsula.frc2023.util.swerveDrivers.SwerveModule;
import com.sun.management.GarbageCollectionNotificationInfo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.*;
import javax.management.Notification;
import javax.management.NotificationEmitter;
import javax.management.NotificationListener;
import javax.management.openmbean.CompositeData;
import org.photonvision.PhotonPoseEstimator;

@SuppressWarnings("java:S1104")
public class Robot extends TimedRobot {

  public static final double kPeriod = 0.02;

  public static boolean blue = false;

  SendableChooser<AutoBase> autoChooser = new SendableChooser<>();
  //  SendableChooser<RoutineBase> autoChooser = new SendableChooser<>();
  RoutineBase autoRoutine;
  ShuffleboardTab tab = Shuffleboard.getTab("Auto chooser");

  private final RobotState mRobotState = new RobotState();
  private final Control mOperatorInterface = new Control();
  private final RoutineManager mRoutineManager = new RoutineManager();
  private final HardwareReader mHardwareReader = new HardwareReader();
  private final OdometryThread mOdometryThread = new OdometryThread(mRobotState);
  private final HardwareWriter mHardwareWriter = new HardwareWriter();
  private final Commands mCommands = new Commands();

  /* Subsystems */
  private final Swerve mSwerve = Swerve.getInstance();
  private final Vision mVision = Vision.getInstance();
  private final Arm mArm = Arm.getInstance();
  private final Gripper mGripper = Gripper.getInstance();
  private final Lighting mLights = Lighting.getInstance();
  private final Intake mIntake = Intake.getInstance();

  boolean setup = false;

  static {
    //     notification listener. is notified whenever a gc finishes.
    NotificationListener notificationListener =
        new NotificationListener() {
          @Override
          public void handleNotification(Notification notification, Object handback) {
            if (notification
                .getType()
                .equals(GarbageCollectionNotificationInfo.GARBAGE_COLLECTION_NOTIFICATION)) {
              //           extract garbage collection information from notification.
              GarbageCollectionNotificationInfo gcInfo =
                  GarbageCollectionNotificationInfo.from(
                      (CompositeData) notification.getUserData());

              //           access garbage collection information...
              SmartDashboard.putNumber("gc", gcInfo.getGcInfo().getDuration());
            }
          }
        };

    //     register our listener with all gc beans
    for (GarbageCollectorMXBean gcBean : ManagementFactory.getGarbageCollectorMXBeans()) {
      NotificationEmitter emitter = (NotificationEmitter) gcBean;
      emitter.addNotificationListener(notificationListener, null, null);
    }
  }

  public static boolean real = RobotBase.isReal();

  public static boolean isRobotReal() {
    return real;
  }

  // private final Set<SubsystemBase> mSubsystems = Set.of(mSwerve);
  private final Set<SubsystemBase> mSubsystems =
      Set.of(mArm, mGripper, mLights, mSwerve, mVision, mIntake);

  public static final LoopOverrunDebugger sLoopDebugger =
      new LoopOverrunDebugger("teleop", kPeriod);

  public Robot() {
    super(kPeriod);
  }

  @Override
  public void robotInit() {
    mHardwareWriter.configureHardware(mSubsystems);
    HardwareAdapter.SwerveHardware.getInstance().gyro.zeroYaw();
    if (!isRobotReal()) {
      // Used so that swerve module states are not null in simulation
      Swerve.getInstance()
          .getOutputs()
          .setOutputs(
              new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
              });
    }
    mOdometryThread.start();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void disabledInit() {
    mRobotState.gamePeriod = RobotState.GamePeriod.DISABLED;
    resetCommandsAndRoutines();

    if (!setup) {
      tab.add(autoChooser);

      autoChooser.setDefaultOption("None", new None());
      autoChooser.addOption(
          "ChargeThreePiece #",
          new com.peninsula.frc2023.auto.drive.withoutClimb.ThreePieceChargeCableAuto());
      autoChooser.addOption(
          "Flat Three Piece #",
          new com.peninsula.frc2023.auto.drive.withoutClimb.ThreePieceFlatAuto());
      autoChooser.addOption("Cone Cube + C #", new ConeStationSideAuto());
      autoChooser.addOption(
          "Cone Cube", new com.peninsula.frc2023.auto.drive.withoutClimb.ConeStationSideAuto());

      autoChooser.addOption("Station side + C #", new StationSideAuto());

      autoChooser.addOption(
          "Station side", new com.peninsula.frc2023.auto.drive.withoutClimb.StationSideAuto());
      autoChooser.addOption("Mid + C", new MidAuto());
      autoChooser.addOption("Mid", new com.peninsula.frc2023.auto.drive.withoutClimb.MidAuto());
      autoChooser.addOption("Wall side + C", new WallSideAuto());
      autoChooser.addOption(
          "Wall side", new com.peninsula.frc2023.auto.drive.withoutClimb.WallSideAuto());
      autoChooser.addOption("test", new FlipAuto());
      autoChooser.addOption("none", new None());
      autoChooser.addOption("test climb", new Climb());
      autoChooser.addOption("mid auto cone", new MidConeAuto());

      setup = true;
    }
  }

  @Override
  public void autonomousInit() {
    startStage(RobotState.GamePeriod.AUTO);

    HardwareAdapter.SwerveHardware.getInstance().gyro.zeroYaw();
    // Auto routine is being set in disabled periodic
    mCommands.addWantedRoutine(autoRoutine);
  }

  private void startStage(RobotState.GamePeriod period) {
    mRobotState.gamePeriod = period;
    resetCommandsAndRoutines();
    readRobotState();
    Swerve.getInstance().setOutputFromMeasured(mRobotState.realModuleStates);
  }

  @Override
  public void teleopInit() {
    startStage(RobotState.GamePeriod.TELEOP);
    mRobotState.visionOverrideOff = false;
  }

  @Override
  public void testInit() {
    startStage(RobotState.GamePeriod.TESTING);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void simulationPeriodic() {}

  DriverStation.Alliance gotten = DriverStation.Alliance.Invalid;

  @Override
  public void disabledPeriodic() {
    sLoopDebugger.reset();
    for (SwerveModule module : HardwareAdapter.SwerveHardware.getInstance().modules) {
      if (!module.check()) {
        module.resetToAbsolute();
        SmartDashboard.putBoolean("Module" + module.moduleNumber + "Set", false);
      } else {
        SmartDashboard.putBoolean("Module" + module.moduleNumber + "Set", true);
      }
    }

    gotten = DriverStation.getAlliance();

    blue = gotten == DriverStation.Alliance.Blue;
    SmartDashboard.putBoolean("b", blue);

    autoRoutine = autoChooser.getSelected().getRoutine();

    SmartDashboard.putNumber("pitch", mRobotState.gyroPitch);
    SmartDashboard.putNumber("roll", mRobotState.gyroRoll);
    sLoopDebugger.finish();
  }

  @Override
  public void autonomousPeriodic() {
    sLoopDebugger.reset();
    readRobotState();
    mRoutineManager.update(mCommands, mRobotState);
    updateSubsystemsAndApplyOutputs();
    sLoopDebugger.finish();
  }

  public static double lastDt = 0;

  @Override
  public void teleopPeriodic() {
    sLoopDebugger.reset();
    readRobotState();
    try {
      mOperatorInterface.updateCommands(mCommands, mRobotState);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    mRoutineManager.update(mCommands, mRobotState);
    updateSubsystemsAndApplyOutputs();
    sLoopDebugger.finish();
    lastDt = sLoopDebugger.mTimer.get();
    SmartDashboard.putNumber("odom count", mOdometryThread.odomCount);
  }

  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  private void resetCommandsAndRoutines() {
    mOperatorInterface.reset(mCommands);
    mRoutineManager.clearRunningRoutines();
    updateSubsystemsAndApplyOutputs();
  }

  private void readRobotState() {
    mHardwareReader.readState(mSubsystems, mRobotState);
  }

  private void resetOdometryIfWanted() {
    Pose2d wantedPose = mCommands.driveWantedOdometryPose;
    Rotation2d wantedPoseRotation = mCommands.driveWantedOdometryPoseRotation;
    if (wantedPose != null && wantedPoseRotation != null) {
      mRobotState.poseEst.resetPosition(
          wantedPoseRotation,
          new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0))
          },
          wantedPose);
      System.out.println("RESET POSE");
      mCommands.driveWantedOdometryPose = null;
      mCommands.driveWantedOdometryPoseRotation = null;
    }
  }

  private void updateSubsystemsAndApplyOutputs() {
    for (SubsystemBase subsystem : mSubsystems) {
      subsystem.update(mCommands, mRobotState);
      sLoopDebugger.addPoint(subsystem.getName());
    }
    mHardwareWriter.writeHardware(mSubsystems, mRobotState);
    sLoopDebugger.addPoint("updateSubsystemsAndApplyOutputs");
  }

  public class OdometryThread extends Thread {

    private HardwareAdapter.SwerveHardware mSwerveHardwareInstance;
    private HardwareAdapter.VisionHardware mVisionHardwareInstance;
    private BaseStatusSignalValue[] mAllSignals;
    private RobotState mRobotState;
    private long loopStartNano = 0, loopEndNano = 0;
    public boolean mIsRunning = true;
    public int odomCount = 0;

    public OdometryThread(RobotState state) {
      super();
      mRobotState = state;
      mSwerveHardwareInstance = HardwareAdapter.SwerveHardware.getInstance();
      mVisionHardwareInstance = HardwareAdapter.VisionHardware.getInstance();
      mAllSignals = new BaseStatusSignalValue[4 * 4]; // 2 gyro + 4 * 4 for module motors
      int modCount = 0;
      for (SwerveModule module : mSwerveHardwareInstance.modules) {
        BaseStatusSignalValue[] moduleSignals = module.getBaseSignals();
        for (int i = 0; i < 4; i++) {
          mAllSignals[(modCount * 4) + i] = moduleSignals[i];
        }
        modCount++;
      }

      /* setup */
      if (mRobotState.photonPoseEstimatorRIO == null) {
        odomCount++;
        mRobotState.photonPoseEstimatorRIO =
            new PhotonPoseEstimator(
                VisionConstants.aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                mVisionHardwareInstance.cameraRIO,
                VisionConstants.robot_to_camera_RIO);
      }
      if (mRobotState.photonPoseEstimatorNETWORK == null) {
        odomCount++;
        mRobotState.photonPoseEstimatorNETWORK =
            new PhotonPoseEstimator(
                VisionConstants.aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
                mVisionHardwareInstance.cameraNETWORK,
                VisionConstants.robot_to_camera_NETWORK);
      }
      odomCount++;
      mRobotState.photonPoseEstimatorNETWORK.setMultiTagFallbackStrategy(
          PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE);
      mRobotState.photonPoseEstimatorRIO.setMultiTagFallbackStrategy(
          PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_LAST_POSE);
    }

    public void run() {
      while (mIsRunning) {
        odomCount++;
        loopStartNano = System.nanoTime();
        // Read swerve hardware values (motors + gyro)
        BaseStatusSignalValue.waitForAll(0.1, mAllSignals);

        // NOTE, pigeon will require opposite sign as NAVX
        mRobotState.gyroHeading =
            Rotation2d.fromDegrees(-1 * mSwerveHardwareInstance.gyro.getYaw());

        mRobotState.gyroPitch = mSwerveHardwareInstance.gyro.getPitch();
        mRobotState.gyroRoll = mSwerveHardwareInstance.gyro.getRoll();

        for (int i = 0; i < 4; i++) {
          SwerveModuleState a = mSwerveHardwareInstance.modules[i].getState();
          mRobotState.moduleEncoderPos[i] = a.angle.getDegrees();
          mRobotState.realModuleStates[i] = a;
        }

        if (!Robot.isRobotReal()) {
          mRobotState.realModulePositions = Swerve.getInstance().getOutputs().getPositions();
          mRobotState.realModuleStates = Swerve.getInstance().getOutputs().getStates();
        } else {

          mRobotState.realModulePositions =
              new SwerveModulePosition[] {
                mSwerveHardwareInstance.modules[0].getPosition(),
                mSwerveHardwareInstance.modules[1].getPosition(),
                mSwerveHardwareInstance.modules[2].getPosition(),
                mSwerveHardwareInstance.modules[3].getPosition()
              };
        }

        // Update odometry pose estimation using values from above
        RobotStateEstimator.getInstance().updateSwervePoseEstimator(mRobotState);

        // Vision
        mRobotState.photonPipelineResultRIO = mVisionHardwareInstance.cameraRIO.getLatestResult();
        mRobotState.photonPipelineResultNETWORK =
            mVisionHardwareInstance.cameraNETWORK.getLatestResult();

        if (mRobotState.photonPipelineResultRIO.getTimestampSeconds()
                != mRobotState.timestampLastResultRIO
            || mRobotState.photonPipelineResultNETWORK.getTimestampSeconds()
                != mRobotState.timestampLastResultNETWORK) {
          mRobotState.timestampLastResultRIO =
              mVisionHardwareInstance.cameraRIO.getLatestResult().getTimestampSeconds();
          mRobotState.timestampLastResultNETWORK =
              mVisionHardwareInstance.cameraNETWORK.getLatestResult().getTimestampSeconds();
          RobotStateEstimator.getInstance().updateVisionPoseEstimator(mRobotState);
        }

        resetOdometryIfWanted();

        loopEndNano = System.nanoTime();

        SmartDashboard.putNumber("Odometry loop time", (loopEndNano - loopStartNano) / 1_000_000.0);
        SmartDashboard.putNumber("Odometry Hz", 1e9 / (loopEndNano - loopStartNano));

        SmartDashboard.putNumber(
            "Chassis Speed",
            Math.sqrt(
                Math.pow(mRobotState.chassisRelativeSpeeds.vxMetersPerSecond, 2)
                    + Math.pow(mRobotState.chassisRelativeSpeeds.vyMetersPerSecond, 2)));
        for (int i = 0; i < mRobotState.realModuleStates.length; ++i) {
          SmartDashboard.putNumber(
              "Real Abs Module Speed " + i,
              Math.abs(mRobotState.realModuleStates[i].speedMetersPerSecond));
        }
      }
    }
  }
}
