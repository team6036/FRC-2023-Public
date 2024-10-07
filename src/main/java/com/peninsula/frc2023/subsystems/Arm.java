package com.peninsula.frc2023.subsystems;

import com.peninsula.frc2023.config.ArmConstants;
import com.peninsula.frc2023.robot.Commands;
import com.peninsula.frc2023.robot.Robot;
import com.peninsula.frc2023.robot.RobotState;
import com.peninsula.frc2023.util.control.ControllerOutput;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectory;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectoryProfiledAngles;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFKinematics;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.TwoDOFTorques;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;

public class Arm extends SubsystemBase {

  public enum State {
    SET_ANGLES,
    SET_POSITION,
    FOLLOW_TRAJECTORY_PROFILED_ANGLES,
    FOLLOW_TRAJECTORY
  }

  public enum Positions {
    CUBE_PICK,
    STOWED,
    HYBRID_PLACE,
    PLACE_CUBE_H,
    PLACE_CUBE_M,
    PLACE_CONE_M,
    PLACE_CONE_H,
    CONE_SUBSTATION,
    DOWN_CONE_H,
    DOUBLE_SUBSTATION,
    HOLD,
    INTAKE
  }

  private static final Arm sInstance = new Arm();
  private State mState = State.SET_ANGLES;
  private final ControllerOutput joint1Output = new ControllerOutput();
  private final ControllerOutput joint2Output = new ControllerOutput();

  private final PIDController j1 =
      new PIDController(
          ArmConstants.joint1MK2Gains.p,
          ArmConstants.joint1MK2Gains.i,
          ArmConstants.joint1MK2Gains.d);
  private final PIDController j2 =
      new PIDController(
          ArmConstants.joint2MK2Gains.p,
          ArmConstants.joint2MK2Gains.i,
          ArmConstants.joint2MK2Gains.d);

  public Rotation2d want1 = new Rotation2d();
  public Rotation2d want2 = new Rotation2d();

  public static Arm getInstance() {
    return sInstance;
  }

  public ControllerOutput getJoint1Output() {
    return joint1Output;
  }

  public ControllerOutput getJoint2Output() {
    return joint2Output;
  }

  public Arm() {}

  @Override
  public void update(Commands commands, RobotState state) {
    State wantedState = commands.wantedArmState;
    boolean isNewState = mState != wantedState;
    mState = wantedState;

    switch (wantedState) {
      case SET_ANGLES:
        setAngles(
            commands.wantedSetAngles.getAngle1(),
            commands.wantedSetAngles.getAngle2(),
            state.armState);
        break;
      case SET_POSITION:
        Optional<TwoDOFKinematics.ArmConfiguration> wantedConfigOptional =
            ArmConstants.dof.inverseKinematics(commands.wantedArmPosition);
        if (wantedConfigOptional.isEmpty()) break;

        setAngles(
            wantedConfigOptional.get().getAngle1(),
            wantedConfigOptional.get().getAngle2(),
            state.armState);
        break;
      case FOLLOW_TRAJECTORY_PROFILED_ANGLES:
        double timeInTrajectory = Timer.getFPGATimestamp() - state.armTrajectoryTimeStart;

        ArmTrajectoryProfiledAngles wantedTraj;

        if (Robot.isRobotReal()) {
          wantedTraj = commands.wantedArmTrajectoryProfiledAngles;
        } else {
          wantedTraj = state.simulationActiveArmTrajectoryProfiledAngles;
        }

        if (timeInTrajectory > wantedTraj.getTime()) {
          setAnglesTraj(
              wantedTraj.sample(wantedTraj.getTime()).getAngle1(),
              wantedTraj.sample(wantedTraj.getTime()).getAngle2(),
              new TwoDOFTorques.Desired(0, 0, 0, 0),
              state.armQ,
              state.holdingPiece);
          break;
        }
        var sample = wantedTraj.sample(timeInTrajectory);

        var sampleVa = wantedTraj.vaSample(timeInTrajectory);

        SmartDashboard.putNumberArray("armSampleVA", sampleVa);

        TwoDOFTorques.Desired des =
            new TwoDOFTorques.Desired(
                sampleVa[0], sampleVa[1] + sampleVa[0], sampleVa[2], sampleVa[3] + sampleVa[2]);

        TwoDOFKinematics.ArmConfiguration sampleWantedConfig = wantedTraj.sample(timeInTrajectory);

        if (!Robot.isRobotReal()) {
          state.armQ =
              new TwoDOFTorques.State(
                  sample.getAngle1().getRadians(),
                  sample.getAngle2().getRadians(),
                  sampleVa[0],
                  sampleVa[1]);
        }

        setAnglesTraj(
            sampleWantedConfig.getAngle1(),
            sampleWantedConfig.getAngle2(),
            des,
            state.armQ,
            state.holdingPiece);
        break;
      case FOLLOW_TRAJECTORY:
        timeInTrajectory = Timer.getFPGATimestamp() - state.armTrajectoryTimeStart;

        ArmTrajectory wantedTrajN;

        if (Robot.isRobotReal()) {
          wantedTrajN = commands.wantedArmTrajectory;
        } else {
          wantedTrajN = state.simulationActiveArmTrajectory;
        }

        if (timeInTrajectory > wantedTrajN.getTime()) {
          setAnglesTraj(
              wantedTrajN.samplePose(wantedTrajN.getTime()).getAngle1(),
              wantedTrajN.samplePose(wantedTrajN.getTime()).getAngle2(),
              new TwoDOFTorques.Desired(0, 0, 0, 0),
              state.armQ,
              state.holdingPiece);
          state.armFollowingTrajectory = false;
          break;
        }

        sampleWantedConfig = wantedTrajN.samplePose(timeInTrajectory);
        SimpleMatrix sampleWantedV = wantedTrajN.sampleVelocity(timeInTrajectory);
        SimpleMatrix sampleWantedA = wantedTrajN.sampleAcceleration(timeInTrajectory);

        des =
            new TwoDOFTorques.Desired(
                sampleWantedV.get(0),
                sampleWantedV.get(1),
                sampleWantedA.get(0),
                sampleWantedA.get(1));

        if (!Robot.isRobotReal()) {
          state.armQ =
              new TwoDOFTorques.State(
                  sampleWantedConfig.getAngle1().getRadians(),
                  sampleWantedConfig.getAngle2().getRadians(),
                  0,
                  0);
        }

        setAnglesTraj(
            sampleWantedConfig.getAngle1(),
            sampleWantedConfig.getAngle2(),
            des,
            state.armQ,
            state.holdingPiece);

        break;
    }
  }

  public void setAnglesTraj(
      Rotation2d angle1,
      Rotation2d angle2,
      TwoDOFTorques.Desired des,
      TwoDOFTorques.State state,
      RobotState.EEHolding piece) {

    SimpleMatrix ffvec =
        switch (piece) {
          case NONE -> ArmConstants.torquesFF.motorFeedforward(state, des);
          case CONE -> ArmConstants.torquesFF_with_cone.motorFeedforward(state, des);
          case CUBE -> ArmConstants.torquesFF.motorFeedforward(state, des);
        };

    SmartDashboard.putNumber("s_t1", state.t1);
    SmartDashboard.putNumber("s_t2", state.t2);

    SmartDashboard.putNumber("s_t1_dot", state.t1_dot);
    SmartDashboard.putNumber("s_t2_dot", state.t2_dot);

    SmartDashboard.putNumber("des_t1_dot", des.omega.get(0));
    SmartDashboard.putNumber("des_t2_dot", des.omega.get(1));

    SmartDashboard.putNumber("des_t1_2dot", des.alpha.get(0));
    SmartDashboard.putNumber("des_t2_2dot", des.alpha.get(1));

    double feedforward1 = ffvec.get(0);
    double feedforward2 = ffvec.get(1);

    SmartDashboard.putNumber("feed1", feedforward1);
    SmartDashboard.putNumber("feed2", feedforward2);

    SmartDashboard.putNumber("des_t1", angle1.getRadians());
    SmartDashboard.putNumber("des_t2", angle2.getRadians());

    joint1Output.setVolt(feedforward1 + j1.calculate(state.t1, angle1.getRadians()));
    joint2Output.setVolt(feedforward2 + j2.calculate(state.t2, angle2.getRadians()));

    want1 = angle1;
    want2 = angle2;
  }

  public void setAngles(
      Rotation2d angle1, Rotation2d angle2, TwoDOFKinematics.ArmConfiguration configuration) {

    double feedforward1 =
        ArmConstants.torqueOnJ1(configuration) * ArmConstants.antiGravFeedforwardJ1Coeff;
    double feedforward2 =
        ArmConstants.torqueOnJ2(configuration) * ArmConstants.antiGravFeedforwardJ2Coeff;

    joint1Output.setTargetPosition(
        joint1AngleToMotor(angle1), feedforward1, ArmConstants.jointOld1Gains);
    joint2Output.setTargetPosition(
        joint2AngleToMotor(angle1, angle2), feedforward2, ArmConstants.jointOld2Gains);

    want1 = angle1;
    want2 = angle2;
  }

  /** checked */
  public double joint1AngleToMotor(Rotation2d angle) {
    return angle.getRotations() * ArmConstants.joint1GearRatio;
  }

  /** checked */
  public Rotation2d motorToJoint1Angle(double motor) {
    return Rotation2d.fromRotations(motor / ArmConstants.joint1GearRatio);
  }

  /**
   * checked
   *
   * @return a1 * ma + a2 * ma * aj = mot2
   */
  public double joint2AngleToMotor(Rotation2d angle1, Rotation2d angle2) {
    double offset = angle1.getRotations() * ArmConstants.joint2MotorToAxel;
    double movement =
        angle2.getRotations() * ArmConstants.joint2MotorToAxel * ArmConstants.joint2AxelToJoint;
    return offset + movement;
  }

  /** a2 = (mot2 - a1 * ma) / (ma * aj ) */
  public Rotation2d motorToJoint2FrameAngle(double motor1, double motor2) {

    Rotation2d angle1 = motorToJoint1Angle(motor1);

    return Rotation2d.fromRotations(
        (motor2 - angle1.getRotations() * ArmConstants.joint2MotorToAxel)
            / (ArmConstants.joint2MotorToAxel * ArmConstants.joint2AxelToJoint));
  }
}
