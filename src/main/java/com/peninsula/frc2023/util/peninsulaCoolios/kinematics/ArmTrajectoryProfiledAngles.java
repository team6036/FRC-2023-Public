package com.peninsula.frc2023.util.peninsulaCoolios.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmTrajectoryProfiledAngles {

  private final double accelTime1, accelTime2, maxTime;
  private final double accel1, accel2;
  public final TwoDOFKinematics.ArmConfiguration pose_init, pose_end;
  private final double flatTime1, flatTime2;
  private final boolean flip1, flip2;

  public ArmTrajectoryProfiledAngles(
      double accelTime1,
      double accelTime2,
      double accel1,
      double accel2,
      double maxTime,
      TwoDOFKinematics.ArmConfiguration init,
      TwoDOFKinematics.ArmConfiguration end,
      boolean flip1,
      boolean flip2) {

    this.accelTime1 = accelTime1;
    this.accelTime2 = accelTime2;
    this.maxTime = maxTime;
    this.pose_init = init;
    this.pose_end = end;

    this.flip1 = flip1;
    this.flip2 = flip2;

    this.accel1 = accel1;
    this.accel2 = accel2;

    flatTime1 = maxTime - accelTime1 * 2;
    flatTime2 = maxTime - accelTime2 * 2;
  }

  public double getTime() {
    return maxTime;
  }

  public TwoDOFKinematics.ArmConfiguration sample(double t) {
    double t1 = t;
    double t2 = t;

    if (flip1) t1 = maxTime - t1;
    if (flip2) t2 = maxTime - t2;

    double deltaJ1 = sampleTrapezoid(accel1, accelTime1, accel1 * accelTime1, flatTime1, t1);
    double deltaJ2 = sampleTrapezoid(accel2, accelTime2, accel2 * accelTime2, flatTime2, t2);

    Rotation2d start = pose_init.getAngle1();
    Rotation2d start2 = pose_init.getAngle2();

    if (flip1) start = pose_end.getAngle1();
    if (flip2) start2 = pose_end.getAngle2();

    Rotation2d j1 = start.plus(new Rotation2d(deltaJ1));
    Rotation2d j2 = start2.plus(new Rotation2d(deltaJ2));

    return new TwoDOFKinematics.ArmConfiguration(j1, j2);
  }

  public double[] vaSample(double t) {
    double t1 = t;
    double t2 = t;
    if (flip1) t1 = maxTime - t1;
    if (flip2) t2 = maxTime - t2;

    double v1 = sampleTrapezoidVel(accel1, accelTime1, accel1 * accelTime1, flatTime1, t1);
    double v2 = sampleTrapezoidVel(accel2, accelTime2, accel2 * accelTime2, flatTime2, t2);

    double a1 = sampleTrapezoidAcc(accel1, accelTime1, accel1 * accelTime1, flatTime1, t1);
    double a2 = sampleTrapezoidAcc(accel2, accelTime2, accel2 * accelTime2, flatTime2, t2);

    return new double[] {v1, v2, a1, a2};
  }

  public double[] vaSampleP(double p11, double p22) {

    p11 -= flip1 ? pose_end.getAngle1().getRadians() : pose_init.getAngle1().getRadians();
    p22 -= flip2 ? pose_end.getAngle2().getRadians() : pose_init.getAngle2().getRadians();

    double v1 = sampleTrapezoidVelP(accel1, accelTime1, accel1 * accelTime1, flatTime1, p11);
    double v2 = sampleTrapezoidVelP(accel2, accelTime2, accel2 * accelTime2, flatTime2, p22);

    double a1 = sampleTrapezoidAccP(accel1, accelTime1, accel1 * accelTime1, flatTime1, p11);
    double a2 = sampleTrapezoidAccP(accel2, accelTime2, accel2 * accelTime2, flatTime2, p22);

    return new double[] {v1, v2, a1, a2};
  }

  private double sampleTrapezoid(double a, double aT, double f, double fT, double t) {
    double aDelta = 0.5 * a * aT * aT;
    double fDelta = f * fT;

    if (t <= aT) return 0.5 * a * t * t;
    if (t <= fT + aT) return aDelta + fDelta * ((t - aT) / fT);
    double aat = t - aT - fT;
    return aDelta + fDelta + (f * aat - 0.5 * a * aat * aat);
  }

  private double sampleTrapezoidVel(double a, double aT, double f, double fT, double t) {
    if (t <= aT) return a * t;
    if (t <= fT + aT) return a * aT;
    return a * aT - a * (t - aT - fT);
  }

  //  0.5 * a * aT * aT + a * aT * fT + a * aT * t - 0.5 * a * t^2 = position
  private double sampleTrapezoidVelP(double a, double aT, double f, double fT, double position) {
    if (position <= 0.5 * a * aT * aT) return Math.sqrt(position / (0.5 * a));
    if (position <= 0.5 * a * aT * aT + a * aT * fT) return a * aT;

    double aQ = -0.5 * a;
    double bQ = a * aT;
    double cQ = 0.5 * a * aT * aT + a * aT * fT - position;

    double quad = (-bQ + Math.sqrt(bQ * bQ - 4 * aQ * cQ)) / (2 * aQ);

    return a * aT - a * (quad);
  }

  private double sampleTrapezoidAcc(double a, double aT, double f, double fT, double t) {
    if (t <= aT) return a;
    if (t <= fT + aT) return 0;
    return -a;
  }

  private double sampleTrapezoidAccP(double a, double aT, double f, double fT, double position) {
    if (position <= 0.5 * a * aT * aT + 1e-6) return a;
    if (position <= 0.5 * a * aT * aT + f * fT) return 0;
    return -a;
  }

  public static ArmTrajectoryProfiledAngles generate(
      TwoDOFKinematics.ArmConfiguration pose_init,
      TwoDOFKinematics.ArmConfiguration pose_end,
      JointMotionProfileConfig motionProfileConfigJ1,
      JointMotionProfileConfig motionProfileConfigJ2) {

    double rotJ1Init = pose_init.getAngle1().getRadians();
    double rotJ2Init = pose_init.getAngle2().getRadians();

    double rotJ1End = pose_end.getAngle1().getRadians();
    double rotJ2End = pose_end.getAngle2().getRadians();

    boolean flip1 = false;
    if (rotJ1Init > rotJ1End) {
      flip1 = true;
      double temp = rotJ1End;
      rotJ1End = rotJ1Init;
      rotJ1Init = temp;
    }

    boolean flip2 = false;
    if (rotJ2Init > rotJ2End) {
      flip2 = true;
      double temp = rotJ2End;
      rotJ2End = rotJ2Init;
      rotJ2Init = temp;
    }

    motionProfileConfigJ1.accel = Math.abs(motionProfileConfigJ1.accel);
    motionProfileConfigJ2.accel = Math.abs(motionProfileConfigJ2.accel);
    motionProfileConfigJ1.maxVel = Math.abs(motionProfileConfigJ1.maxVel);
    motionProfileConfigJ2.maxVel = Math.abs(motionProfileConfigJ2.maxVel);

    JointMotionProfile motionProfileJ1 = getProfile(rotJ1Init, rotJ1End, motionProfileConfigJ1);
    JointMotionProfile motionProfileJ2 = getProfile(rotJ2Init, rotJ2End, motionProfileConfigJ2);

    double t1 = motionProfileJ1.accelTime * 2 + motionProfileJ1.flatTime;
    double t2 = motionProfileJ2.accelTime * 2 + motionProfileJ2.flatTime;

    double maxTime = Math.max(t1, t2);

    double correctedAccelTimeJ1 =
        solveT((rotJ1End - rotJ1Init) / 2, maxTime, motionProfileConfigJ1.accel);
    double correctedAccelTimeJ2 =
        solveT((rotJ2End - rotJ2Init) / 2, maxTime, motionProfileConfigJ2.accel);

    return new ArmTrajectoryProfiledAngles(
        correctedAccelTimeJ1,
        correctedAccelTimeJ2,
        motionProfileConfigJ1.accel,
        motionProfileConfigJ2.accel,
        maxTime,
        pose_init,
        pose_end,
        flip1,
        flip2);
  }

  /**
   * Solution for t in Δx = 0.5at² + at(0.5m − t)
   *
   * @param d (d) displacement at halfway point (rad)
   * @param timeToEnd (m) time to the end of trajectory (s).
   * @param a (a) acceleration (rad/s^2)
   * @return time to accelerate (s)
   */
  private static double solveT(double d, double timeToEnd, double a) {
    double inside = a * a * timeToEnd * timeToEnd - 8 * d * a;
    if (inside < 0.001) inside = 0;
    return (a * timeToEnd - Math.sqrt(inside)) / (2 * a);
  }

  public static JointMotionProfile getProfile(
      double a, double b, JointMotionProfileConfig motionProfileConfig) {

    double halfMoveJ1 = (b - a) / 2;

    double accelTime = motionProfileConfig.maxVel / motionProfileConfig.accel;
    double flatNeededTime = 0;

    double accelToMaxVeloDx = 0.5 * motionProfileConfig.accel * accelTime * accelTime;

    if (accelToMaxVeloDx < halfMoveJ1) { // Trapezoid
      flatNeededTime = (halfMoveJ1 - accelToMaxVeloDx) / motionProfileConfig.maxVel;
    } else { // Triangle profile
      accelTime = Math.sqrt(halfMoveJ1 * 2 / motionProfileConfig.accel);
    }

    return new JointMotionProfile(accelTime, flatNeededTime * 2, motionProfileConfig);
  }

  public static class JointMotionProfile {
    private double accelTime;
    private double flatTime;
    private JointMotionProfileConfig motionProfileConfig;

    public JointMotionProfile(
        double accelTime, double flatTime, JointMotionProfileConfig motionProfileConfig) {
      this.accelTime = accelTime;
      this.flatTime = flatTime;
      this.motionProfileConfig = motionProfileConfig;
    }
  }

  public static class JointMotionProfileConfig {
    private double accel; // rad/s^2
    private double maxVel; // rad/s

    public JointMotionProfileConfig(double accel, double maxVel) {
      this.accel = accel;
      this.maxVel = maxVel;
    }
  }
}
