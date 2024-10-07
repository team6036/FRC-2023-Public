package com.peninsula.frc2023.util.peninsulaCoolios.kinematics;

import com.peninsula.frc2023.util.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ejml.simple.SimpleMatrix;

public class ArmTrajectory {
  private final double dt;
  private final double totalTime;
  private final double[] j1, j2;
  private double controlPoints;

  public ArmTrajectory(double dt, double[] j1, double[] j2) {
    this.dt = dt;
    this.j1 = j1;
    this.j2 = j2;
    totalTime = dt * (j1.length - 1);
    controlPoints = j1.length;
  }

  public TwoDOFKinematics.ArmConfiguration samplePose(double time) {
    time = Util.clamp(time, 0, totalTime);

    int index = (int) (time / dt);
    int nexDex = (int) Math.ceil(time / dt);
    nexDex = (int) Math.min(nexDex, controlPoints - 1);

    double t = (time % dt) / dt;

    double j1L = lerp(j1[index], j1[nexDex], t);
    double j2L = lerp(j2[index], j2[nexDex], t);

    return new TwoDOFKinematics.ArmConfiguration(new Rotation2d(j1L), new Rotation2d(j2L));
  }

  public SimpleMatrix sampleVelocity(double time) {
    time = Util.clamp(time, 0, totalTime);

    int nexDex = (int) Math.ceil(time / dt);
    int prevDex = (int) Math.floor(time / dt);
    if (nexDex == prevDex) nexDex++;

    nexDex = (int) Math.min(nexDex, controlPoints - 1);
    prevDex = Math.max(prevDex, 0);

    SimpleMatrix v = new SimpleMatrix(2, 1);

    v.set(0, (j1[nexDex] - j1[prevDex]) / dt);
    v.set(1, (j2[nexDex] - j2[prevDex]) / dt);

    return v;
  }

  public SimpleMatrix sampleAcceleration(double time) {
    time = Util.clamp(time, 0, totalTime);

    int nexDex = (int) Math.ceil(time / dt);
    int prevDex = (int) Math.floor(time / dt);
    if (nexDex == prevDex) nexDex++;

    int nexDex2 = nexDex + 1;
    int prevDex2 = prevDex - 1;

    nexDex = (int) Math.min(nexDex, controlPoints - 1);
    nexDex2 = (int) Math.min(nexDex2, controlPoints - 1);
    prevDex = Math.max(prevDex, 0);
    prevDex2 = Math.max(prevDex2, 0);

    double preV1 = (j1[prevDex] - j1[prevDex2]) / dt;
    double preV2 = (j2[prevDex] - j2[prevDex2]) / dt;

    double curV1 = (j1[nexDex] - j1[prevDex]) / dt;
    double curV2 = (j2[nexDex] - j2[prevDex]) / dt;

    double nexV1 = (j1[nexDex2] - j1[nexDex]) / dt;
    double nexV2 = (j2[nexDex2] - j2[nexDex]) / dt;

    double a1, a2;

    if ((time % dt) / dt > 0.5) {
      a1 = (nexV1 - curV1) / dt;
      a2 = (nexV2 - curV2) / dt;
    } else {
      a1 = (curV1 - preV1) / dt;
      a2 = (curV2 - preV2) / dt;
    }

    SimpleMatrix a = new SimpleMatrix(2, 1);
    a.set(0, a1);
    a.set(1, a2);

    return a;
  }

  public double getTime() {
    return totalTime;
  }

  /**
   * @param a start point
   * @param b end point
   * @param t value between 0 and 1
   * @return lerped value (linear interpolation)
   */
  public double lerp(double a, double b, double t) {
    return a * (1 - t) + b * t;
  }
}
