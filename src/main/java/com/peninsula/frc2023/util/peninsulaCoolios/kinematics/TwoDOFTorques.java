package com.peninsula.frc2023.util.peninsulaCoolios.kinematics;

import com.peninsula.frc2023.config.ArmConstants;
import org.ejml.simple.SimpleMatrix;

/**
 * Wraps around the essential manipulator equation for 2 DOF long link arms. Reference:
 * http://underactuated.mit.edu/multibody.html & https://github.com/rafchuk21/arm-simulator
 */
public class TwoDOFTorques {

  public double length1;
  public double cgOffset1, cgOffset2;
  public double mass1, mass2;
  public double inertia1, inertia2;

  public double gravity;

  public static boolean enableFoc = true;

  public static double gearRatio1;
  public static double gearRatio2;

  public static double efficiencyTweak = 0.95; // 971 2018

  public static double stall_torque = (enableFoc ? 5.84 : 4.69) * efficiencyTweak;
  public static double stall_current = enableFoc ? 304 : 257;
  public static double free_speed = ((enableFoc ? 6079 : 6380) / 60.0) * 2 * Math.PI;

  public static double motorsJ1;
  public static double motorsJ2;

  public static double Rm = 12.0 / stall_current;

  public static double Kv = free_speed / 12.0;
  public static double Kt = stall_torque / stall_current;

  public static SimpleMatrix K3;
  public static SimpleMatrix K3_inverse;
  public static SimpleMatrix K4;

  public SimpleMatrix M = new SimpleMatrix(2, 2);
  public SimpleMatrix C = new SimpleMatrix(2, 2);
  public SimpleMatrix G = new SimpleMatrix(2, 1);

  public TwoDOFTorques(
      double len1, double lc1, double lc2, double mass1, double mass2, double i1, double i2) {
    this.length1 = len1;
    this.cgOffset1 = lc1;
    this.cgOffset2 = lc2;
    this.mass1 = mass1;
    this.mass2 = mass2;
    inertia1 = i1;
    inertia2 = i2;

    gearRatio1 = ArmConstants.joint1GearRatio;
    gearRatio2 = ArmConstants.joint2MotorToAxel * ArmConstants.joint2AxelToJoint;

    motorsJ1 = ArmConstants.motorsJ1;
    motorsJ2 = ArmConstants.motorsJ2;

    K3 = new SimpleMatrix(2, 2);
    K3.set(0, gearRatio1 * motorsJ1 * Kt / Rm);
    K3.set(3, gearRatio2 * motorsJ2 * Kt / Rm);

    K4 = new SimpleMatrix(2, 2);
    K4.set(0, gearRatio1 * gearRatio1 * motorsJ1 * Kt / Kv / Rm);
    K4.set(3, gearRatio2 * gearRatio2 * motorsJ2 * Kt / Kv / Rm);

    K3_inverse = K3.invert();

    //    System.out.println("k4 " + K4);

    //    System.out.println("k3i " + K3_inverse);
    // Why? Because I can.
    gravity =
        switch (Location.loc) {
          case PALO_ALTO -> 9.79922;
          case HUENEME -> 9.79641;
          case BOISE_IDAHO -> 9.80249;
          case PHOENIX_ARIZONA -> 9.79496;
          case HOUSTON -> 9.79285;
        };

    M.set(3, (cgOffset2 * cgOffset2) * mass2 + inertia2);

    gravity *= efficiencyTweak; // 971 2018 dynamics.h:160 verify?
  }

  /**
   * Equation: K₃ * V - K₄ * dq/dt = torque → V = K₃⁻¹ * (torque + K₄ * dq/dt)
   *
   * @param state represents the state and first derivative of joints
   * @param desired represented the first and second derivatives of a wanted state.
   * @return vector containing the voltages to apply to motors.
   */
  public SimpleMatrix motorFeedforward(State state, Desired desired) {
    SimpleMatrix feedforward = feedforward(state, desired);

    feedforward.plus(K4.mult(desired.omega)); // check

    return K3_inverse.mult(feedforward);
  }

  /**
   * Second order differential equation: M(dq/dt) * d²q/dt² + C(dq/dt, d²q/dt²) * dq/dt + G(q)
   *
   * @param state represents the state and first derivative of joints
   * @param desired represented the first and second derivatives of a wanted state.
   * @return vector containing torque on the joints.
   */
  public SimpleMatrix feedforward(State state, Desired desired) {
    SimpleMatrix feedforward;

    computeDynamicsMatrices(state);

    feedforward = M.mult(desired.alpha).plus(C.mult(desired.omega)).plus(G);

    //    System.out.println(feedforward);

    return feedforward;
  }

  /**
   * Computes M, C and G vector/matrices
   *
   * @param state represents the state and first derivative of joints
   */
  public void computeDynamicsMatrices(State state) {
    M(state);
    C(state);
    G(state);
  }

  /**
   * Matrix accounts for the coriolis forces. This is a function C(q, q_dot) Computes the "Coriolis
   * matrix"
   *
   * @param state represents the state and first derivative of joints
   */
  public void C(State state) {
    double hC = -mass2 * length1 * cgOffset2 * Math.sin(state.t2);

    C.set(0, hC * state.t2_dot);
    C.set(1, hC * state.t1_dot + hC * state.t2_dot);
    C.set(2, -hC * state.t1_dot);
  }

  /**
   * Vector represents the effect of gravity on the arm. Computes the "Gravity vector"
   *
   * @param state represents the state and first derivative of joints
   */
  public void G(State state) {

    double g1Scale = gravity * Math.cos(state.t1);
    double g2Scale = gravity * Math.cos(state.t1 + state.t2);

    G.set(0, (mass1 * cgOffset1 + mass2 * length1 + mass2 * cgOffset2) * g1Scale);
    G.set(1, (mass2 * cgOffset2) * g2Scale);
  }

  /**
   * Matrix accounts for the effect of inertia on the arm. This is a function M(q) Computes the
   * "Mass matrix"
   *
   * @param state represents the state and first derivative of joints
   */
  public void M(State state) {
    double hM = length1 * cgOffset2 * Math.cos(state.t2);

    M.set(
        0,
        (cgOffset1 * cgOffset1 * mass1)
            + (length1 * length1 + cgOffset2 * cgOffset2 + 2 * hM) * mass2
            + (inertia2 + inertia1));
    M.set(1, (cgOffset2 * cgOffset2 + hM) * mass2 + inertia2);
    M.set(2, (cgOffset2 * cgOffset2 + hM) * mass2 + inertia2);

    // 3 set in constructor
  }

  public static class State {
    public double t1, t2;
    public double t1_dot, t2_dot;

    public State(double t1, double t2, double t1_dot, double t2_dot) {
      this.t1 = t1;
      this.t2 = t2;
      this.t1_dot = t1_dot;
      this.t2_dot = t2_dot;
    }
  }

  public static class Desired {
    public SimpleMatrix omega; // velos
    public SimpleMatrix alpha; // accel

    public boolean applyAccel;

    public Desired(double j1_dot, double j2_dot, double j1_ddot, double j2_ddot) {
      this(j1_dot, j2_dot);

      alpha = new SimpleMatrix(2, 1);
      alpha.set(0, j1_ddot);
      alpha.set(1, j2_ddot);

      this.applyAccel = true;
    }

    public Desired(double j1_dot, double j2_dot) {
      omega = new SimpleMatrix(2, 1);
      omega.set(0, j1_dot);
      omega.set(1, j2_dot);

      this.applyAccel = false;
    }
  }
}
