package com.peninsula.frc2023.util;

import com.peninsula.frc2023.util.input.Joystick;

public class MacroPad {
  public Joystick board = new Joystick(2);

  public boolean RC(int row, int column) {
    return pressed(9 * row + column);
  }

  public boolean pressed(int b) {
    return board.getRawButton(b + 1);
  }

  public boolean stow() {
    return pressed(27);
  }

  public boolean grip() {
    return !pressed(31);
  }

  public boolean cubePick() {
    return false;
  }

  public boolean down() {
    return pressed(30);
  }

  public boolean conePick() {
    return pressed(29);
  }

  public boolean doublePick() {
    return board.getRawAxis(0) == -1.0;
  }

  public boolean intake() {
    return pressed(28);
  }
}
