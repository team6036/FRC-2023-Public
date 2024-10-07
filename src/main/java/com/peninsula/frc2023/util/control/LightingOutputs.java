package com.peninsula.frc2023.util.control;

import com.peninsula.frc2023.util.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class LightingOutputs {

  public List<Color.HSV> lightingOutput;

  public LightingOutputs() {
    lightingOutput = new ArrayList<>();
  }

  @Override // Auto-generated
  public int hashCode() {
    return Objects.hash(lightingOutput);
  }

  @Override // Auto-generated
  public boolean equals(Object other) {
    if (this == other) return true;
    if (other == null || getClass() != other.getClass()) return false;
    LightingOutputs otherOutput = (LightingOutputs) other;
    return lightingOutput.equals(otherOutput.lightingOutput);
  }
}
