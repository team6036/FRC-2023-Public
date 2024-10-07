package com.peninsula.frc2023.util.peninsulaCoolios;

import com.peninsula.frc2023.subsystems.Arm;
import com.peninsula.frc2023.util.TrajectoryReader;
import com.peninsula.frc2023.util.peninsulaCoolios.kinematics.ArmTrajectory;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.util.HashMap;

public class PathHolder {
  HashMap<Arm.Positions, HashMap<Arm.Positions, ArmTrajectory>> paths = new HashMap<>();

  public PathHolder() throws IOException {
    Arm.Positions[] positions = {
      Arm.Positions.STOWED,
      Arm.Positions.CUBE_PICK,
      Arm.Positions.PLACE_CUBE_H,
      Arm.Positions.PLACE_CUBE_M,
      Arm.Positions.HYBRID_PLACE,
      Arm.Positions.PLACE_CONE_M,
      Arm.Positions.PLACE_CONE_H,
      Arm.Positions.DOUBLE_SUBSTATION,
      Arm.Positions.HOLD,
      Arm.Positions.INTAKE
    };

    for (int i = 0; i < positions.length; i++) {
      paths.put(positions[i], new HashMap<>());
      for (int j = 0; j < positions.length; j++) {
        if (i != j) {
          TrajectoryReader read =
              new TrajectoryReader(
                  Filesystem.getDeployDirectory().toPath()
                      + "/paths/"
                      + (positions[i] + "_to_" + positions[j])
                      + ".json");
          ArmTrajectory traj =
              new ArmTrajectory(
                  read.getDeltaTime(), read.getTrajectoryP1(), read.getTrajectoryP2());
          paths.get(positions[i]).put(positions[j], traj);
        }
      }
    }
  }

  public ArmTrajectory getPath(Arm.Positions a, Arm.Positions b) {
    return paths.get(a).get(b);
  }
}
