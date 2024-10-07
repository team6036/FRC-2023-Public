import com.peninsula.frc2023.util.peninsulaCoolios.AprilTagMSTDSUGraph;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.util.Arrays;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TestGraph {
  @Test
  public void graphTest() {
    AprilTagMSTDSUGraph aprilTagMSTDSUGraph = new AprilTagMSTDSUGraph(3, -1);

    Pose3d zeroP = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    Pose3d oneP = new Pose3d(new Translation3d(0, 1, 0), new Rotation3d(0, 0, 0));
    Pose3d twoP = new Pose3d(new Translation3d(0, 2, 0), new Rotation3d(0, 0, 0));

    Transform3d zero_one = new Transform3d(new Translation3d(0, 1, 0), new Rotation3d(0, 0, 0));
    Transform3d one_two = new Transform3d(new Translation3d(0, 1, 0), new Rotation3d(0, 0, 0));
    Transform3d zero_two = new Transform3d(new Translation3d(0, 2, 0), new Rotation3d(0, 0, 0));

    aprilTagMSTDSUGraph.addEdge(0, 1, zero_one, 10);
    aprilTagMSTDSUGraph.addEdge(1, 2, one_two, 5);
    aprilTagMSTDSUGraph.addEdge(0, 2, zero_two, 9);

    AprilTag zero = new AprilTag(0, zeroP);
    AprilTag one = new AprilTag(1, oneP);
    AprilTag two = new AprilTag(2, twoP);

    aprilTagMSTDSUGraph.addPoseForRooting(zero);
    aprilTagMSTDSUGraph.addPoseForRooting(one);
    aprilTagMSTDSUGraph.addPoseForRooting(two);

    aprilTagMSTDSUGraph.MST();

    System.out.println("Edges in MST: ");
    boolean failed = false;
    for (int i = 0; i < aprilTagMSTDSUGraph.adjacencyListMST.size(); i++) {
      for (AprilTagMSTDSUGraph.Edge e : aprilTagMSTDSUGraph.adjacencyListMST.get(i)) {
        System.out.println(e.getTagId_from() + " -> " + e.getTagId_to());
        if ((e.getTagId_from() + " -> " + e.getTagId_to()).equals("0 -> 1")) failed = true;
      }
    }
    System.out.println();

    /* Checks if the Kruskal MST included the right edges */
    Assert.assertFalse("Edge 0 -> 1 exists in MST", failed);

    aprilTagMSTDSUGraph.findRootsMinDist();

    /* Checking if Floyd Warshall min dist root finding is working */
    Assert.assertEquals(aprilTagMSTDSUGraph.getRoots().size(), 1);
    Assert.assertTrue(aprilTagMSTDSUGraph.getRoots().contains(2));

    aprilTagMSTDSUGraph.fillPoses();

    System.out.println("Estimated poses");
    System.out.println(Arrays.toString(aprilTagMSTDSUGraph.getAllTagPoses()));

    /* Checks estimated poses were correct */
    Assert.assertEquals(aprilTagMSTDSUGraph.getAllTagPoses()[0], zero.pose);
    Assert.assertEquals(aprilTagMSTDSUGraph.getAllTagPoses()[1], one.pose);
    Assert.assertEquals(aprilTagMSTDSUGraph.getAllTagPoses()[2], two.pose);
  }

  @Test
  public void nonfilledGraphTest() {
    AprilTagMSTDSUGraph aprilTagMSTDSUGraph = new AprilTagMSTDSUGraph(3, -1);

    Pose3d zeroP = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    Pose3d oneP = new Pose3d(new Translation3d(0, 1, 0), new Rotation3d(0, 0, 0));

    AprilTag zero = new AprilTag(0, zeroP);
    AprilTag one = new AprilTag(1, oneP);

    aprilTagMSTDSUGraph.addPoseForRooting(zero);
    aprilTagMSTDSUGraph.addPoseForRooting(one);

    Transform3d zero_one = new Transform3d(new Translation3d(0, 1, 0), new Rotation3d(0, 0, 0));

    aprilTagMSTDSUGraph.addEdge(0, 1, zero_one, 10);

    aprilTagMSTDSUGraph.MST();

    System.out.println("Edges in MST: ");
    for (int i = 0; i < aprilTagMSTDSUGraph.adjacencyListMST.size(); i++) {
      for (AprilTagMSTDSUGraph.Edge e : aprilTagMSTDSUGraph.adjacencyListMST.get(i)) {
        System.out.println(e.getTagId_from() + " -> " + e.getTagId_to());
      }
    }

    aprilTagMSTDSUGraph.findRootsMinDist();

    /* Checking if Floyd Warshall min dist root finding is working */
    Assert.assertEquals(aprilTagMSTDSUGraph.getRoots().size(), 1);
    Assert.assertTrue(aprilTagMSTDSUGraph.getRoots().contains(0));

    aprilTagMSTDSUGraph.fillPoses();

    System.out.println("Estimated poses");
    System.out.println(Arrays.toString(aprilTagMSTDSUGraph.getAllTagPoses()));

    /* Checks estimated poses were correct */
    Assert.assertEquals(aprilTagMSTDSUGraph.getAllTagPoses()[0], zero.pose);
    Assert.assertEquals(aprilTagMSTDSUGraph.getAllTagPoses()[1], one.pose);
  }

  @Test
  public void testOnApriltags() throws IOException {
    AprilTagFieldLayout layout = new AprilTagFieldLayout("2023-chargedup.json");

    AprilTagMSTDSUGraph aprilTagMSTDSUGraph = new AprilTagMSTDSUGraph(60, -1);

    for (AprilTag tag : layout.getTags()) aprilTagMSTDSUGraph.addPoseForRooting(tag);
  }
}
