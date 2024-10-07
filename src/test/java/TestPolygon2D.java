import com.peninsula.frc2023.util.peninsulaCoolios.Polygon2D;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TestPolygon2D {
  @Test
  public void polygonPIPSquareTest() {
    double[] x = {0, 1, 1, 0};
    double[] y = {0, 0, 1, 1};
    Polygon2D polygon2D = new Polygon2D(x, y);

    // Inside cases
    Assert.assertTrue(polygon2D.PIP(0.5, 0.5));

    // Outside cases
    Assert.assertFalse(polygon2D.PIP(1.5, 0.5));
    Assert.assertFalse(polygon2D.PIP(-0.5, 0.5));

    // On the line
    Assert.assertTrue(polygon2D.PIP(1, 0.5));
  }

  @Test
  public void polygonPIPTriangleTest() {
    double[] x = {0, 1, 1};
    double[] y = {0, 0, 1};
    Polygon2D polygon2D = new Polygon2D(x, y);

    // Inside cases
    Assert.assertTrue(polygon2D.PIP(0.5, 0.25));
    Assert.assertTrue(polygon2D.PIP(0.5, 0.5));

    // Outside cases
    Assert.assertFalse(polygon2D.PIP(0.5, 0.6));
  }
}
