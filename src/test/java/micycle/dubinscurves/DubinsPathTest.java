package micycle.dubinscurves;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

class DubinsPathTest {

    private static final double DELTA = 1e-9;

    @Test
    void testLSLPath() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {3, 0, 0}; // End configuration
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius, DubinsPathType.LSL);

        assertNotNull(path);
        assertEquals(DubinsPathType.LSL, path.getPathType());

        // For LSL q0={0,0,0}, q1={3,0,0}, rho=1, segments are [0, 3, 0] (straight line)
        assertEquals(0.0, path.getNormalisedSegmentLength(0), DELTA);
        assertEquals(3.0, path.getNormalisedSegmentLength(1), DELTA);
        assertEquals(0.0, path.getNormalisedSegmentLength(2), DELTA);
        assertEquals(3.0, path.getLength(), DELTA);
    }

    @Test
    void testLSRPath() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {2, -2, -Math.PI/2}; // Example end configuration
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius, DubinsPathType.LSR);

        assertNotNull(path);
        assertEquals(DubinsPathType.LSR, path.getPathType());
    }

    @Test
    void testRSLPath() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {2, 2, Math.PI/2}; // Example end configuration
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius, DubinsPathType.RSL);

        assertNotNull(path);
        assertEquals(DubinsPathType.RSL, path.getPathType());
    }

    @Test
    void testRSRPath() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {3, 0, 0}; // Example end configuration (straight line for RSR too)
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius, DubinsPathType.RSR);

        assertNotNull(path);
        assertEquals(DubinsPathType.RSR, path.getPathType());
    }

    @Test
    void testRLRPath() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {0, 2, 0}; // Example end configuration
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius, DubinsPathType.RLR);

        assertNotNull(path);
        assertEquals(DubinsPathType.RLR, path.getPathType());
    }

    @Test
    void testLRLPath() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {0, -2, 0}; // Example end configuration
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius, DubinsPathType.LRL);

        assertNotNull(path);
        assertEquals(DubinsPathType.LRL, path.getPathType());
    }

    @Test
    void testPathLength() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {3, 0, 0};
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius); // Shortest path
        assertNotNull(path);
        assertEquals(3.0, path.getLength(), DELTA);
    }

    @Test
    void testGetEndpoint() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {2, 2, Math.PI/2};
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius); // Shortest path
        assertNotNull(path);

        double[] endpoint = path.getEndpoint();
        assertNotNull(endpoint);
        assertEquals(q1[0], endpoint[0], DELTA);
        assertEquals(q1[1], endpoint[1], DELTA);
        assertEquals(q1[2], endpoint[2], DELTA);
    }

    @Test
    void testSample() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {3, 0, 0}; // Straight line along x-axis
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius); // Shortest path (LSL or RSR)
        assertNotNull(path);

        double[] midPoint = path.sample(1.5); // Sample at half the path length
        assertNotNull(midPoint);
        assertEquals(1.5, midPoint[0], DELTA); // x-coordinate
        assertEquals(0.0, midPoint[1], DELTA); // y-coordinate
        assertEquals(0.0, midPoint[2], DELTA); // theta (heading)
    }

    @Test
    void testSampleMany() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {2, 0, 0}; // Path length is 2.0
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius); // Shortest path
        assertNotNull(path);

        final List<double[]> samples = new ArrayList<>();
        // Sample points with a step of 0.5.
        // For a path of length 2.0, this should give points at t=0, 0.5, 1.0, 1.5
        // The callback is (config, segment_length_along_path) -> error_code
        // Return 0 to continue sampling.
        path.sampleMany(0.5, (q, t) -> {
            samples.add(q.clone());
            return 0;
        });

        assertEquals(5, samples.size());
        // Samples should be at (0,0,0), (0.5,0,0), (1.0,0,0), (1.5,0,0)
        assertEquals(0.0, samples.get(0)[0], DELTA); // x at t=0
        assertEquals(0.0, samples.get(0)[1], DELTA); // y at t=0
        assertEquals(0.0, samples.get(0)[2], DELTA); // theta at t=0

        assertEquals(1.5, samples.get(3)[0], DELTA); // x at t=1.5
        assertEquals(0.0, samples.get(3)[1], DELTA); // y at t=1.5
        assertEquals(0.0, samples.get(3)[2], DELTA); // theta at t=1.5
    }

    @Test
    void testExtractSubpath() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {3, 0, 0};
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius); // Shortest path
        assertNotNull(path);

        DubinsPath subPath = path.extractSubpath(1.0); // Extract subpath of length 1.0
        assertNotNull(subPath);
        assertEquals(1.0, subPath.getLength(), DELTA);

        double[] subPathEndpoint = subPath.getEndpoint();
        assertNotNull(subPathEndpoint);
        assertEquals(1.0, subPathEndpoint[0], DELTA); // x-coordinate of subpath endpoint
        assertEquals(0.0, subPathEndpoint[1], DELTA); // y-coordinate
        assertEquals(0.0, subPathEndpoint[2], DELTA); // theta
    }

    @Test
    void testZeroLengthPath() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {0, 0, 0}; // Coincident start and end
        double turningRadius = 1.0;
        DubinsPath path = new DubinsPath(q0, q1, turningRadius);

        assertNotNull(path);
        assertEquals(0.0, path.getLength(), DELTA);

        double[] endpoint = path.getEndpoint();
        assertNotNull(endpoint);
        assertEquals(q0[0], endpoint[0], DELTA);
        assertEquals(q0[1], endpoint[1], DELTA);
        assertEquals(q0[2], endpoint[2], DELTA);
    }

    @Test
    void testInvalidTurningRadius() {
        double[] q0 = {0, 0, 0};
        double[] q1 = {1, 1, 0};

        DubinsPath pathWithZeroRadius = new DubinsPath(q0, q1, 0.0);
        assertNotNull(pathWithZeroRadius);
        assertNull(pathWithZeroRadius.getPathType(), "Path type should be null for zero turning radius");
        assertEquals(0.0, pathWithZeroRadius.getLength(), DELTA, "Path length should be 0 for zero turning radius");

        DubinsPath pathWithNegativeRadius = new DubinsPath(q0, q1, -1.0);
        assertNotNull(pathWithNegativeRadius);
        assertNull(pathWithNegativeRadius.getPathType(), "Path type should be null for negative turning radius");
        assertEquals(0.0, pathWithNegativeRadius.getLength(), DELTA, "Path length should be 0 for negative turning radius");
    }
}
