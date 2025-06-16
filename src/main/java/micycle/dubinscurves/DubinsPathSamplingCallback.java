package micycle.dubinscurves;

/**
 * A functional interface for handling sampled points along a Dubins path.
 * <p>
 * Implementations of this interface can be passed to the
 * {@link DubinsPath#sampleMany(double, DubinsPathSamplingCallback)} method.
 * The {@link #invoke(double[], double)} method will be called for each
 * configuration (point and orientation) sampled along the path.
 * <p>
 * This allows users to define custom actions to be performed at each sampled point,
 * such as collecting points, logging, or performing other calculations.
 */
@FunctionalInterface
public interface DubinsPathSamplingCallback {

    /**
     * Invoked for each sampled configuration along a Dubins path during path sampling.
     *
     * @param q A <code>double[3]</code> array representing the sampled configuration
     *          <code>[x, y, theta]</code>, where <code>(x,y)</code> are the Cartesian
     *          coordinates and <code>theta</code> is the orientation in radians.
     * @param t The distance from the start of the path to the sampled configuration <code>q</code>.
     * @return An integer value. If the value is <code>0</code> (or {@link DubinsCurves#EDUBOK}),
     *         sampling will continue. Any non-zero value will terminate the sampling
     *         process, and this non-zero value will be returned by the
     *         {@link DubinsPath#sampleMany(double, DubinsPathSamplingCallback)} method.
     */
    int invoke(double[] q, double t);
}
