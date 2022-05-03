package micycle.dubinscurves;

/**
 * A Dubins path is the shortest curve that connects two points in the 2D plane,
 * with a constraint on the curvature of the path and with prescribed initial
 * and terminal tangents to the path.
 * 
 * <p>
 * Dubins paths consist of 3 adjoined segments, that can be either left-curving,
 * right-curving or straight. There are only 6 combinations of these segments
 * that describe all possible Dubins paths.
 * 
 * @author Michael Carleton
 *
 */
public class DubinsPath {

	/** the initial configuration */
	public double[] configStart = new double[3];
	/** the lengths of the three segments */
	double[] segmentLengths = new double[3];
	/** model forward velocity / model angular velocity */
	public double rho;
	/** the path type of the path */
	DubinsPathType type;

	private DubinsPath() {
	}

	/**
	 * Generates the shortest possible Dubin path between a starting and ending
	 * configuration.
	 * <p>
	 * A configuration is <code>(x, y, theta)</code>, where theta is heading
	 * direction in radians, with zero along the line x = 0 (facing east), and
	 * counter-clockwise is positive
	 * 
	 * @param configStart starting configuration, specified as an array of
	 *                    <code>x, y, theta</code>
	 * @param configEnd   ending configuration, specified as an array of
	 *                    <code>x, y, theta</code>
	 * @param rho         turning radius of the vehicle (forward velocity divided by
	 *                    maximum angular velocity)
	 * @see #DubinsPath(double[], double[], double, DubinsPathType)
	 */
	public DubinsPath(double[] configStart, double[] configEnd, double rho) {
		this();
		DubinsCurves.dubins_shortest_path(this, configStart, configEnd, rho);
	}

	/**
	 * Generates a Dubin path between a starting and ending configuration, having a
	 * specific path type; it is not neccessarily the shortest path.
	 * <p>
	 * A configuration is <code>(x, y, theta)</code>, where theta is heading
	 * direction in radians, with zero along the line x = 0 (facing east), and
	 * counter-clockwise is positive
	 * 
	 * @param configStart starting configuration, specified as an array of
	 *                    <code>x, y, theta</code>
	 * @param configEnd   ending configuration, specified as an array of
	 *                    <code>x, y, theta</code>
	 * @param rho         turning radius of the vehicle (forward velocity divided by
	 *                    maximum angular velocity)
	 * @see #DubinsPath(double[], double[], double)
	 */
	public DubinsPath(double[] configStart, double[] configEnd, double rho, DubinsPathType pathType) {
		this();
		DubinsCurves.dubins_path(this, configStart, configEnd, rho, pathType);
	}

	/**
	 * @return the length of this path
	 */
	public double getLength() {
		return DubinsCurves.dubins_path_length(this);
	}

	/**
	 * Finds the length of a specific segment from the path.
	 *
	 * @param segment the index (0-2) of the desired segment
	 */
	public double getSegmentLength(int segment) {
		return DubinsCurves.dubins_segment_length(this, segment);
	}

	/**
	 * Finds the normalized length of a specific segment from the path.
	 * 
	 * @param segment the index (0-2) of the desired segment
	 * @return
	 */
	public double getNormalisedSegmentLength(int segment) {
		return DubinsCurves.dubins_segment_length_normalized(this, segment);
	}

	public DubinsPathType getPathType() {
		return type;
	}

	/**
	 * Calculates the configuration along the path, using the parameter t.
	 * 
	 * @param t a length measure, where 0 <= t < length(path)
	 * @return <code>(x, y, theta)</code>, where theta is the gradient/tangent of
	 *         the path at <code>(x, y)</code>
	 */
	public double[] sample(double t) {
		double[] q = new double[3];
		DubinsCurves.dubins_path_sample(this, t, q);
		return q;
	}

	/**
	 * Walks along the path at a fixed sampling interval, calling the callback
	 * function at each interval.
	 *
	 * The sampling process continues until the whole path is sampled, or the
	 * callback returns a non-zero value
	 *
	 * @param stepSize the distance along the path between each successive sample
	 * @param callback the callback function to call for each sample
	 */
	public void sampleMany(double stepSize, DubinsPathSamplingCallback callback) {
		DubinsCurves.dubins_path_sample_many(this, stepSize, callback);
	}

	/**
	 * 
	 * @return <code>(x, y, theta)</code> at the endpoint of the path
	 */
	public double[] getEndpoint() {
		double[] endpoint = new double[3];
		DubinsCurves.dubins_path_endpoint(this, endpoint);
		return endpoint;
	}

	/**
	 * Extracts a subset of the path.
	 *
	 * @param t a length measure, where 0 < t < length(path)
	 * @return a new path
	 */
	public DubinsPath extractSubpath(double t) {
		DubinsPath extract = new DubinsPath();
		DubinsCurves.dubins_extract_subpath(this, t, extract);
		return extract;
	}

}