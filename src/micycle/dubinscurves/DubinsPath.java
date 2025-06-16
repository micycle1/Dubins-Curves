package micycle.dubinscurves;

/**
 * Represents a Dubins path, which is the shortest curve connecting two
 * configurations (position and orientation) in a 2D plane, subject to a
 * constraint on the path's curvature and with prescribed initial and terminal
 * tangents.
 * <p>
 * Dubins paths are composed of three segments, each being either a circular arc
 * (Left (L) or Right (R) turn at a constant radius) or a straight line (S).
 * There are six fundamental types of Dubins paths: LSL, LSR, RSL, RSR, RLR, LRL.
 * <p>
 * This class is the primary user-facing API for working with Dubins paths.
 * It provides methods to generate, sample, and query properties of these paths.
 * 
 * @author Michael Carleton
 */
public class DubinsPath {

	/**
	 * The initial configuration (x, y, theta) of the path.
	 * Populated by the constructor.
	 */
	public double[] configStart = new double[3];
	/**
	 * The lengths of the three segments of the path, before scaling by rho.
	 * These are the "normalized" segment lengths.
	 * Populated by the underlying DubinsCurves library.
	 */
	double[] segmentLengths = new double[3]; // package-private for direct access by DubinsCurves
	/**
	 * The turning radius (rho) of the path.
	 * This is equivalent to (forward velocity / maximum angular velocity).
	 * Populated by the constructor.
	 */
	public double rho;
	/**
	 * The type of the Dubins path (e.g., LSL, RSR).
	 * Populated by the underlying DubinsCurves library.
	 * @see DubinsPathType
	 */
	DubinsPathType type; // package-private for direct access by DubinsCurves

	/**
	 * Private constructor to initialize internal structures.
	 * Not intended for direct public use.
	 */
	private DubinsPath() {
	}

	/**
	 * Generates the shortest possible Dubins path between a starting and ending
	 * configuration.
	 * <p>
	 * A configuration is an array <code>[x, y, theta]</code>, where <code>(x, y)</code> is the
	 * Cartesian coordinate and <code>theta</code> is the orientation (heading) in radians.
	 * A heading of 0 corresponds to movement along the positive x-axis (East),
	 * and positive angles indicate counter-clockwise rotation.
	 * 
	 * @param configStart The starting configuration <code>[x, y, theta]</code>.
	 * @param configEnd   The ending configuration <code>[x, y, theta]</code>.
	 * @param rho         The turning radius of the vehicle. Must be a positive value.
	 *                    This is typically calculated as (forward velocity / maximum angular velocity).
	 * @see #DubinsPath(double[], double[], double, DubinsPathType)
	 */
	public DubinsPath(double[] configStart, double[] configEnd, double rho) {
		this(); // Initialize internal fields
		// This call populates this DubinsPath object with the shortest path details
		DubinsCurves.dubins_shortest_path(this, configStart, configEnd, rho);
	}

	/**
	 * Generates a Dubins path of a specific type between a starting and ending
	 * configuration. The resulting path is not necessarily the shortest one.
	 * <p>
	 * A configuration is an array <code>[x, y, theta]</code>, where <code>(x, y)</code> is the
	 * Cartesian coordinate and <code>theta</code> is the orientation (heading) in radians.
	 * A heading of 0 corresponds to movement along the positive x-axis (East),
	 * and positive angles indicate counter-clockwise rotation.
	 * 
	 * @param configStart The starting configuration <code>[x, y, theta]</code>.
	 * @param configEnd   The ending configuration <code>[x, y, theta]</code>.
	 * @param rho         The turning radius of the vehicle. Must be a positive value.
	 *                    This is typically calculated as (forward velocity / maximum angular velocity).
	 * @param pathType    The specific {@link DubinsPathType} to generate (e.g., LSL, RSR).
	 * @see #DubinsPath(double[], double[], double)
	 */
	public DubinsPath(double[] configStart, double[] configEnd, double rho, DubinsPathType pathType) {
		this(); // Initialize internal fields
		// This call populates this DubinsPath object with the specified path type details
		DubinsCurves.dubins_path(this, configStart, configEnd, rho, pathType);
	}

	/**
	 * Returns the total length of this Dubins path.
	 * The length is the sum of the lengths of its three segments.
	 *
	 * @return The total length of the path.
	 */
	public double getLength() {
		return DubinsCurves.dubins_path_length(this);
	}

	/**
	 * Returns the length of a specific segment of the path.
	 * Each Dubins path consists of three segments.
	 *
	 * @param segment The index of the segment (0, 1, or 2).
	 *                0: first segment, 1: second segment, 2: third segment.
	 * @return The length of the specified segment. Returns 0 if the segment index is invalid,
	 *         though behavior for invalid indices primarily depends on the underlying C library.
	 */
	public double getSegmentLength(int segment) {
		return DubinsCurves.dubins_segment_length(this, segment);
	}

	/**
	 * Returns the "normalized" length of a specific segment from the path.
	 * The normalized length is the segment length before being scaled by the turning radius <code>rho</code>.
	 * To get the actual length, multiply this value by <code>rho</code>.
	 * 
	 * @param segment The index of the segment (0, 1, or 2).
	 *                0: first segment, 1: second segment, 2: third segment.
	 * @return The normalized length of the specified segment.
	 * @see #getSegmentLength(int)
	 */
	public double getNormalisedSegmentLength(int segment) {
		// This corresponds to the values stored in the internal segmentLengths array.
		return DubinsCurves.dubins_segment_length_normalized(this, segment);
	}

	/**
	 * Returns the type of this Dubins path.
	 *
	 * @return The {@link DubinsPathType} enum representing the path's type (e.g., LSL, RSR).
	 *         May return null if the path could not be successfully computed (e.g., due to invalid turning radius).
	 */
	public DubinsPathType getPathType() {
		return type;
	}

	/**
	 * Calculates the configuration (position and heading) at a specific distance
	 * along the path.
	 * 
	 * @param t The distance along the path from the starting configuration.
	 *          Must be non-negative and less than or equal to the total path length.
	 *          If <code>t</code> is outside this range, the behavior is defined by the
	 *          underlying DubinsCurves library (often clamps to start/end).
	 * @return A <code>double[3]</code> array representing the configuration <code>[x, y, theta]</code>
	 *         at distance <code>t</code> along the path.
	 */
	public double[] sample(double t) {
		double[] q = new double[3];
		DubinsCurves.dubins_path_sample(this, t, q);
		return q;
	}

	/**
	 * Samples configurations at regular intervals along the path.
	 * This method iterates along the path, invoking the provided callback function
	 * at each <code>stepSize</code> increment.
	 * <p>
	 * The sampling process continues until the entire path has been sampled or
	 * the callback function returns a non-zero value, which signals to stop sampling.
	 *
	 * @param stepSize The distance along the path between successive samples. Must be positive.
	 * @param callback A {@link DubinsPathSamplingCallback} function that will be invoked
	 *                 for each sampled configuration. The callback receives the configuration
	 *                 <code>[x, y, theta]</code> and the cumulative distance <code>t</code> along the path.
	 *                 If the callback returns a non-zero integer, sampling is halted.
	 */
	public void sampleMany(double stepSize, DubinsPathSamplingCallback callback) {
		DubinsCurves.dubins_path_sample_many(this, stepSize, callback);
	}

	/**
	 * Returns the final configuration <code>[x, y, theta]</code> at the endpoint of the path.
	 * This should be identical to the <code>configEnd</code> provided to the constructor if the
	 * path was successfully computed.
	 * 
	 * @return A <code>double[3]</code> array representing the final configuration <code>[x, y, theta]</code>.
	 */
	public double[] getEndpoint() {
		double[] endpoint = new double[3];
		DubinsCurves.dubins_path_endpoint(this, endpoint);
		return endpoint;
	}

	/**
	 * Extracts a sub-path from the beginning of this path up to a specified length.
	 * The extracted path starts at this path's <code>configStart</code> and has a total length of <code>t</code>.
	 *
	 * @param t The desired length of the sub-path to extract.
	 *          Must be non-negative and not greater than the total length of this path.
	 *          If <code>t</code> is 0, an empty path is returned. If <code>t</code> exceeds
	 *          the path length, the returned path will be equivalent to the original path.
	 * @return A new {@link DubinsPath} object representing the extracted sub-path.
	 */
	public DubinsPath extractSubpath(double t) {
		DubinsPath extract = new DubinsPath(); // Create a new path object to be populated
		// This call populates 'extract' with the sub-path details
		DubinsCurves.dubins_extract_subpath(this, t, extract);
		return extract;
	}

}