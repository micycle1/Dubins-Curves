package micycle.dubinscurves;

/**
 * Represents the individual types of segments that can form a Dubins path.
 * A Dubins path consists of three such segments.
 * This enum is package-private as it's an internal detail used by {@link DubinsPathType}.
 */
enum SegmentType {
	/** A segment representing a turn to the Left at the maximum turning radius. */
	L_SEG,
	/** A segment representing travel in a Straight line. */
	S_SEG,
	/** A segment representing a turn to the Right at the maximum turning radius. */
	R_SEG;
}

/**
 * Defines the six fundamental types of Dubins paths.
 * <p>
 * A Dubins path is the shortest curve connecting two configurations (position and orientation)
 * in a 2D plane, subject to a constraint on the path's curvature. These paths are
 * constructed by concatenating three segments, where each segment is either a
 * circular arc (Left (L) or Right (R) turn at a constant radius) or a straight line (S).
 * <p>
 * The six types of Dubins paths are:
 * <ul>
 *   <li>LSL (Left turn, Straight segment, Left turn)</li>
 *   <li>LSR (Left turn, Straight segment, Right turn)</li>
 *   <li>RSL (Right turn, Straight segment, Left turn)</li>
 *   <li>RSR (Right turn, Straight segment, Right turn)</li>
 *   <li>RLR (Right turn, Left turn, Right turn)</li>
 *   <li>LRL (Left turn, Right turn, Left turn)</li>
 * </ul>
 * This enumeration provides these types and the sequence of {@link SegmentType} they correspond to.
 */
public enum DubinsPathType {

	/** Path consists of a Left turn, then a Straight segment, then a Left turn. */
	LSL(new SegmentType[] { SegmentType.L_SEG, SegmentType.S_SEG, SegmentType.L_SEG }),
	/** Path consists of a Left turn, then a Straight segment, then a Right turn. */
	LSR(new SegmentType[] { SegmentType.L_SEG, SegmentType.S_SEG, SegmentType.R_SEG }),
	/** Path consists of a Right turn, then a Straight segment, then a Left turn. */
	RSL(new SegmentType[] { SegmentType.R_SEG, SegmentType.S_SEG, SegmentType.L_SEG }),
	/** Path consists of a Right turn, then a Straight segment, then a Right turn. */
	RSR(new SegmentType[] { SegmentType.R_SEG, SegmentType.S_SEG, SegmentType.R_SEG }),
	/** Path consists of a Right turn, then a Left turn, then a Right turn. */
	RLR(new SegmentType[] { SegmentType.R_SEG, SegmentType.L_SEG, SegmentType.R_SEG }),
	/** Path consists of a Left turn, then a Right turn, then a Left turn. */
	LRL(new SegmentType[] { SegmentType.L_SEG, SegmentType.R_SEG, SegmentType.L_SEG });

	private final SegmentType[] segmentTypes;

	/**
	 * Constructor for DubinsPathType.
	 * @param types The sequence of {@link SegmentType} that defines this path type.
	 */
	private DubinsPathType(SegmentType[] types) {
		this.segmentTypes = types;
	}

	/**
	 * Returns the sequence of {@link SegmentType} that defines this Dubins path type.
	 * This is primarily for internal use by the Dubins path generation logic.
	 *
	 * @return An array of {@link SegmentType} representing the path segments.
	 */
	SegmentType[] getValue() { // package-private getter
		return segmentTypes;
	}
}
