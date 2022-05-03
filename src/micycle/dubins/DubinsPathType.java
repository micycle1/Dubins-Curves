package micycle.dubins;

/**
 * Each segment motion primitive applies a constant action over an interval of
 * time.
 */
enum SegmentType {
	L_SEG, S_SEG, R_SEG;
}

/**
 * Describes each possible kind of shortest path.
 * <p>
 * Dubins cars have 3 controls: “turn left at maximum”, “turn right at maximum”,
 * and “go straight”. All the paths traced out by the Dubin’s car are
 * combinations of these three controls. Let’s name the controls: “turn left at
 * maximum” will be L, “turn right at maximum” will be R, and “go straight” will
 * be S. There are only 6 combinations of these controls that describe ALL the
 * shortest paths, and they are: RSR, LSL, RSL, LSR, RLR, and LRL.
 */
public enum DubinsPathType {

	LSL(new SegmentType[] { SegmentType.L_SEG, SegmentType.S_SEG, SegmentType.L_SEG }),
	LSR(new SegmentType[] { SegmentType.L_SEG, SegmentType.S_SEG, SegmentType.R_SEG }),
	RSL(new SegmentType[] { SegmentType.R_SEG, SegmentType.S_SEG, SegmentType.L_SEG }),
	RSR(new SegmentType[] { SegmentType.R_SEG, SegmentType.S_SEG, SegmentType.R_SEG }),
	RLR(new SegmentType[] { SegmentType.R_SEG, SegmentType.L_SEG, SegmentType.R_SEG }),
	LRL(new SegmentType[] { SegmentType.L_SEG, SegmentType.R_SEG, SegmentType.L_SEG });

	private SegmentType[] value;

	private DubinsPathType(SegmentType[] value) {
		this.value = value;
	}

	SegmentType[] getValue() {
		return value;
	}
}