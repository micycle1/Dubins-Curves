package micycle.dubins;

enum SegmentType {
	L_SEG, S_SEG, R_SEG;
}

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