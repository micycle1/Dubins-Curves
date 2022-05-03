package micycle.dubins;

enum DubinsPathType {
	
	LSL(0), LSR(1), RSL(2), RSR(3), RLR(4), LRL(5);

	public static final int SIZE = java.lang.Integer.SIZE;

	private int intValue;
	private static java.util.HashMap<Integer, DubinsPathType> mappings;

	private static java.util.HashMap<Integer, DubinsPathType> getMappings() {
		if (mappings == null) {
			synchronized (DubinsPathType.class) {
				if (mappings == null) {
					mappings = new java.util.HashMap<>();
				}
			}
		}
		return mappings;
	}

	private DubinsPathType(int value) {
		intValue = value;
		getMappings().put(value, this);
	}

	public int getValue() {
		return intValue;
	}

	public static DubinsPathType forValue(int value) {
		return getMappings().get(value);
	}
}