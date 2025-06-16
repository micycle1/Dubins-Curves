package micycle.dubinscurves;

/**
 * Static subroutines for Dubins curves computation.
 * <p>
 * The source code in this class is a very close port of the original C++ code by
 * Andrew Walker, preserving its "C-ish" idiosyncrasies and structure.
 * {@link micycle.dubinscurves.DubinsPath DubinsPath} provides a more idiomatic
 * object-oriented Java interface to this core logic and should generally be used instead.
 * <p>
 * This class is package-private and not intended for direct use by external clients.
 * Comments within this class are primarily for maintainability.
 * 
 * @author Michael Carleton
 * @author Andrew Walker (Original C++ implementation)
 */
class DubinsCurves {

	/** No error. */
	private static final int EDUBOK = 0;
	/** Configurations are co-located (start and end points are the same or very close). */
	private static final int EDUBCOCONFIGS = 1;
	/** Path parameterization error (e.g., 't' value out of bounds during sampling). */
	private static final int EDUBPARAM = 2;
	/** Turning radius 'rho' is invalid (e.g., zero or negative). */
	private static final int EDUBBADRHO = 3;
	/** No valid Dubins path found for the given configurations and word (path type). */
	private static final int EDUBNOPATH = 4;

	/** A small epsilon value for floating-point comparisons. */
	private static final double EPSILON = 10e-10; // Note: previous value was (10e-10) which is 1e-9. Standardizing.

	/**
	 * Calculates the shortest Dubins path between two configurations.
	 * Populates the provided {@link DubinsPath} object with the path details.
	 *
	 * @param path The {@link DubinsPath} object to be populated with the shortest path found.
	 *             Its internal fields (configStart, rho, segmentLengths, type) will be set.
	 * @param q0   The starting configuration <code>[x, y, theta]</code>.
	 * @param q1   The ending configuration <code>[x, y, theta]</code>.
	 * @param rho  The turning radius. Must be positive.
	 * @return {@link #EDUBOK} on success, or an error code (e.g., {@link #EDUBBADRHO}, {@link #EDUBNOPATH}) on failure.
	 */
	static int dubins_shortest_path(DubinsPath path, double[] q0, double[] q1, double rho) {
		DubinsIntermediateResults in = new DubinsIntermediateResults();
		double[] params = new double[3];
		DubinsPathType best_word = null;
		int errcode = dubins_intermediate_results(in, q0, q1, rho);
		if (errcode != EDUBOK) {
			// If intermediate results failed (e.g. bad rho), pass the error up
			if (errcode == EDUBBADRHO) path.type = null; // Indicate no path
			return errcode;
		}

		path.configStart[0] = q0[0];
		path.configStart[1] = q0[1];
		path.configStart[2] = q0[2];
		path.rho = rho;

		double best_cost = Double.MAX_VALUE;
		// Iterate over all possible path types to find the shortest one
		for (DubinsPathType pathType : DubinsPathType.values()) {
			errcode = dubins_word(in, pathType, params);
			if (errcode == EDUBOK) {
				double cost = params[0] + params[1] + params[2];
				if (cost < best_cost) {
					best_word = pathType;
					best_cost = cost;
					path.segmentLengths[0] = params[0];
					path.segmentLengths[1] = params[1];
					path.segmentLengths[2] = params[2];
					path.type = pathType; // Set the type of the shortest path found
				}
			}
		}
		if (best_word == null) {
			path.type = null; // Indicate no path found
			return EDUBNOPATH; // No path found among all types
		}
		return EDUBOK; // Success
	}

	/**
	 * Calculates a Dubins path of a specific {@link DubinsPathType} between two configurations.
	 * Populates the provided {@link DubinsPath} object with the path details.
	 * This path is not necessarily the shortest.
	 *
	 * @param path     The {@link DubinsPath} object to be populated.
	 *                 Its internal fields will be set according to the specified pathType.
	 * @param q0       The starting configuration <code>[x, y, theta]</code>.
	 * @param q1       The ending configuration <code>[x, y, theta]</code>.
	 * @param rho      The turning radius. Must be positive.
	 * @param pathType The specific {@link DubinsPathType} to generate.
	 * @return {@link #EDUBOK} on success, or an error code (e.g., {@link #EDUBBADRHO}, {@link #EDUBNOPATH}) if the specified pathType is not valid for the given configurations.
	 */
	static int dubins_path(DubinsPath path, double[] q0, double[] q1, double rho, DubinsPathType pathType) {
		int errcode;
		DubinsIntermediateResults in = new DubinsIntermediateResults();
		errcode = dubins_intermediate_results(in, q0, q1, rho); // Pre-calculate intermediate values
		if (errcode == EDUBOK) {
			double[] params = new double[3];
			errcode = dubins_word(in, pathType, params); // Calculate segment lengths for the given pathType
			if (errcode == EDUBOK) {
				// Populate the DubinsPath object
				path.segmentLengths[0] = params[0];
				path.segmentLengths[1] = params[1];
				path.segmentLengths[2] = params[2];
				path.configStart[0] = q0[0];
				path.configStart[1] = q0[1];
				path.configStart[2] = q0[2];
				path.rho = rho;
				path.type = pathType;
			} else {
				path.type = null; // Indicate no path of this type
			}
		} else {
		    path.type = null; // Indicate no path due to bad rho or other initial error
        }
		return errcode;
	}

	/**
	 * Calculates the total length of an initialized Dubins path.
	 *
	 * @param path The {@link DubinsPath} object, which must have been previously initialized.
	 * @return The total length of the path (sum of segment lengths scaled by rho). Returns 0 if path type is null.
	 */
	static double dubins_path_length(DubinsPath path) {
		if (path.type == null) return 0.0;
		double length = 0.0;
		length += path.segmentLengths[0];
		length += path.segmentLengths[1];
		length += path.segmentLengths[2];
		length = length * path.rho;
		return length;
	}

	/**
	 * Returns the length of a specific segment of an initialized Dubins path.
	 *
	 * @param path The {@link DubinsPath} object.
	 * @param i    The segment index (0, 1, or 2).
	 * @return The length of the specified segment (segmentLength * rho).
	 *         Returns {@link Double#MAX_VALUE} if the segment index <code>i</code> is invalid.
	 */
	static double dubins_segment_length(DubinsPath path, int i) {
		if (path.type == null || (i < 0) || (i > 2)) {
			return Double.MAX_VALUE; // Or handle error appropriately
		}
		return path.segmentLengths[i] * path.rho;
	}

	/**
	 * Returns the normalized length of a specific segment of an initialized Dubins path.
	 * The normalized length is the length before scaling by the turning radius <code>rho</code>.
	 *
	 * @param path The {@link DubinsPath} object.
	 * @param i    The segment index (0, 1, or 2).
	 * @return The normalized length of the specified segment (<code>path.segmentLengths[i]</code>).
	 *         Returns {@link Double#MAX_VALUE} if the segment index <code>i</code> is invalid.
	 */
	static double dubins_segment_length_normalized(DubinsPath path, int i) {
		if (path.type == null || (i < 0) || (i > 2)) {
			return Double.MAX_VALUE; // Or handle error appropriately
		}
		return path.segmentLengths[i];
	}

	/**
	 * Returns the {@link DubinsPathType} of an initialized Dubins path.
	 *
	 * @param path The {@link DubinsPath} object.
	 * @return The {@link DubinsPathType} of the path. May be null if path initialization failed.
	 */
	static DubinsPathType dubins_path_type(DubinsPath path) {
		return path.type;
	}

	/**
	 * Samples a configuration <code>[x, y, theta]</code> at a specific distance <code>t</code> along an initialized Dubins path.
	 *
	 * @param path The {@link DubinsPath} object.
	 * @param t    The distance along the path to sample. Should be in the range <code>[0, dubins_path_length(path)]</code>.
	 * @param q    A <code>double[3]</code> array to be populated with the sampled configuration <code>[x, y, theta]</code>.
	 * @return {@link #EDUBOK} on success, or {@link #EDUBPARAM} if <code>t</code> is out of valid range (less than 0 or greater than path length).
	 *         Returns {@link #EDUBNOPATH} if path.type is null.
	 */
	static int dubins_path_sample(DubinsPath path, double t, double[] q) {
		if (path.type == null) {
		    if (q.length >=3) { q[0]=path.configStart[0]; q[1]=path.configStart[1]; q[2]=path.configStart[2]; }
		    return EDUBNOPATH; // No path to sample
        }
		/* tprime is the normalised variant of the parameter t */
		double tprime = t / path.rho;
		double pathLength = dubins_path_length(path);

		// Original C code checks t < 0 or t > pathLength.
		// Allowing t == pathLength for sampling endpoint.
		// Clamping t to be within [0, pathLength]
		if (t < -EPSILON || t > pathLength + EPSILON) {
			return EDUBPARAM;
		}
        t = Math.max(0, Math.min(t, pathLength)); // Clamp t to valid range
        tprime = t / path.rho;


		double[] qi = new double[3]; // The translated initial configuration (normalized coordinates)
		double[] q1 = new double[3]; // End-of segment 1 (normalized)
		double[] q2 = new double[3]; // End-of segment 2 (normalized)
		SegmentType[] types = path.type.getValue(); // Segment types (L, S, R)
		double p1, p2;

		/* initial configuration, in normalized space (assuming start at origin, aligned with x-axis) */
		qi[0] = 0.0;
		qi[1] = 0.0;
		qi[2] = path.configStart[2]; // Original start orientation

		/* generate the target configuration by stepping through segments */
		p1 = path.segmentLengths[0]; // Normalized length of segment 1
		p2 = path.segmentLengths[1]; // Normalized length of segment 2

		dubins_segment(p1, qi, q1, types[0]); // Calculate end of segment 1
		dubins_segment(p2, q1, q2, types[1]); // Calculate end of segment 2

		if (tprime < p1) {
			dubins_segment(tprime, qi, q, types[0]);
		} else if (tprime < (p1 + p2)) {
			dubins_segment(tprime - p1, q1, q, types[1]);
		} else {
			dubins_segment(tprime - p1 - p2, q2, q, types[2]);
		}

		/* Scale and translate the configuration back to the original coordinate frame */
		q[0] = q[0] * path.rho + path.configStart[0];
		q[1] = q[1] * path.rho + path.configStart[1];
		q[2] = mod2pi(q[2]); // Normalize final orientation

		return EDUBOK;
	}

	/**
	 * Samples configurations at regular step intervals along an initialized Dubins path,
	 * invoking a callback for each sample.
	 *
	 * @param path     The {@link DubinsPath} object.
	 * @param stepSize The distance between samples along the path. Must be positive.
	 * @param cb       The {@link DubinsPathSamplingCallback} to invoke for each sample.
	 *                 The callback receives the sampled configuration <code>q</code> and the distance <code>t</code>.
	 *                 If the callback returns a non-zero value, sampling is halted.
	 * @return 0 if sampling completes successfully, or the non-zero value returned by the callback if sampling was halted.
	 *         Returns {@link #EDUBNOPATH} if path.type is null.
	 */
	static int dubins_path_sample_many(DubinsPath path, double stepSize, DubinsPathSamplingCallback cb) {
		if (path.type == null) return EDUBNOPATH;
		if (stepSize <= 0) return EDUBPARAM;

		int retcode;
		double[] q = new double[3];
		double x = 0.0;
		double length = dubins_path_length(path);
		while (x <= length) { // Use <= to include endpoint if it falls on a step
			dubins_path_sample(path, x, q);
			retcode = cb.invoke(q, x);
			if (retcode != 0) {
				return retcode; // Callback requested halt
			}
			x += stepSize;
			if (x > length && (x - stepSize) < length - EPSILON) { // ensure last point is sampled if missed by step
                dubins_path_sample(path, length, q);
                retcode = cb.invoke(q, length);
                if (retcode != 0) return retcode;
            }
		}
		return 0; // Success
	}

	/**
	 * Calculates the endpoint configuration of an initialized Dubins path.
	 *
	 * @param path The {@link DubinsPath} object.
	 * @param q    A <code>double[3]</code> array to be populated with the endpoint configuration <code>[x, y, theta]</code>.
	 * @return {@link #EDUBOK} on success, {@link #EDUBNOPATH} if path.type is null.
	 *         Internally calls {@link #dubins_path_sample(DubinsPath, double, double[])}.
	 */
	static int dubins_path_endpoint(DubinsPath path, double[] q) {
		if (path.type == null) {
		    if (q.length >=3) { q[0]=path.configStart[0]; q[1]=path.configStart[1]; q[2]=path.configStart[2]; }
		    return EDUBNOPATH;
        }
		// Sample at path_length. Using path_length directly, dubins_path_sample handles clamping.
		return dubins_path_sample(path, dubins_path_length(path), q);
	}

	/**
	 * Extracts a sub-path from the beginning of an initialized Dubins path up to a specified length <code>t</code>.
	 * The new path starts at the original path's start configuration.
	 *
	 * @param path    The original {@link DubinsPath} object.
	 * @param t       The desired length of the sub-path. Should be in <code>[0, dubins_path_length(path)]</code>.
	 * @param newpath A {@link DubinsPath} object to be populated with the extracted sub-path details.
	 * @return {@link #EDUBOK} on success, or {@link #EDUBPARAM} if <code>t</code> is out of valid range.
	 *         Returns {@link #EDUBNOPATH} if path.type is null.
	 */
	static int dubins_extract_subpath(DubinsPath path, double t, DubinsPath newpath) {
		if (path.type == null) {
		    newpath.type = null;
		    newpath.rho = path.rho;
            newpath.configStart[0] = path.configStart[0];
            newpath.configStart[1] = path.configStart[1];
            newpath.configStart[2] = path.configStart[2];
            newpath.segmentLengths[0] = 0; newpath.segmentLengths[1] = 0; newpath.segmentLengths[2] = 0;
		    return EDUBNOPATH;
        }
		/* calculate the true parameter */
		double tprime = t / path.rho;
		double pathLength = dubins_path_length(path);

		if (t < -EPSILON || t > pathLength + EPSILON) {
			return EDUBPARAM;
		}
        t = Math.max(0, Math.min(t, pathLength)); // Clamp t
        tprime = t / path.rho;


		/* copy most of the data */
		newpath.configStart[0] = path.configStart[0];
		newpath.configStart[1] = path.configStart[1];
		newpath.configStart[2] = path.configStart[2];
		newpath.rho = path.rho;
		newpath.type = path.type; // Subpath is of the same fundamental type

		/* Fix the segment lengths for the new path */
		newpath.segmentLengths[0] = Math.min(path.segmentLengths[0], tprime);
		newpath.segmentLengths[1] = Math.min(path.segmentLengths[1], tprime - newpath.segmentLengths[0]);
		// Ensure segment length isn't negative if tprime is smaller than previous segments
		newpath.segmentLengths[1] = Math.max(0, newpath.segmentLengths[1]);
		newpath.segmentLengths[2] = Math.min(path.segmentLengths[2], tprime - newpath.segmentLengths[0] - newpath.segmentLengths[1]);
		newpath.segmentLengths[2] = Math.max(0, newpath.segmentLengths[2]);

		return EDUBOK;
	}

	/**
	 * Pre-calculates intermediate values used in Dubins path computations.
	 * These values depend on the start and end configurations and the turning radius.
	 *
	 * @param in The {@link DubinsIntermediateResults} object to be populated.
	 * @param q0 The starting configuration <code>[x, y, theta]</code>.
	 * @param q1 The ending configuration <code>[x, y, theta]</code>.
	 * @param rho The turning radius. Must be positive.
	 * @return {@link #EDUBOK} on success, or {@link #EDUBBADRHO} if rho is invalid,
	 *         or {@link #EDUBCOCONFIGS} if configurations are co-located (though original C code doesn't return this here).
	 */
	private static int dubins_intermediate_results(DubinsIntermediateResults in, double[] q0, double[] q1, double rho) {
		if (rho <= 0.0) {
			return EDUBBADRHO;
		}

		double dx = q1[0] - q0[0];
		double dy = q1[1] - q0[1];
		double D = Math.sqrt(dx * dx + dy * dy); // Distance between start and end points
		double d = D / rho; // Normalized distance

		// Check for co-located configurations (distance is near zero)
        // The original C code doesn't explicitly return EDUBCOCONFIGS here, but it's a potential condition.
        // For now, matching original logic which might proceed and find a zero-length path.
        // if (d < EPSILON) { return EDUBCOCONFIGS; }


		double theta = 0; // Angle of the line connecting start and end points

		/* test required to prevent domain errors if dx=0 and dy=0 */
		if (d > EPSILON) { // Avoid atan2(0,0)
			theta = mod2pi(Math.atan2(dy, dx));
		}
		double alpha = mod2pi(q0[2] - theta); // Angle between start orientation and line connecting points
		double beta = mod2pi(q1[2] - theta);  // Angle between end orientation and line connecting points

		in.alpha = alpha;
		in.beta = beta;
		in.d = d;
		in.sa = Math.sin(alpha);
		in.sb = Math.sin(beta);
		in.ca = Math.cos(alpha);
		in.cb = Math.cos(beta);
		in.c_ab = Math.cos(alpha - beta);
		in.d_sq = d * d;

		return EDUBOK;
	}

	/**
	 * Calculates the normalized segment lengths (t, p, q) for a specific Dubins path type (word),
	 * using pre-calculated intermediate results.
	 *
	 * @param in       The {@link DubinsIntermediateResults} containing pre-calculated values.
	 * @param pathType The {@link DubinsPathType} (word, e.g., LSL, RSR) for which to calculate segment lengths.
	 * @param out      A <code>double[3]</code> array to be populated with the normalized segment lengths <code>[t, p, q]</code>.
	 * @return {@link #EDUBOK} if a valid path of the given type exists, otherwise {@link #EDUBNOPATH}.
	 */
	private static int dubins_word(DubinsIntermediateResults in, DubinsPathType pathType, double[] out) {
		int result;
		switch (pathType) {
			case LSL :
				result = dubins_LSL(in, out);
				break;
			case RSL :
				result = dubins_RSL(in, out);
				break;
			case LSR :
				result = dubins_LSR(in, out);
				break;
			case RSR :
				result = dubins_RSR(in, out);
				break;
			case LRL :
				result = dubins_LRL(in, out);
				break;
			case RLR :
				result = dubins_RLR(in, out);
				break;
			default : // Should not happen if DubinsPathType is exhaustive
				result = EDUBNOPATH;
		}
		return result;
	}

	/**
	 * Calculates the configuration <code>qt</code> resulting from traversing a single segment of length <code>t</code>
	 * starting from configuration <code>qi</code> with a given {@link SegmentType}.
	 * This operates in a normalized space where rho=1 and the segment starts at (qi[0], qi[1]) with orientation qi[2].
	 *
	 * @param t    The normalized length of the segment to traverse.
	 * @param qi   The starting normalized configuration <code>[x, y, theta]</code> for this segment.
	 * @param qt   A <code>double[3]</code> array to be populated with the resulting normalized configuration <code>[x, y, theta]</code> after traversing the segment.
	 * @param type The {@link SegmentType} (L_SEG, R_SEG, S_SEG) of the segment.
	 */
	private static void dubins_segment(double t, double[] qi, double[] qt, SegmentType type) {
		double sin_qi2 = Math.sin(qi[2]);
		double cos_qi2 = Math.cos(qi[2]);
		if (type == SegmentType.L_SEG) { // Left turn
			qt[0] = +Math.sin(qi[2] + t) - sin_qi2; // delta_x = sin(theta_start + t) - sin(theta_start)
			qt[1] = -Math.cos(qi[2] + t) + cos_qi2; // delta_y = -cos(theta_start + t) + cos(theta_start)
			qt[2] = t;                             // delta_theta = t
		} else if (type == SegmentType.R_SEG) { // Right turn
			qt[0] = -Math.sin(qi[2] - t) + sin_qi2; // delta_x = -sin(theta_start - t) + sin(theta_start)
			qt[1] = +Math.cos(qi[2] - t) - cos_qi2; // delta_y = cos(theta_start - t) - cos(theta_start)
			qt[2] = -t;                            // delta_theta = -t
		} else if (type == SegmentType.S_SEG) { // Straight segment
			qt[0] = cos_qi2 * t;                   // delta_x = cos(theta_start) * t
			qt[1] = sin_qi2 * t;                   // delta_y = sin(theta_start) * t
			qt[2] = 0.0;                           // delta_theta = 0
		}
		// Add deltas to initial qi to get final qt for the segment
		qt[0] += qi[0];
		qt[1] += qi[1];
		qt[2] += qi[2];
		// No mod2pi here, it's done at the end of dubins_path_sample
	}

	// Path type specific calculations (LSL, RSR, etc.)
	// These methods calculate the three normalized segment lengths (out[0], out[1], out[2])
	// for their respective path types, using the precomputed values in 'in'.
	// They return EDUBOK if a valid path of that type exists, otherwise EDUBNOPATH.

	private static int dubins_LSL(DubinsIntermediateResults in, double[] out) {
		double tmp0;
		double tmp1;
		double p_sq;

		tmp0 = in.d + in.sa - in.sb; // Denominator for atan2
		p_sq = 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sa - in.sb)); // Square of length of S segment

		if (p_sq >= 0) { // If p_sq is negative, sqrt is imaginary, so no such path
			tmp1 = Math.atan2((in.cb - in.ca), tmp0); // Angle related to S segment
			out[0] = mod2pi(tmp1 - in.alpha); // Length of first L segment
			out[1] = Math.sqrt(p_sq);         // Length of S segment
			out[2] = mod2pi(in.beta - tmp1);  // Length of second L segment
			return EDUBOK;
		}
		return EDUBNOPATH;
	}

	private static int dubins_RSR(DubinsIntermediateResults in, double[] out) {
		double tmp0 = in.d - in.sa + in.sb;
		double p_sq = 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sb - in.sa));
		if (p_sq >= 0) {
			double tmp1 = Math.atan2((in.ca - in.cb), tmp0);
			out[0] = mod2pi(in.alpha - tmp1);
			out[1] = Math.sqrt(p_sq);
			out[2] = mod2pi(tmp1 - in.beta);
			return EDUBOK;
		}
		return EDUBNOPATH;
	}

	private static int dubins_LSR(DubinsIntermediateResults in, double[] out) {
		double p_sq = -2 + (in.d_sq) + (2 * in.c_ab) + (2 * in.d * (in.sa + in.sb));
		if (p_sq >= 0) {
			double p = Math.sqrt(p_sq);
			double tmp0 = Math.atan2((-in.ca - in.cb), (in.d + in.sa + in.sb)) - Math.atan2(-2.0, p);
			out[0] = mod2pi(tmp0 - in.alpha);
			out[1] = p;
			out[2] = mod2pi(tmp0 - mod2pi(in.beta)); // Original C code uses mod2pi(beta) here.
			return EDUBOK;
		}
		return EDUBNOPATH;
	}

	private static int dubins_RSL(DubinsIntermediateResults in, double[] out) {
		double p_sq = -2 + in.d_sq + (2 * in.c_ab) - (2 * in.d * (in.sa + in.sb));
		if (p_sq >= 0) {
			double p = Math.sqrt(p_sq);
			double tmp0 = Math.atan2((in.ca + in.cb), (in.d - in.sa - in.sb)) - Math.atan2(2.0, p);
			out[0] = mod2pi(in.alpha - tmp0);
			out[1] = p;
			out[2] = mod2pi(in.beta - tmp0);
			return EDUBOK;
		}
		return EDUBNOPATH;
	}

	private static int dubins_RLR(DubinsIntermediateResults in, double[] out) {
		double tmp0 = (6.0 - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sa - in.sb)) / 8.0; // cos(phi - alpha) / 2
		double phi = Math.atan2(in.ca - in.cb, in.d - in.sa + in.sb); // Angle related to the geometry
		if (Math.abs(tmp0) <= 1) { // If tmp0 is a valid cosine value
			double p = mod2pi((2 * Math.PI) - Math.acos(tmp0)); // Length of the middle R segment (angle)
			double t = mod2pi(in.alpha - phi + mod2pi(p / 2.0)); // Length of the first R segment
			out[0] = t;
			out[1] = p;
			out[2] = mod2pi(in.alpha - in.beta - t + mod2pi(p)); // Length of the third R segment
			return EDUBOK;
		}
		return EDUBNOPATH;
	}

	private static int dubins_LRL(DubinsIntermediateResults in, double[] out) {
		double tmp0 = (6.0 - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sb - in.sa)) / 8.0;
		double phi = Math.atan2(in.ca - in.cb, in.d + in.sa - in.sb);
		if (Math.abs(tmp0) <= 1) {
			double p = mod2pi(2 * Math.PI - Math.acos(tmp0));
			double t = mod2pi(-in.alpha - phi + p / 2.0);
			out[0] = t;
			out[1] = p;
			out[2] = mod2pi(mod2pi(in.beta) - in.alpha - t + mod2pi(p));
			return EDUBOK;
		}
		return EDUBNOPATH;
	}

	/**
	 * Computes a modulo operation that handles negative numbers correctly for angles,
	 * ensuring the result is always in [0, y).
	 * Equivalent to Python's <code>x % y</code> where y is positive.
	 * @param x The dividend.
	 * @param y The divisor (must be positive).
	 * @return The value of x modulo y, in the range [0, y).
	 */
	private static double floorMod(double x, double y) {
		double r = x % y;
		if (r < 0) {
			r += y;
		}
		return r;
	}

	/**
	 * Normalizes an angle in radians to the range <code>[0, 2*PI)</code>.
	 * @param theta The angle in radians.
	 * @return The equivalent angle in the range <code>[0, 2*PI)</code>.
	 */
	private static double mod2pi(double theta) {
		return floorMod(theta, 2 * Math.PI);
	}

}