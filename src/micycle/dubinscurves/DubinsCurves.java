package micycle.dubinscurves;

/**
 * Static subroutines for Dubins curves computation.
 * <p>
 * The source code in this class is a very close port of the original C++ code,
 * preserving its "C-ish" idiosyncrasies; {@link micycle.dubinscurves.DubinsPath
 * DubinsPath} provides a object-orientated Java interface to this class, and
 * should be used instead.
 * 
 * @author Michael Carleton
 * @author Andrew Walker
 *
 */
class DubinsCurves {

	private static final int EDUBOK = 0; // No error
	private static final int EDUBCOCONFIGS = 1; // Colocated configurations
	private static final int EDUBPARAM = 2; // Path parameterisitation error
	private static final int EDUBBADRHO = 3; // the rho value is invalid
	private static final int EDUBNOPATH = 4; // no connection between configurations with this word
	private static final double EPSILON = (10e-10);

	/**
	 * Generate a path from an initial configuration to a target configuration, with
	 * a specified maximum turning radii.
	 * <p>
	 * A configuration is (<code>x, y, theta</code>), where theta is in radians,
	 * with zero along the line x = 0, and counter-clockwise is positive
	 *
	 * @segmentLengths path - the path object to initialise into
	 * @segmentLengths q0   - starting configuration, specified as an array of
	 *             <code>x, y, theta</code>
	 * @segmentLengths q1   - ending configuration, specified as an array of
	 *             <code>x, y, theta</code>
	 * @segmentLengths rho  - turning radius of the vehicle (forward velocity divided by
	 *             maximum angular velocity)
	 * @return non-zero on error
	 */
	static int dubins_shortest_path(DubinsPath path, double[] q0, double[] q1, double rho) {
		DubinsIntermediateResults in = new DubinsIntermediateResults();
		double[] params = new double[3];
		DubinsPathType best_word = null;
		int errcode = dubins_intermediate_results(in, q0, q1, rho);
		if (errcode != EDUBOK) {
			return errcode;
		}

		path.configStart[0] = q0[0];
		path.configStart[1] = q0[1];
		path.configStart[2] = q0[2];
		path.rho = rho;

		double best_cost = Double.MAX_VALUE;
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
					path.type = pathType;
				}
			}
		}
		if (best_word == null) {
			return EDUBNOPATH;
		}
		return EDUBOK;
	}

	/**
	 * Generate a path with a specified word from an initial configuration to a
	 * target configuration, with a specified turning radius.
	 *
	 * @segmentLengths path     - the path object to initialise into
	 * @segmentLengths q0       - starting configuration specified as an array of x, y, theta
	 * @segmentLengths q1       - ending configuration specified as an array of x, y, theta
	 * @segmentLengths rho      - turning radius of the vehicle (forward velocity divided by
	 *                 maximum angular velocity)
	 * @segmentLengths pathType - the specific path type to use
	 * @return non-zero on error
	 */
	static int dubins_path(DubinsPath path, double[] q0, double[] q1, double rho, DubinsPathType pathType) {
		int errcode;
		DubinsIntermediateResults in = new DubinsIntermediateResults();
		errcode = dubins_intermediate_results(in, q0, q1, rho);
		if (errcode == EDUBOK) {
			double[] params = new double[3];
			errcode = dubins_word(in, pathType, params);
			if (errcode == EDUBOK) {
				path.segmentLengths[0] = params[0];
				path.segmentLengths[1] = params[1];
				path.segmentLengths[2] = params[2];
				path.configStart[0] = q0[0];
				path.configStart[1] = q0[1];
				path.configStart[2] = q0[2];
				path.rho = rho;
				path.type = pathType;
			}
		}
		return errcode;
	}

	/**
	 * Calculate the length of an initialised path.
	 *
	 * @segmentLengths path - the path to find the length of
	 */
	static double dubins_path_length(DubinsPath path) {
		double length = 0.0;
		length += path.segmentLengths[0];
		length += path.segmentLengths[1];
		length += path.segmentLengths[2];
		length = length * path.rho;
		return length;
	}

	/**
	 * Return the length of a specific segment in an initialized path.
	 *
	 * @segmentLengths path - the path to find the length of
	 * @segmentLengths i    - the segment you to get the length of (0-2)
	 */
	static double dubins_segment_length(DubinsPath path, int i) {
		if ((i < 0) || (i > 2)) {
			return Double.MAX_VALUE;
		}
		return path.segmentLengths[i] * path.rho;
	}

	/**
	 * Return the normalized length of a specific segment in an initialized path.
	 *
	 * @segmentLengths path - the path to find the length of
	 * @segmentLengths i    - the segment you to get the length of (0-2)
	 */
	static double dubins_segment_length_normalized(DubinsPath path, int i) {
		if ((i < 0) || (i > 2)) {
			return Double.MAX_VALUE;
		}
		return path.segmentLengths[i];
	}

	/**
	 * Extract an integer that represents which path type was used.
	 *
	 * @segmentLengths path - an initialised path
	 * @return - one of LSL, LSR, RSL, RSR, RLR or LRL
	 */
	static DubinsPathType dubins_path_type(DubinsPath path) {
		return path.type;
	}

	/**
	 * Calculate the configuration along the path, using the parameter t.
	 *
	 * @segmentLengths path - an initialised path
	 * @segmentLengths t    - a length measure, where 0 <= t < dubins_path_length(path)
	 * @segmentLengths q    - the configuration result
	 * @returns - non-zero if 't' is not in the correct range
	 */
	static int dubins_path_sample(DubinsPath path, double t, double[] q) {
		/* tprime is the normalised variant of the parameter t */
		double tprime = t / path.rho;
		double[] qi = new double[3]; // The translated initial configuration
		double[] q1 = new double[3]; // end-of segment 1
		double[] q2 = new double[3]; // end-of segment 2
		SegmentType[] types = path.type.getValue();
		double p1;
		double p2;

		if (t < 0 || t > dubins_path_length(path)) {
			return EDUBPARAM;
		}

		/* initial configuration */
		qi[0] = 0.0;
		qi[1] = 0.0;
		qi[2] = path.configStart[2];

		/* generate the target configuration */
		p1 = path.segmentLengths[0];
		p2 = path.segmentLengths[1];
		dubins_segment(p1, qi, q1, types[0]);
		dubins_segment(p2, q1, q2, types[1]);
		if (tprime < p1) {
			dubins_segment(tprime, qi, q, types[0]);
		} else if (tprime < (p1 + p2)) {
			dubins_segment(tprime - p1, q1, q, types[1]);
		} else {
			dubins_segment(tprime - p1 - p2, q2, q, types[2]);
		}

		/*
		 * scale the target configuration, translate back to the original starting point
		 */
		q[0] = q[0] * path.rho + path.configStart[0];
		q[1] = q[1] * path.rho + path.configStart[1];
		q[2] = mod2pi(q[2]);

		return EDUBOK;
	}

	/**
	 * Walk along the path at a fixed sampling interval, calling the callback
	 * function at each interval.
	 *
	 * The sampling process continues until the whole path is sampled, or the
	 * callback returns a non-zero value
	 *
	 * @segmentLengths path      - the path to sample
	 * @segmentLengths stepSize  - the distance along the path for subsequent samples
	 * @segmentLengths cb        - the callback function to call for each sample
	 * @segmentLengths user_data - optional information to pass on to the callback
	 *
	 * @returns - zero on successful completion, or the result of the callback
	 */
	static int dubins_path_sample_many(DubinsPath path, double stepSize, DubinsPathSamplingCallback cb) {
		int retcode;
		double[] q = new double[3];
		double x = 0.0;
		double length = dubins_path_length(path);
		while (x < length) {
			dubins_path_sample(path, x, q);
			retcode = cb.invoke(q, x);
			if (retcode != 0) {
				return retcode;
			}
			x += stepSize;
		}
		return 0;
	}

	/**
	 * Convenience function to identify the endpoint of a path.
	 *
	 * @segmentLengths path - an initialised path
	 * @segmentLengths q    - the configuration result
	 */
	static int dubins_path_endpoint(DubinsPath path, double[] q) {
		return dubins_path_sample(path, dubins_path_length(path) - EPSILON, q);
	}

	/**
	 * Convenience function to extract a subset of a path.
	 *
	 * @segmentLengths path    - an initialised path
	 * @segmentLengths t       - a length measure, where 0 < t < dubins_path_length(path)
	 * @segmentLengths newpath - the resultant path
	 */
	static int dubins_extract_subpath(DubinsPath path, double t, DubinsPath newpath) {
		/* calculate the true parameter */
		double tprime = t / path.rho;

		if ((t < 0) || (t > dubins_path_length(path))) {
			return EDUBPARAM;
		}

		/* copy most of the data */
		newpath.configStart[0] = path.configStart[0];
		newpath.configStart[1] = path.configStart[1];
		newpath.configStart[2] = path.configStart[2];
		newpath.rho = path.rho;
		newpath.type = path.type;

		/* fix the parameters */
		newpath.segmentLengths[0] = Math.min(path.segmentLengths[0], tprime);
		newpath.segmentLengths[1] = Math.min(path.segmentLengths[1], tprime - newpath.segmentLengths[0]);
		newpath.segmentLengths[2] = Math.min(path.segmentLengths[2], tprime - newpath.segmentLengths[0] - newpath.segmentLengths[1]);
		return 0;
	}

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
			default :
				result = EDUBNOPATH;
		}
		return result;
	}

	private static int dubins_intermediate_results(DubinsIntermediateResults in, double[] q0, double[] q1, double rho) {
		double dx;
		double dy;
		double D;
		double d;
		double theta;
		double alpha;
		double beta;
		if (rho <= 0.0) {
			return EDUBBADRHO;
		}

		dx = q1[0] - q0[0];
		dy = q1[1] - q0[1];
		D = Math.sqrt(dx * dx + dy * dy);
		d = D / rho;
		theta = 0;

		/* test required to prevent domain errors if dx=0 and dy=0 */
		if (d > 0) {
			theta = mod2pi(Math.atan2(dy, dx));
		}
		alpha = mod2pi(q0[2] - theta);
		beta = mod2pi(q1[2] - theta);

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

	private static void dubins_segment(double t, double[] qi, double[] qt, SegmentType type) {
		double st = Math.sin(qi[2]);
		double ct = Math.cos(qi[2]);
		if (type == SegmentType.L_SEG) {
			qt[0] = +Math.sin(qi[2] + t) - st;
			qt[1] = -Math.cos(qi[2] + t) + ct;
			qt[2] = t;
		} else if (type == SegmentType.R_SEG) {
			qt[0] = -Math.sin(qi[2] - t) + st;
			qt[1] = +Math.cos(qi[2] - t) - ct;
			qt[2] = -t;
		} else if (type == SegmentType.S_SEG) {
			qt[0] = ct * t;
			qt[1] = st * t;
			qt[2] = 0.0;
		}
		qt[0] += qi[0];
		qt[1] += qi[1];
		qt[2] += qi[2];
	}

	private static int dubins_LSL(DubinsIntermediateResults in, double[] out) {
		double tmp0;
		double tmp1;
		double p_sq;

		tmp0 = in.d + in.sa - in.sb;
		p_sq = 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sa - in.sb));

		if (p_sq >= 0) {
			tmp1 = Math.atan2((in.cb - in.ca), tmp0);
			out[0] = mod2pi(tmp1 - in.alpha);
			out[1] = Math.sqrt(p_sq);
			out[2] = mod2pi(in.beta - tmp1);
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
			out[2] = mod2pi(tmp0 - mod2pi(in.beta));
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
		double tmp0 = (6.0 - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sa - in.sb)) / 8.0;
		double phi = Math.atan2(in.ca - in.cb, in.d - in.sa + in.sb);
		if (Math.abs(tmp0) <= 1) {
			double p = mod2pi((2 * Math.PI) - Math.acos(tmp0));
			double t = mod2pi(in.alpha - phi + mod2pi(p / 2.0));
			out[0] = t;
			out[1] = p;
			out[2] = mod2pi(in.alpha - in.beta - t + mod2pi(p));
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

	private static double floorMod(double x, double y) {
		return x - y * Math.floor(x / y);
	}

	private static double mod2pi(double theta) {
		return floorMod(theta, 2 * Math.PI);
	}

}