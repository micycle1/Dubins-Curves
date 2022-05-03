package micycle.dubins;

/**
 * Callback function for path sampling
 *
 * @note the q parameter is a configuration
 * @note the t parameter is the distance along the path
 * @note return non-zero to denote sampling should be stopped
 */
@FunctionalInterface
public interface DubinsPathSamplingCallback {
	int invoke(double[] q, double t);
}