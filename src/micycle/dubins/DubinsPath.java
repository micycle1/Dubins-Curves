package micycle.dubins;

public class DubinsPath {
	
	public DubinsPath() {
 }
	
	/** the initial configuration */
	public double[] qi = new double[3];
	/** the lengths of the three segments */
	public double[] param = new double[3];
	/** model forward velocity / model angular velocity */
	public double rho;
	/** the path type described */
	public DubinsPathType type;
}