/**
 * 
 */
package uk.ac.ed.inf.aqmaps;

import java.io.IOException;


public interface DroneBackendTemplate {
	
	public void generateRoute(String algorithm) throws IOException, InterruptedException;
}