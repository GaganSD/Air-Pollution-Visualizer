/**
 * 
 */
package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;
import com.mapbox.geojson.Point;

public interface DroneTemplate {

	DroneTemplate build();

	// -------
	// Getters
	Point getCurrentLocation();

	ArrayList<SensorData> getSensors();

	ArrayList<WordsData> getW3Words();

	NoFlyZone getForbiddenLocations();

	// ------
	// Setters
	void updateSensors(ArrayList<SensorData> sensorsList);

	void updateW3Words(ArrayList<WordsData> wordsList);

	void updateForbiddenLocations(NoFlyZone forbiddenRegions);

	void updateCurrentLocation(Double longitude, Double latitude);

}