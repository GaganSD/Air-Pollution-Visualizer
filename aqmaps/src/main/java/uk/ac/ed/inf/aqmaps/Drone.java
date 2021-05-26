package uk.ac.ed.inf.aqmaps;

import java.util.ArrayList;
import com.mapbox.geojson.Point;

/**
 * @author s1854008
 *
 */
public class Drone implements DroneTemplate {
	/*
	 * Set up the current point of the drone.
	 */

	// static variables. constraints set in the guidelines.
	public static final int MAX_MOVES = 150;
	public static final double DETECTOR_RANGE = 0.0002;

	// private instances with setters & getters provided at the bottom
	private double lng;
	private double lat;
	private ArrayList<SensorData> sensorList;
	private ArrayList<WordsData> w3w;
	private NoFlyZone noFlyZone;

	/**
	 * @param lng
	 * @param lat
	 * @param sensorsData
	 * @param wordsData
	 * @param noflyzone
	 */
	public Drone(double lng, double lat, ArrayList<SensorData> sensorsData, ArrayList<WordsData> wordsData,
			NoFlyZone noflyzone) {

		this.lng = lng;
		this.lat = lat;
		this.sensorList = sensorsData;
		this.w3w = wordsData;
		this.noFlyZone = noflyzone;

	}

	@Override
	public DroneTemplate build() {

		return new Drone(lng, lat, sensorList, w3w, noFlyZone);

	}

	/**
	 * Builder Design Pattern ref: https://en.wikipedia.org/wiki/Builder_pattern
	 */
	public static class Builder {

		private double lng;
		private double lat;
		private ArrayList<SensorData> sensorList;
		private ArrayList<WordsData> w3w;
		private NoFlyZone noFlyZone;

		public Builder() {
		}

		public Builder setLongitude(double longitude) {
			this.lng = longitude;
			return this;
		}

		public Builder setLatitude(double latitude) {
			this.lat = latitude;
			return this;
		}

		public Builder setSensorsData(ArrayList<SensorData> sensorData) {
			this.sensorList = sensorData;
			return this;
		}

		public Builder setW3WordsData(ArrayList<WordsData> wordsData) {
			this.w3w = wordsData;
			return this;
		}

		public Builder setNoFlyZones(NoFlyZone noflyzones) {
			this.noFlyZone = noflyzones;
			return this;
		}

		public Drone build() {
			return new Drone(lng, lat, sensorList, w3w, noFlyZone);
		}
	}

	// --------------------------
	// Getters

	@Override
	public Point getCurrentLocation() {
		return Point.fromLngLat(lng, lat);
	}

	@Override
	public ArrayList<SensorData> getSensors() {
		return this.sensorList;
	}

	@Override
	public ArrayList<WordsData> getW3Words() {
		return this.w3w;
	}

	@Override
	public NoFlyZone getForbiddenLocations() {
		return this.noFlyZone;
	}

	// --------------------------
	// Setters
	/**
	 * @param sensorsList
	 */
	@Override
	public void updateSensors(ArrayList<SensorData> sensorsList) {
		this.sensorList = sensorsList;
		return;
	}
	/*Setter for What 3 words
	 * @params List of What3Words
	 */
	@Override
	public void updateW3Words(ArrayList<WordsData> wordsList) {
		this.w3w = wordsList;
		return;
	}
	
	/**
	 * Setter for Forbidden Locations
	 * @param NoFlyZone forbiddenRegions
	 */
	@Override
	public void updateForbiddenLocations(NoFlyZone forbiddenRegions) {
		this.noFlyZone = forbiddenRegions;
		return;
	}
	
	/**
	 * Setter for current location
	 * @param Double longitude, Double latitude
	 * this should be used with drone's current location
	 */
	@Override
	public void updateCurrentLocation(Double longitude, Double latitude) {
		this.lng = longitude;
		this.lat = latitude;
		return;
	}

	/**
	 * Setter for current location
	 * @param Point currentLocation
	 * Overloaded method - this should be used with drone's current location
	 */
	public void updateCurrentLocation(Point currentLocation) {
		this.lng = currentLocation.longitude();
		this.lat = currentLocation.latitude();
		return;
	}

}
