package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Point;

public class SensorData {
	private String location;
	private double battery;
	private String reading;
	private Point where;
	
	public SensorData(String location, double battery, String reading) {
		location = this.location;
		battery = this.battery;
		reading = this.reading;
	}
	public String getLocation() {
		return location;
	}
	public double getBattery() {
		return battery;
	}
	public String getReading() {
		return reading;
	}
	public void setWhere(Point where) {
		this.where = where;
	}
	public Point getWhere() {
		return where;
	}
}
