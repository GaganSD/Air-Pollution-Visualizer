/*
 * Written by Gagan Devagiri
 * for Informatics Large Practical coursework 2
 * Student number: s1854008
 */

package uk.ac.ed.inf.aqmaps;

import java.io.*;
import java.util.ArrayList;

import com.mapbox.geojson.*;

/**
 * @author s1854008
 *
 */
public class IOHandler {

	// input arguments ---
	// start positions of the drone
	Double latitude;
	Double longitude;

	// seed for random generators
	int seed;
	//port the web server's running in
	int port;
	// date to get the data from
	String day;
	String month;
	String year;

	// -------------------
	private DroneBackend connectedBackend;
	private ArrayList<Point> droneRoute = new ArrayList<Point>();
	// ------------------
	final static String FLIGHT_PATH_FILE_PREFIX = "flightpath-";
	final static String TXT_FILE_SUFFIX = ".txt";
	final static String GEOJSON_FILE_PREFIX = "readings-";
	final static String GEOJSON_FILE_SUFFIX = ".geojson";

	/*
	 * IOHandler Constructor
	 */
	public IOHandler() {
	}

	/* 
	 * @param drone BackEnd IOHandler Constructor with 
	 * the backend as the parameter.
	 * Overloaded constructor. 
	 */
	public IOHandler(DroneBackend connectedBackend) {
		this.ConnectDroneBackend(connectedBackend);
	}

	/**
	 * getter for connectedBackend instance variables
	 * @return the connectedBackend
	 */
	protected DroneBackend getDroneBackend() {
		return this.connectedBackend;
	}

	/**
	 * Setter to update the instance variable
	 * @param Connect drone's Backend with the handler.
	 */
	protected void ConnectDroneBackend(DroneBackend connectedBackend) {
		this.connectedBackend = connectedBackend;
		this.droneRoute = connectedBackend.getdroneRoute();
	}

	/**
	 * @param arguments given by the user. Makes checks for the argument and
	 *                  populates the variables.
	 */
	protected void parseInputs(String[] args) {
		
		var defaultPort = false;
		
		if (args.length < 6) {
			throw new IllegalArgumentException("Invalid argument!");
		} else if (args.length == 6) {
			// port isn't given. use default port of 80
			this.port = 80;
			defaultPort = true;

		}
		
		boolean invalid = false;
		// check date
		var day = Integer.parseInt(args[0]);

		if (day < 1 || day > 31) {
			invalid = true;
			System.out.println("Day should be between 1 and 31/30/28");
		} else if (day < 10 && args[0].length() == 1) {
			this.day = "0" + args[0];
		} else {
			this.day = args[0];
		}
		// check month
		var month = Integer.parseInt(args[1]);

		if (month < 1 || month > 12) {
			invalid = true;
		}
		if (month < 12 && args[1].length() == 1) {
			this.month = "0" + args[1];
		} else {
			this.month = args[1];
		}

		// check year
		var year = Integer.parseInt(args[2]);

		if (year != 2020 && year != 2021) {
			invalid = true;
		}
		this.year = args[2];

		// check location
		var latitude = Double.parseDouble(args[3]);
		var longitude = Double.parseDouble(args[4]);

		if (latitude <= -90 || longitude <= -180 || latitude >= 90 || longitude >= 180) {
			invalid = true;
			System.out.println("invalid starting location. \n "
					+ "Range is: latitude =< -90 || longitude =< -180 || latitude >= 90 || longitude >= 180");

		}

		// add location & seed
		this.latitude = latitude;
		this.longitude = longitude;
		this.seed = Integer.parseInt(args[5]);

		// add port
		if (!defaultPort) {
			port = Integer.parseInt(args[6]);
		}

		if (port < 0 || port > 65353) {
			invalid = true;
			System.out.println("Port should be between 0 and 65353");
		}
		if (invalid) {
		System.out.println("You've given an invalid input.\nPlease give "
				+ "an input in the correct order & make sure its semantically correct"
				+ "\nThe system will now exit.");
			System.exit(1);
		}

		return;
	}

	/**
	 * @return void
	 * Print's basic statistics of the flightpath to the command line.
	 */
	protected void printInterestingFacts() {

		System.out.println("***Success!***");
		System.out.println("The drone took these many steps: " + connectedBackend.getStepsTaken());

		if (connectedBackend.getSensorsNotVisitedCount() == 0) {
			System.out.println("The drone managed to visit all the sensors!");
		} else {
			System.out.println(String.format("There were 33 sensors and the drone couldn't visit %d sensors.",
					connectedBackend.getSensorsNotVisitedCount()));
		}

		return;
	}

	/**
	 * @param GeoJson or a feature collection Write a GeoJson to the system, returns
	 *                nothing.
	 */
	protected void writeGeoJSONFile() throws IOException {

		// follow format as given in the doc
		String filename = IOHandler.GEOJSON_FILE_PREFIX + this.day + "-" + this.month + "-" 
				+ this.year + IOHandler.GEOJSON_FILE_SUFFIX;
		FileWriter readings = new FileWriter(filename);

		// write to file
		readings.write(connectedBackend.getGeoJsonFeatures().toJson());
		readings.close();

		System.out.println(filename + " was written to the system. You can visualise it on www.geojson.io.");

	}

	/**
	 * Write the path that the drone has taken (simulated) in the text file, following the guidelines given
	 * @return void
	 */
	protected void writeTXTFile() throws IOException {

		String filename = IOHandler.FLIGHT_PATH_FILE_PREFIX + this.day + "-" 
				+ this.month + "-" + this.year + IOHandler.TXT_FILE_SUFFIX;

		FileWriter dronePath = new FileWriter(filename);
		try {
		var movesTaken = connectedBackend.getStepsTaken();
		var directions = connectedBackend.getDirections();

		for (int i = 0; i < movesTaken; i++) {
			
			var drCurrMove = i + 1;
			var drLng = droneRoute.get(i).longitude();
			var drLat = droneRoute.get(i).latitude();
			var drDir = directions.get(i);
			var drLngMoved = droneRoute.get(i + 1).longitude();
			var drLatMoved = droneRoute.get(i + 1).latitude();
			var drW3WLoc = connectedBackend.getlocationStrings().get(i);

			var currLine = drCurrMove + "," + drLng + "," + drLat + "," + drDir + "," + 
					drLngMoved + "," + drLatMoved + "," + drW3WLoc + "\n";

			dronePath.write(currLine);
		}
		dronePath.close();
		} catch (Exception e) {
			System.out.println("The app failed while writing the file.\n" + filename + " could not be written.");
			e.printStackTrace();

		}

		System.out.println(filename + " was written to the current path.");
	}

	/**
	 * @param GeoJson or a feature collection Write geojson & log to the system.
	 *                (current path) Returns nothing.
	 */
	protected void writeToSys() throws IOException, InterruptedException {

		try {

			writeGeoJSONFile();
			writeTXTFile();

		} catch (IOException e) {

			System.out.println("Oh noes! Something terrible has happened while writing the file.\n\n");
			e.printStackTrace();
		}

		return;
	}

	protected void runExperiments() {

	}
}
