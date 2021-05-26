/*
 * Written by Gagan Devagiri
 * for Informatics Large Practical coursework 2
 * Student number: s1854008
 */

package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;

import com.mapbox.geojson.Feature;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.GeoJson;
import com.mapbox.geojson.Geometry;
import com.mapbox.geojson.LineString;
import com.mapbox.geojson.Point;

import java.util.*;
import org.jgrapht.alg.tour.*;
import org.jgrapht.graph.*;

public class DroneBackend implements DroneBackendTemplate {

	public static double DIST_DRONE_CAN_MOVE = 0.0003;

	// private instance variables with getters and setters
	// provided at the bottom
	private Drone currDrone;
	private Point startPositions;
	private ArrayList<SensorData> sensorsList;
	private int seed;
	private final String RGB = "rgb-string";
	private ArrayList<SensorData> orgSensorsList;
	private int nodeCount;

	private ArrayList<SensorData> shortestPathSensors = new ArrayList<SensorData>();
	private ArrayList<WordsData> wordsDataList;
	private ArrayList<Integer> directions = new ArrayList<>();
	private ArrayList<String> locationStrings = new ArrayList<String>();
	private ArrayList<Point> droneRoute;

	private NoFlyZone noFlyZone;
	private int stepsCompleted = 0;

	private GeoJson geoJsonFeatures;
	private int sensorsNotVisitedCount = 0;

	private boolean visited = false;

	public DroneBackend(Drone drone, Point startPositions, ArrayList<WordsData> wordsDataList,
			ArrayList<SensorData> sensorsList, NoFlyZone noFlyZone, int seed, ArrayList<SensorData> orgSensorsList) {

		this.startPositions = startPositions;
		this.wordsDataList = wordsDataList;
		this.sensorsList = sensorsList;
		this.noFlyZone = noFlyZone;
		this.currDrone = drone;
		this.seed = seed;
		this.orgSensorsList = orgSensorsList;

		// populate the sensors location with the w3words given to the drone backend.
		for (int i = 0; i < currDrone.getSensors().size(); i++) {
			var correspondingWord = currDrone.getW3Words().get(i);
			var sensorLocation = correspondingWord.getCoordinates().getPoint();
			// in-place

			currDrone.getSensors().get(i).setWhere(sensorLocation);
		}
	}

	public static class Builder {

		private Point startPositions;
		private ArrayList<WordsData> wordsDataList;
		private ArrayList<SensorData> sensorsList;
		private ArrayList<SensorData> orgSensorsList;
		private NoFlyZone noFlyZone;
		private Drone currDrone;
		private int seed;

		public Builder() {
		}

		public Builder connectDrone(Drone droneX) {
			this.currDrone = droneX;
			return this;
		}

		public Builder setStartPosition(Double startlng, Double startlat) {
			this.startPositions = Point.fromLngLat(startlng, startlat);
			return this;
		}

		public Builder setSensorsList(ArrayList<SensorData> sensorsList) {
			this.sensorsList = sensorsList;
			return this;
		}

		public Builder setW3WordsData(ArrayList<WordsData> wordsList) {
			this.wordsDataList = wordsList;
			return this;
		}

		public Builder setNoFlyZones(NoFlyZone noflyzones) {
			this.noFlyZone = noflyzones;
			return this;
		}

		public Builder setSeed(int seed) {
			this.seed = seed;
			return this;
		}

		@SuppressWarnings("unchecked")
		public Builder copySensors(ArrayList<SensorData> orgSensorsList) {
			this.orgSensorsList = (ArrayList<SensorData>) orgSensorsList.clone();
			return this;
		}

		public DroneBackend build() {

			return new DroneBackend(this.currDrone, this.startPositions, this.wordsDataList, this.sensorsList,
					this.noFlyZone, this.seed, this.orgSensorsList);
		}
	}

	// -----------------------------------------------------------------------
	// -----------------------------------------------------------------------
	// Main methods that can be accessed.

	/**
	 * in-place function that generates the route to sensors. Updates the
	 * droneRoute.
	 * 
	 * @throws IOException
	 * @throws InterruptedException
	 */
	@Override
	public void generateRoute(String algorithm) throws IOException, InterruptedException {

		try {

			Random randomGenerator = new Random(this.seed);
			var routePoints = this.generateSensorsToVisit(algorithm);

			// if the generated route is greater than allowed limit
			// remove some sensors.
			while (stepsCompleted > Drone.MAX_MOVES) {
				var i = randomGenerator.nextInt(wordsDataList.size());
				// Randomly delete nodes. Get sensors.
				wordsDataList.remove(i);
				sensorsList.remove(i);
				stepsCompleted = 0;
				// generate route points again!
				routePoints = this.generateSensorsToVisit(algorithm);

			}

			this.droneRoute = routePoints;
			// We now have the drone routes, so we convert it to features
			this.setGeoJsonFeatures(convertToFeatures());

		} catch (Exception e) {

			System.out.println("Something went wrong while simulating the routes!");
			e.printStackTrace();
			System.out.println("\n\nThe app will now exit.");
			System.exit(1);
		}
	}

	// -----------------------------------------------------------------------
	// -----------------------------------------------------------------------
	// Private methods used as helper functions belong here

	/**
	 * @param TSPalgorithm - String
	 * @return an array of points of sensors to visit
	 */
	private ArrayList<Point> generateSensorsToVisit(String algorithm) {

		ArrayList<Point> sensorsList = new ArrayList<Point>();
		ArrayList<SensorData> routeList;
		// Build the entire graph with vertices as sensors and edges calculated
		var graph = buildGraph(currDrone.getSensors());

		// These algorithms may fail due to several reasons such as imperfect matching
		// and the graph not being
		// suitable for the algorithm. The developer is responsible for the algorithm,
		// however, the application takes safety checks and uses the greedy/nearest neighbour
		// heuristic as a back up. Here we use an algorithm to remove edges & only get the 
		// shortest path through the TSP.
		try {
			switch (algorithm) {
			
			case "Greedy": {
				var chosenTSPAlgorithm = new GreedyHeuristicTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			} case "HeldKarp": {
				var chosenTSPAlgorithm = new HeldKarpTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			} case "NearestInsertion": {
				var chosenTSPAlgorithm = new NearestInsertionHeuristicTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			} case "NearestNeighbor": {
				var chosenTSPAlgorithm = new NearestNeighborHeuristicTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			} case "PalmerHamiltonian": {
				var chosenTSPAlgorithm = new PalmerHamiltonianCycle<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			} case "RandomTour": {
				var chosenTSPAlgorithm = new RandomTourTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			} case "GreedyHeuristic": {
				var chosenTSPAlgorithm = new RandomTourTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			} case "TwoOptHeuristic": {
				System.out.println("aa");
				var chosenTSPAlgorithm = new TwoOptHeuristicTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			} default: {
				var chosenTSPAlgorithm = new ChristofidesThreeHalvesApproxMetricTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
				}
			}
			
		} catch (Exception e) {

			if (algorithm.equals("Greedy")) {
				System.out.println("The chosen algorithm failed on this set of sensors with an error" + e
						+ ". \nA NearestNeighborHeuristicTSP is implemented as a backup.");

				var chosenTSPAlgorithm = new NearestNeighborHeuristicTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
			}

			else {
				System.out.println("The chosen algorithm failed on this set of sensors with an error" + e
						+ ". \nA Greedy heuristic is implemented as a backup.");

				var chosenTSPAlgorithm = new GreedyHeuristicTSP<SensorData, DefaultEdge>();
				var tour = chosenTSPAlgorithm.getTour(graph);
				routeList = (ArrayList<SensorData>) tour.getVertexList();
				


			}

		}
		this.shortestPathSensors = routeList;
		var routeSize = this.shortestPathSensors.size();
		// Add drone's current point. As we would like to come back to the original point.
		sensorsList.add(currDrone.getCurrentLocation());

		// Here we adjust the paths given to us by the algorithm as we used Euclidean
		// distance between the sensors & didn't account the no fly zones earlier.
		// Here we also consider the confinement region given to us
		for (int i = 0; i < routeSize; i++) {

			var currSensor = this.shortestPathSensors.get(i);
			var locs = buildValidRoute(currSensor.getWhere());
			sensorsList.addAll(locs);

			locationStrings.add(currSensor.getLocation());

			if (i == this.shortestPathSensors.size() - 1) {

				locs = buildValidRoute(startPositions);

				sensorsList.addAll(locs);
				locationStrings.add(null);

			}
		}

		return sensorsList;
	}

	/**
	 * Helper method. 
	 * 
	 * This method builds a valid route from the drone's current
	 * location to the location given as the argument.
	 * 
	 * @param Point - Location the drone should go to.
	 * @return
	 */
	private ArrayList<Point> buildValidRoute(Point reachState) {

		// get currDrone location
		var currDroneLoc = currDrone.getCurrentLocation();
		var nextLoc = reachState;
		// set lat & long
		var currLng = currDroneLoc.longitude();
		var currLat = currDroneLoc.latitude();
		var nextLng = nextLoc.longitude();
		var nextLat = nextLoc.latitude();
		// get the direction to move to reach towards the final state
		double towards = getPossibleMove(reachState);
		var power = 0;
		var times = 1;

		var pointsRoutes = new ArrayList<Point>();

		Point currPoint;
		// while the two points are not in range
		var distance = Point2D.distance(currLng, currLat, nextLng, nextLat);

		// we keep appending new points until the distance between the point (sensor's
		// location)
		// and the drone's location is within the detector's range.

		while (distance > Drone.DETECTOR_RANGE) {

			// the angle should be a multiple of 10 or else the
			// drone won't be able to move this way.
			while (towards % 10 != 0) {
				// we update our angle until this is satisfied.
				towards = towards + (0 - towards % 10);
				currDroneLoc = currDrone.getCurrentLocation();
				currPoint = newLoc(currDroneLoc, towards);

				if (validRoute(currDroneLoc, currPoint)) {
					// valid route checks if the new point isn't in the no flyzone
					var increment = -(Math.pow(-1, power)) * times * 10;
					towards = (towards + increment) % 360;

				}
			}

			// angle shouldn't be negative.
			// change to its corresponding positive angle.
			if (towards < 0) {
				towards = 360 - towards;
			}

			// get a new valid location at the angle.
			currDroneLoc = currDrone.getCurrentLocation();
			currPoint = newLoc(currDroneLoc, towards);

			// check if the new location's route is valid and by checking against
			// no fly zone.
			// check if the location has already been visited.
			var allClear = validRoute(currDroneLoc, currPoint) || pointsRoutes.contains(currPoint);

			while (allClear) {
				// one of the above is true. change it!
				var increment = -(Math.pow(-1, power)) * times;

				power += 1;
				times += 1;
				//
				increment = increment * 10; // angle should be multiple of 10
				towards = (towards + increment) % 360; // should be less than 360

				currDroneLoc = currDrone.getCurrentLocation();
				currPoint = newLoc(currDroneLoc, towards);
				// check again!
				allClear = validRoute(currDroneLoc, currPoint) || pointsRoutes.contains(currPoint);
				// continue if its still not valid.
			}

			// negative angle check.
			if (towards < 0) {
				towards = 360 - towards;
			}
			
			// reset!
			power = 0;
			times = 1;
			
			// angle should be a multiple of 10, floating point numbers aren't!
			var towardsInt = (int) towards;

			// append the next move!
			directions.add(towardsInt);
			pointsRoutes.add(currPoint);


			// update the drone's location with the new valid location
			currDrone.updateCurrentLocation(currPoint);

			currDroneLoc = currDrone.getCurrentLocation();
			nextLoc = reachState;
			// update variables & calculate distance
			currLng = currDroneLoc.longitude();
			currLat = currDroneLoc.latitude();
			nextLng = nextLoc.longitude();
			nextLat = nextLoc.latitude();
			distance = Point2D.distance(currLng, currLat, nextLng, nextLat);
			// increment counter
			// increment
			if (distance > Drone.DETECTOR_RANGE) {
				locationStrings.add(null);
			}

			stepsCompleted++;
			// get the next angle the drone should possiblty turn to reach the state.
			towards = getPossibleMove(reachState);
		}

		return pointsRoutes;
	}

	/**
	 * Given a point, this method will give us the direction the drone must move
	 * towards to reach the point.
	 * 
	 * @param Point location
	 * @return returns angle in double
	 */
	private double getPossibleMove(Point p2Loc) {

		// get variables
		var droneCurrLocation = currDrone.getCurrentLocation();

		var droneCurrLng = droneCurrLocation.longitude();
		var droneCurrLat = droneCurrLocation.latitude();

		var p2Lng = p2Loc.longitude();
		var p2Lat = p2Loc.latitude();

		double diffLng = p2Lng - droneCurrLng;
		double diffLat = p2Lat - droneCurrLat;

		double angle = 0;

		if (diffLat > 0) {

			var radians = Math.atan2(diffLat, diffLng);
			angle = Math.toDegrees(radians);
		}

		if (diffLat < 0) {
			var radians = Math.atan2(diffLat, diffLng);
			angle = 360 - (-Math.toDegrees(radians)) % 360;
		}

		return angle;
	}

	/**
	 * This method answers the question: if the drone will goes to the point will it
	 * go through a forbidden region?
	 * 
	 * @param drone
	 * @param possibleNextLoc
	 * @return
	 */
	private Boolean validRoute(Point currLoc, Point possibleNextLoc) {

		var locOutside = false;
		var NORTHEAST = NoFlyZone.NORTHEAST;
		var SOUTHEAST = NoFlyZone.SOUTHEAST;
		var SOUTHWEST = NoFlyZone.SOUTHWEST;
		var NORTHWEST = NoFlyZone.NORTHWEST;

		// Check if the lines intersect for our any of our confinement zone.
		var sideCheck = Line2D.linesIntersect(NORTHEAST.longitude(), NORTHEAST.latitude(), SOUTHEAST.longitude(),
				SOUTHEAST.latitude(), currLoc.longitude(), currLoc.latitude(), possibleNextLoc.longitude(),
				possibleNextLoc.latitude());

		locOutside = locOutside || sideCheck;

		sideCheck = Line2D.linesIntersect(SOUTHWEST.longitude(), SOUTHWEST.latitude(), NORTHWEST.longitude(),
				NORTHWEST.latitude(), currLoc.longitude(), currLoc.latitude(), possibleNextLoc.longitude(),
				possibleNextLoc.latitude());

		locOutside = locOutside || sideCheck;

		sideCheck = Line2D.linesIntersect(NORTHWEST.longitude(), NORTHWEST.latitude(), NORTHEAST.longitude(),
				NORTHEAST.latitude(), currLoc.longitude(), currLoc.latitude(), possibleNextLoc.longitude(),
				possibleNextLoc.latitude());

		locOutside = locOutside || sideCheck;

		sideCheck = Line2D.linesIntersect(SOUTHEAST.longitude(), SOUTHEAST.latitude(), SOUTHWEST.longitude(),
				SOUTHWEST.latitude(), currLoc.longitude(), currLoc.latitude(), possibleNextLoc.longitude(),
				possibleNextLoc.latitude());

		locOutside = locOutside || sideCheck;
		
		// for each edge/vertex of each polygon, check whether the the drone's current
		// location goes through the drone.
		for (var currForbiddenRegion : noFlyZone.noFlyZone) {

			var currZone = new ArrayList<Point>();

			currZone.addAll(currForbiddenRegion.coordinates().get(0));

			int currZonePoints = currZone.size();
			// get adjacent zones and current zones & check edges
			for (int i = 0; i < currZonePoints - 1; i++) {

				// ith node location
				var currLocT = currZone.get(i);
				// i +1th node location
				var nextLocT = currZone.get(i + 1);

				// intersect check
				var possibleLocs = Line2D.linesIntersect(currLocT.longitude(), currLocT.latitude(),
						nextLocT.longitude(), nextLocT.latitude(), currLoc.longitude(), currLoc.latitude(),
						possibleNextLoc.longitude(), possibleNextLoc.latitude());

				locOutside = possibleLocs || locOutside;

			}
		}

		return locOutside;
	}

	/**
	 * This method is used to find the nearest next point to the current location
	 * from the angle.
	 * 
	 * @param droneCurrLocation
	 * @param angle
	 * @return
	 */
	private Point newLoc(Point droneCurrLocation, double angle) {

		var droneCurrLng = droneCurrLocation.longitude();
		var droneCurrLat = droneCurrLocation.latitude();

		double newLat = droneCurrLat + Math.sin(angle * (Math.PI / 180)) * DIST_DRONE_CAN_MOVE;
		double newLong = droneCurrLng + Math.cos(angle * (Math.PI / 180)) * DIST_DRONE_CAN_MOVE;

		Point newLocation = Point.fromLngLat(newLong, newLat);

		return newLocation;
	}

	/**
	 * Builds the graphs and connects all the edges to each other the given
	 * arraylist of sensor data.
	 * 
	 * @param ArrayList<SensorData> sensorsList
	 * @return returns a DefaultUndirectedWeightedGraph<SensorData, DefaultEdge>
	 */
	private DefaultUndirectedWeightedGraph<SensorData, DefaultEdge> buildGraph(ArrayList<SensorData> arrSensors) {

		var graph = new DefaultUndirectedWeightedGraph<SensorData, DefaultEdge>(DefaultEdge.class);
		int count = 0;

		// convert array to points
		for (int i = 0; i < arrSensors.size(); i++) {
			graph.addVertex(arrSensors.get(i));
		}

		// O(N^2 * M*P)
		for (var sensor1 : arrSensors) {
			for (var sensor2 : arrSensors) {

				var sensor1Loc = sensor1.getWhere();
				var sensor2Loc = sensor2.getWhere();

				if (sensor1.equals(sensor2)) {
					continue;
				}
				// O(M*P) method
				else if (noFlyZone.illegalMove(sensor1Loc, sensor2Loc)) {
					// goes through the noflyzone.
					// set edgeweight to infinity/ max_val
					count++;
					graph.addEdge(sensor1, sensor2);
					graph.setEdgeWeight(sensor1, sensor2, Double.MAX_VALUE);
				} else {
					// doesn't pass through forbidden zone
					// add sensor & add distance
					graph.addEdge(sensor1, sensor2);
					count++;
					var long1 = sensor1.getWhere().longitude();
					var lat1 = sensor1.getWhere().latitude();
					var long2 = sensor2.getWhere().longitude();
					var lat2 = sensor2.getWhere().latitude();

					var dist = Point2D.distance(long1, lat1, long2, lat2);
					graph.setEdgeWeight(sensor1, sensor2, dist);

				}
			}
		}
		this.setNodeCount(count);
		return graph;
	}

	/**
	 * @return GeoJson object. converts the route & the sensor locations to GeoJson
	 *         object.
	 */
	private GeoJson convertToFeatures() throws IOException, InterruptedException {

		var visitedSensors = new HashSet<SensorData>();

		// Add drone's path as features.
		var features = new ArrayList<Feature>();
		Feature travelledRoute = Feature.fromGeometry(LineString.fromLngLats(droneRoute));
		travelledRoute.addStringProperty("fill", "#000000");
		features.add(travelledRoute);

		// for each sensor, append it to our features after adding properties (if our
		// drone visits that sensor)

		for (var currSensor : getshortestPathSensors()) {

			var currSensorLoc = currSensor.getWhere();
			var currSensorReading = currSensor.getReading();
			var currSensorBattery = currSensor.getBattery();
			var currSensorLocStr = currSensor.getLocation();

			// see if drone visits this sensor at all
			for (var currDroneLoc : droneRoute) {

				var currLng = currDroneLoc.longitude();
				var currLat = currDroneLoc.latitude();
				var nextLng = currSensorLoc.longitude();
				var nextLat = currSensorLoc.latitude();

				var droneSensorDistance = Point2D.distance(currLng, currLat, nextLng, nextLat);

				if (droneSensorDistance < DIST_DRONE_CAN_MOVE) {
					// Drone goes near the sensor, so it means we can collect its reading.
					// So add it to our features.
					visitedSensors.add(currSensor);
					var sensorGeometry = (Geometry) currSensorLoc;
					var sensorFeature = Feature.fromGeometry(sensorGeometry);

					// Use appendProperties method to appriopiately add properties.
					sensorFeature = appendProperties(currSensorReading, currSensorLocStr, currSensorBattery,
							sensorFeature, true);
					features.add(sensorFeature);

					// since we know that drone visited this sensor, we can stop the search.
					break;
				} else {
					continue;
				}
			}
		}
		// here we add gray markers to sensors that the drone couldn't
		// reach towards. We do this by keeping a hashset of all the sensors visited
		// and adding gray color to sensors not in the hashset.
		for (var currSensor : orgSensorsList) {

			var currSensorLoc = currSensor.getWhere();
			var currSensorReading = currSensor.getReading();
			var currSensorBattery = currSensor.getBattery();
			var currSensorLocStr = currSensor.getLocation();

			// O(1) retrieval (hashmap)
			boolean contains = visitedSensors.contains(currSensor);

			if (!contains) {
				setSensorsNotVisitedCount(getSensorsNotVisitedCount() + 1);
				var sensorLocGeometry = (Geometry) currSensorLoc;
				var sensorLocFeature = Feature.fromGeometry(sensorLocGeometry);

				// Mark as gray
				sensorLocFeature = appendProperties(currSensorReading, currSensorLocStr, currSensorBattery,
						sensorLocFeature, false);
				features.add(sensorLocFeature);

			}
		}

		// convert to feature collection, cast it to GeoJson and return it.
		var featuresColl = (GeoJson) FeatureCollection.fromFeatures(features);

		return featuresColl;
	}
	
	/**
	 * 
	 * Helper method used by convertToFeatures() method to convert
	 * each sensor/drone location to a feature.
	 *
	 * @param String reading - current reading of the sensor
	 * @param String loc
	 * @param double battery
	 * @param Feature currFeature
	 * @param boolean visited
	 * @return
	 */
	private Feature appendProperties(String reading, String loc, double battery, Feature currFeature, boolean visited) {
		
		// if location isn't visited, then we add a gray marker
		if (!visited) {
			currFeature.addStringProperty(this.RGB, Properties.GRAY);
			currFeature.addStringProperty("marker-color", Properties.GRAY);
			return currFeature;
		}
		// drone was visited, add features!
		currFeature.addStringProperty("marker-size", "medium");
		currFeature.addStringProperty("location", loc);

		// Add Black
		// Battery is down, so our reading is wrong. We ignore this & mark a cross
		if (battery <= 10 || reading == "NaN" || reading == "null") {

			currFeature.addStringProperty("marker-symbol", Properties.MS_CROSS);
			currFeature.addStringProperty("marker-color", Properties.BLACK);
			currFeature.addStringProperty(this.RGB, Properties.BLACK);
			return currFeature;

		} else {
			// No problem with reading or battery, append properties!
			var currReading = Double.parseDouble(reading);

			if (currReading >= 0 && currReading < 128) {

				currFeature.addStringProperty("marker-symbol", Properties.MS_LIGHTHOUSE);

				if (currReading >= 0 && currReading < 32) {

					currFeature.addStringProperty("marker-color", Properties.GREEN);
					currFeature.addStringProperty(this.RGB, Properties.GREEN);
				}
				if (currReading >= 32 && currReading < 64) {

					currFeature.addStringProperty(this.RGB, Properties.MEDIUM_GREEN);
					currFeature.addStringProperty("marker-color", Properties.MEDIUM_GREEN);
				}
				if (currReading >= 64 && currReading < 96) {

					currFeature.addStringProperty(this.RGB, Properties.LIGHT_GREEN);
					currFeature.addStringProperty("marker-color", Properties.LIGHT_GREEN);
				}
				if (currReading >= 96 && currReading < 128) {

					currFeature.addStringProperty(this.RGB, Properties.LIME_GREEN);
					currFeature.addStringProperty("marker-color", Properties.LIME_GREEN);
				}
			}

			if (currReading >= 128 && currReading < 256) {

				currFeature.addStringProperty("marker-symbol", Properties.MS_DANGER);

				if (currReading >= 128 && currReading < 160) {

					currFeature.addStringProperty(this.RGB, Properties.GOLD);
					currFeature.addStringProperty("marker-color", Properties.GOLD);
				}

				if (currReading >= 160 && currReading < 192) {

					currFeature.addStringProperty(this.RGB, Properties.ORANGE);
					currFeature.addStringProperty("marker-color", Properties.ORANGE);
				}
				if (currReading >= 192 && currReading < 224) {

					currFeature.addStringProperty(this.RGB, Properties.RED_ORANGE);
					currFeature.addStringProperty("marker-color", Properties.RED_ORANGE);
				}
				if (currReading >= 224 && currReading < 256) {

					currFeature.addStringProperty(this.RGB, Properties.RED);
					currFeature.addStringProperty("marker-color", Properties.RED);
				}
			}
		}
		// Properties has been added. Return the feature!
		return currFeature;
	}

	// -------------------------------------------------------------------
	// -------------------------------------------------------------------
	// Standard Getters and setters methods for private instance variables.

	/**
	 * @return the shortestPathSensors
	 */
	public ArrayList<SensorData> getshortestPathSensors() {
		return this.shortestPathSensors;
	}

	/**
	 * @param shortestPathSensors the shortestPathSensors to set
	 */
	public void setshortestPathSensors(ArrayList<SensorData> shortestPathSensors) {
		this.shortestPathSensors = shortestPathSensors;
	}

	/**
	 * @return the locationStrings
	 */
	public ArrayList<String> getlocationStrings() {
		return this.locationStrings;
	}

	/**
	 * @param locationStrings the locationStrings to set
	 */
	public void setLocationStrings(ArrayList<String> locationStrings) {
		this.locationStrings = locationStrings;
	}

	/**
	 * @return the visited
	 */
	public boolean getisVisited() {
		return this.visited;
	}

	/**
	 * @param visited the visited to set
	 */
	public void setVisited(boolean visited) {
		this.visited = visited;
	}

	/**
	 * @return the droneRoute
	 */
	public ArrayList<Point> getdroneRoute() {
		return droneRoute;
	}

	/**
	 * @param droneRoute the droneRoute to set
	 */
	public void setdroneRoute(ArrayList<Point> droneRoute) {
		this.droneRoute = droneRoute;
	}

	/**
	 * @param
	 */
	public int getStepsTaken() {
		return this.stepsCompleted;
	}

	/**
	 * @return
	 */
	public ArrayList<Integer> getDirections() {
		return this.directions;
	}

	/**
	 * @return the geoJsonFeatures
	 */
	public GeoJson getGeoJsonFeatures() {
		return geoJsonFeatures;
	}

	/**
	 * @param geoJsonFeatures the geoJsonFeatures to set
	 */
	public void setGeoJsonFeatures(GeoJson geoJsonFeatures) {
		this.geoJsonFeatures = geoJsonFeatures;
	}

	/**
	 * @return the sensorsNotVisitedCount
	 */
	public int getSensorsNotVisitedCount() {
		return sensorsNotVisitedCount;
	}

	/**
	 * @param int sensorsNotVisitedCount - update number of sensors that weren't visited
	 */
	public void setSensorsNotVisitedCount(int sensorsNotVisitedCount) {
		this.sensorsNotVisitedCount = sensorsNotVisitedCount;
	}

	/**
	 * @return the nodeCount
	 */
	public int getNodeCount() {
		return nodeCount;
	}

	/**
	 * @param nodeCount the nodeCount to set
	 */
	public void setNodeCount(int nodeCount) {
		this.nodeCount = nodeCount;
	}
}
