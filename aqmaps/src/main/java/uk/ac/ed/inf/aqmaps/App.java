package uk.ac.ed.inf.aqmaps;
/*
 * Written by Gagan Devagiri
 * for Informatics Large Practical coursework 2
 * Student number: s1854008
 */


import java.io.IOException;

/**
 * 
 * @author s1854008
 *
 */
public class App 
{
	/**
	 * Runs the application -> builds the graph -> outputs geojson & txt file
	 * @param args
	 * @throws IOException
	 * @throws InterruptedException
	 */
	public static void main(String[] args) throws IOException, InterruptedException
    {

		
		// Create new Handler object.
		var io = new IOHandler();
		
		// Complete basic checks and populate instance variables.
		io.parseInputs(args);
		System.out.println("STATUS: ARGUMENTS VALIDATED");

		// Create a serverClient that talks with our web server.
		WebClient serverClient = new WebClient(io.port);
		// Get the ArrayList of SensorsData, Polygons and wordsData respectively from the server
		var sensorsList = serverClient.getSensorData(io.day, io.month, io.year);
		var noFlyZoneList = serverClient.getNoFlyZones(io.day, io.month, io.year);
		var wordsList = serverClient.parsingWords(sensorsList);
		System.out.println("STATUS: RETRIEVED DATA FROM WEB SERVER");
		
		// Wrap NoFlyZone around ArrayList<Polygons> to provide more flexiblity
		NoFlyZone noFlyZone = new NoFlyZone(noFlyZoneList);

		
		// Create DroneX and droneXbackend objects by using the builder Design Pattern 
		// & connect the drone with its backend
		// System.out.println(wordsList.size());

		var droneX = new Drone.Builder()
				.setLongitude(io.longitude)
				.setLatitude(io.latitude)
				.setSensorsData(sensorsList)
				.setW3WordsData(wordsList)
				.setNoFlyZones(noFlyZone)
				.build();
		
		var droneXbackend = new DroneBackend.Builder()
				.connectDrone(droneX)
				.setStartPosition(io.longitude, io.latitude)
				.setSensorsList(sensorsList)
				.setW3WordsData(wordsList)
				.setNoFlyZones(noFlyZone)
				.setSeed(io.seed)
				.copySensors(sensorsList)
				.build();
	
		// choose a TSP algorithm:
		// You can choose from any of these algorithms:
		// https://jgrapht.org/javadoc-1.4.0/org/jgrapht/alg/tour/package-summary.html
		var algorithm = "best";
	
		System.out.println("STATUS: GENERATING ROUTE..");
		// Simulate the drone path & generate the best route!
		droneXbackend.generateRoute(algorithm);
		
		System.out.println("STATUS: SUCCESSFULLY GENERATED ROUTE");
		// connect drone's io with backend to print the route to the system.
		io.ConnectDroneBackend(droneXbackend);
		System.out.println("STATUS: WRITING FILES");
		// Finally write the necessary files to IO & 
		// print statistics to IO
		io.writeToSys();
		io.printInterestingFacts();
		
		System.out.println("STATUS: SIMULATION COMPLETED.");
		System.exit(0);
		
    }
}
