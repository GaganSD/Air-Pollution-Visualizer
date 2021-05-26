package uk.ac.ed.inf.aqmaps;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.http.HttpResponse.BodyHandlers;

import java.util.ArrayList;
import java.io.IOException;
import java.lang.reflect.Type;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import com.mapbox.geojson.FeatureCollection;
import com.mapbox.geojson.Polygon;

public class WebClient {

	private static final HttpClient ourClient = HttpClient.newHttpClient();
	private int port;
	private String prefix;

	/**
	 * @param port
	 */
	public WebClient(int port) {
		setPort(port);
		this.prefix = "http://localhost:" + port + "/";

	}

	public WebClient() {
		setPort(80);
		this.prefix = "http://localhost:" + 80 + "/";
	}

	/**
	 * @param port
	 */
	public void setPort(int port) {
		this.port = port;
	}

	/**
	 * 
	 * @return port number
	 */
	public int getPort() {
		return this.port;
	}

	/**
	 * 
	 * @param url as string.
	 * @return JSON as String.
	 * @throws IOException
	 * @throws InterruptedException
	 */
	public String getJSONFromURL(String url) throws IOException, InterruptedException {
		// this is
		String JSONStr = "";

		try {

			// Get a http request.
			HttpRequest request = HttpRequest.newBuilder().uri(URI.create(url)).build();
			HttpResponse<String> response = ourClient.send(request, BodyHandlers.ofString());

			// if successful get the json file from the response.body()
			JSONStr = response.body();

			return JSONStr;

		} catch (Exception e) {

			System.out
					.println("Fatal error occured. \n Here's the error message: " + e + "\nThe system will now exit.");
			System.exit(1);
		}

		return JSONStr;
	}

	/**
	 * @param day   : String
	 * @param month : String
	 * @param year  : year
	 * @return ArrayList<SensorData> sensorsList
	 * @throws IOException
	 * @throws InterruptedException
	 */
	public ArrayList<SensorData> getSensorData(String day, String month, String year)
			throws IOException, InterruptedException {
		// TODO Auto-generated method stub

		// Build the URL
		String suffix = "/air-quality-data.json";
		String midURL = "/maps/" + year + "/" + month + "/" + day;
		String urlString = this.prefix + midURL + suffix;
		try {
			// Get JSON String
			String airQualityDataJSON = getJSONFromURL(urlString);
			// TODO: Handling ERROR 404 status code & any other exceptions
			Type typeOfObjList = new TypeToken<ArrayList<SensorData>>() {
			}.getType();
			ArrayList<SensorData> airQualityDataList = new Gson().fromJson(airQualityDataJSON, typeOfObjList);
			return airQualityDataList;

		} catch (Exception e) {
			System.out.println("Fatal error occured. \n Here's the error message: " + e + "\nThe system will now end.");
			System.exit(1);
		}
		// should not be reachable
		return new ArrayList<SensorData>();
	}

	/**
	 * 
	 * @param day
	 * @param month
	 * @param year
	 * @return ArrayList of Polygon object
	 * @throws IOException
	 * @throws InterruptedException
	 */
	public ArrayList<Polygon> getNoFlyZones(String day, String month, String year)
			throws IOException, InterruptedException {

		var noFlyZones = new ArrayList<Polygon>();

		try {
			// Build the URL
			String suffix = "/buildings/no-fly-zones.geojson";
			String urlString = this.prefix + suffix;

			// Get JSON String
			String noFlyZonesJSON = getJSONFromURL(urlString);

			// Parse JSON String
			var collectionFeatures = FeatureCollection.fromJson(noFlyZonesJSON).features();

			for (var currFeature : collectionFeatures) {

				Polygon p = (Polygon) currFeature.geometry();
				noFlyZones.add(p);
			}

		} catch (Exception e) {
			System.out.println("Fatal error occured. \n Here's the error message: " + e + "\nThe system will now end.");
			System.exit(1);
		}

		return noFlyZones;
	}

	/**
	 * @param sensorList
	 * @return ArrayList of wordsData object.
	 * @throws IOException
	 * @throws InterruptedException
	 */
	public ArrayList<WordsData> parsingWords(ArrayList<SensorData> sensorList)
			throws IOException, InterruptedException {

		// Build the url: part 1
		String suffix = "/details.json";
		String prefix = this.prefix + "words/";

		ArrayList<WordsData> words = new ArrayList<WordsData>();

		try {

			for (var s : sensorList) {

				String a = s.getLocation();

				// Build the url: part 2
				String[] threeWords = a.split("\\.");
				String url = prefix + threeWords[0] + "/" + threeWords[1] + "/" + threeWords[2] + suffix;

				// Parse the string.
				String JSONStr = getJSONFromURL(url);
				var currWordData = new Gson().fromJson(JSONStr, WordsData.class);

				words.add(currWordData);

			}

		} catch (Exception e) {
			System.out.println("Fatal error occured. \n Here's the error message: " + e + "\nThe system will now end.");
			System.exit(1);
		}
		
		return words;

	}

}
