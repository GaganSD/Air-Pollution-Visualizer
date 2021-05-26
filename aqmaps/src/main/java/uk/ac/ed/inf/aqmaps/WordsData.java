package uk.ac.ed.inf.aqmaps;

import com.mapbox.geojson.Point;

/**
 * @author s1854008
 *
 */
public class WordsData {

	private String country;
	private String words;
	private String language;
	private String maps;
	private String nearestPlace;

	/**
	 * 
	 * @param words
	 * @param country
	 * @param language
	 * @param maps
	 * @param nearestPlace
	 */
	public WordsData(String words, String country, String language, String maps, String nearestPlace) {
		words = this.words;
		country = this.country;
		language = this.language;
		maps = this.maps;
		nearestPlace = this.nearestPlace;
	}

	private Location coordinates;
	
	// nested static class - used to match the JSON format
	public static class Location {
		double lng;
		double lat;

		public Point getPoint() {
			return Point.fromLngLat(lng, lat);
		}
	}
	
	/**
	 * getter for the coordinates
	 * @return Location
	 */
	public Location getCoordinates() {
		return this.coordinates;
	}
	
	/**
	 * getter for the words 
	 * @return words
	 */
	public String getWords() {
		return this.words;
	}

	/**
	 * getter
	 * @return maps
	 */
	public String getMap() {
		return this.maps;
	}
	
	/**
	 * getter
	 * @return nearestPlace
	 */
	public String getnearestPlace() {
		return this.nearestPlace;
	}

}
