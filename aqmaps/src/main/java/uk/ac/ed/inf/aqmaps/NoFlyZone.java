package uk.ac.ed.inf.aqmaps;

import java.awt.geom.Line2D;
import java.util.ArrayList;

import com.mapbox.geojson.Point;
import com.mapbox.geojson.Polygon;

public class NoFlyZone {

	// Static constants for locations
	// KFC
	static final Point NORTHEAST = Point.fromLngLat(-3.184319, 55.946233);
	// Forest Hill
	static final Point NORTHWEST = Point.fromLngLat(-3.192473, 55.946233);
	// Buccleuch St bus stop
	static final Point SOUTHEAST = Point.fromLngLat(-3.184319, 55.942617);
	// Top of the Meadows
	static final Point SOUTHWEST = Point.fromLngLat(-3.192473, 55.942617);

	protected ArrayList<Polygon> noFlyZone;

	public NoFlyZone(ArrayList<Polygon> noFlyZone) {
		this.noFlyZone = noFlyZone;
	}

	/**
	 * A general helper method that checks if the shortest path between two location points
	 * go through any of the forbidden zone (NoFlyZone)
	 * @param Point location1
	 * @param Point location2
	 * @return boolean
	 */
	public boolean illegalMove(Point location1, Point location2) {

		// set longitude & latitudes of the points
		var longS1 = location1.longitude();
		var latS1 = location1.latitude();
		var longS2 = location2.longitude();
		var latS2 = location2.latitude();
		
		var path = new Line2D.Double(longS1, latS1, longS2, latS2);

		// for each vertex of a forbidden zone, get the adjacent vertex and see
		// if our point intersects it.
		for (Polygon currForbiddenRegion : this.noFlyZone) {

			var vertexLocations = currForbiddenRegion.coordinates().get(0);

			// for each vertex in the polygon, check if our line intersects it by
			// checking it with the adjacent vertexes.
			for (int i = 0; i < vertexLocations.size() - 1; i++) {

				Point currVertexLoc = vertexLocations.get(i);
				Point adjacentVertexLoc = vertexLocations.get(i + 1);

				var longV1 = currVertexLoc.longitude();
				var latV1 = currVertexLoc.latitude();
				var longV2 = adjacentVertexLoc.longitude();
				var latV2 = adjacentVertexLoc.latitude();

				var currVertexPath = new Line2D.Double(longV1, latV1, longV2, latV2);

				if (path.intersectsLine(currVertexPath)) {
					return true;
				} else if (currVertexPath.intersectsLine(path)) {
					return true;
				}
			}
		}

		return false;
	}

}
