using System;
using UnityEngine;

namespace BonVoyage
{
	public class GeoUtils
	{
		private const double PI = Math.PI;

		// Haversine algorithm
		// http://www.movable-type.co.uk/scripts/latlong.html
		internal static double GetDistance(double startLatitude, double startLongitude, double endLatitude, double endLongitude, double radius)
		{
			double deltaLatitude =  PI/ 180 * (endLatitude - startLatitude);
			double deltaLongitude = PI/ 180 * (endLongitude - startLongitude);

			startLatitude = PI/ 180 * startLatitude;
			startLongitude = PI/ 180 * startLongitude;
			endLatitude = PI/ 180 * endLatitude;
			endLongitude = PI/ 180 * endLongitude;

			double a = Math.Pow(Math.Sin(deltaLatitude / 2), 2) + Math.Cos(startLatitude) * Math.Cos(endLatitude) *
				Math.Pow(Math.Sin(deltaLongitude / 2), 2);

			double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

			double distance = radius * c;
			return distance;
		}

		// Rad version, useless?
		internal static double GetDistanceRad(double startLatitude, double startLongitude, double endLatitude, double endLongitude, double radius)
		{
			double deltaLatitude =  endLatitude - startLatitude;
			double deltaLongitude = endLongitude - startLongitude;

			double a = Math.Pow(Math.Sin(deltaLatitude / 2), 2) + Math.Cos(startLatitude) * Math.Cos(endLatitude) *
				Math.Pow(Math.Sin(deltaLongitude / 2), 2);

			double c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

			double distance = radius * c;
			return distance;
		}

		// Alternative haversine distance implementation
		// https://www.kaggle.com/c/santas-stolen-sleigh/forums/t/18049/simpler-faster-haversine-distance
		/*		public static double GetDistanceAlt()
		{
		}
*/

		// Bearing from start to end
		// http://www.movable-type.co.uk/scripts/latlong.html
		internal static double InitialBearing(double startLatitude, double startLongitude, double targetLatitude, double targetLongitude)
		{
			startLatitude = PI/ 180 * startLatitude;
			startLongitude = PI/ 180 * startLongitude;
			targetLatitude = PI/ 180 * targetLatitude;
			targetLongitude = PI/ 180 * targetLongitude;

			double y = Math.Sin (targetLongitude - startLongitude) * Math.Cos (targetLatitude);
			double x = Math.Cos (startLatitude) * Math.Sin (targetLatitude) -
				Math.Sin (startLatitude) * Math.Cos (targetLatitude) * Math.Cos (targetLongitude - startLongitude);

			double bearing = Math.Atan2 (y, x);
			bearing = (bearing * 180.0 / PI + 360) % 360;

			return bearing;
		}

		// Bearing from start to end, rad version
		// http://www.movable-type.co.uk/scripts/latlong.html
		internal static double InitialBearingRad(double startLatitude, double startLongitude, double targetLatitude, double targetLongitude)
		{
			double y = Math.Sin (targetLongitude - startLongitude) * Math.Cos (targetLatitude);
			double x = Math.Cos (startLatitude) * Math.Sin (targetLatitude) -
				Math.Sin (startLatitude) * Math.Cos (targetLatitude) * Math.Cos (targetLongitude - startLongitude);

			double bearing = Math.Atan2 (y, x);
			return bearing;
		}

		// Bearing at destination
		// http://www.movable-type.co.uk/scripts/latlong.html
		internal static double FinalBearing(double startLatitude, double startLongitude, double targetLatitude, double targetLongitude) {
			double bearing = InitialBearing (targetLatitude, targetLongitude, startLatitude, startLongitude);
			bearing = (bearing + 180) % 360;
			return bearing;
		}

		// Bearing at destination, rad version
		// http://www.movable-type.co.uk/scripts/latlong.html
		internal static double FinalBearingRad(double startLatitude, double startLongitude, double targetLatitude, double targetLongitude) {
			double bearing = InitialBearingRad (targetLatitude, targetLongitude, startLatitude, startLongitude);
			//			bearing = (bearing + 180) % 360;
			return bearing;
		}

		// "Reverse Haversine" Formula
		// https://gist.github.com/shayanjm/644d895c1fad80b49919
		internal static double[] GetLatitudeLongitude(double latStart, double lonStart, double bearing, double distance, double radius)
		{
			latStart = PI/ 180 * latStart;
			lonStart = PI/ 180 * lonStart;
			bearing = PI/ 180 * bearing;

			var latEnd = Math.Asin(Math.Sin(latStart) * Math.Cos(distance / radius) +
				Math.Cos(latStart) * Math.Sin(distance / radius) * Math.Cos(bearing));
			var lonEnd = lonStart + Math.Atan2(Math.Sin(bearing) * Math.Sin(distance / radius) * Math.Cos(latStart),
				Math.Cos(distance / radius) - Math.Sin(latStart) * Math.Sin(latEnd));

			return new double[] {
				latEnd * 180.0 / PI,
				lonEnd * 180.0 / PI
			};
		}

		/// <summary>
		/// Step back from target by 'step' meters
		/// </summary>
		/// <returns>The back.</returns>
		/// <param name="startLatitude">Start latitude.</param>
		/// <param name="startLongitude">Start longitude.</param>
		/// <param name="endLatitude">End latitude.</param>
		/// <param name="endLongitude">End longitude.</param>
		/// <param name="radius">Radius.</param>
		/// <param name="step">Step.</param>
		internal static double[] StepBack(double startLatitude, double startLongitude, double endLatitude, double endLongitude, double radius, double step) {
			double distanceToTarget = GetDistance (startLatitude, startLongitude, endLatitude, endLongitude, radius);
			if (distanceToTarget <= step)
				return null;
			distanceToTarget -= step;
			double bearing = InitialBearing(startLatitude, startLongitude, endLatitude, endLongitude);
			return GetLatitudeLongitude(startLatitude, startLongitude, bearing, distanceToTarget, radius);
		}

		internal static double[] GetLatitudeLongitudeRad(double latStart, double lonStart, double bearing, double distance, double radius)
		{
			var latEnd = Math.Asin(Math.Sin(latStart) * Math.Cos(distance / radius) +
				Math.Cos(latStart) * Math.Sin(distance / radius) * Math.Cos(bearing));
			var lonEnd = lonStart + Math.Atan2(Math.Sin(bearing) * Math.Sin(distance / radius) * Math.Cos(latStart),
				Math.Cos(distance / radius) - Math.Sin(latStart) * Math.Sin(latEnd));

			return new double[] {
				latEnd,
				lonEnd
			};
		}

		private const int TERRAIN_MASK_BIT = 15;

		// Get altitude at point. Cinically stolen from Waypoint Manager source code :D
		internal static double TerrainHeightAt(double latitude, double longitude, CelestialBody body)
		{
			// Not sure when this happens - for Sun and Jool?
			if (body.pqsController == null)
			{
				return 0;
			}

			// Figure out the terrain height
			double alt;
			double latRads = PI / 180.0 * latitude;
			double lonRads = PI / 180.0 * longitude;
			Vector3d radialVector = new Vector3d(Math.Cos(latRads) * Math.Cos(lonRads), Math.Sin(latRads), Math.Cos(latRads) * Math.Sin(lonRads));

			alt = body.pqsController.GetSurfaceHeight(radialVector) - body.pqsController.radius;

			if (body.ocean)
				alt = Math.Max(0, alt);

			return alt;

			var bodyUpVector = new Vector3d(1, 0, 0);
			bodyUpVector = QuaternionD.AngleAxis(latitude, Vector3d.forward/*around Z axis*/) * bodyUpVector;
			bodyUpVector = QuaternionD.AngleAxis(longitude, Vector3d.down/*around -Y axis*/) * bodyUpVector;

			//alt = body.pqsController.GetSurfaceHeight (bodyUpVector) - body.pqsController.radius;
			//			return Math.Max(body.pqsController.GetSurfaceHeight(radialVector) - body.pqsController.radius, 0.0);
			alt = body.TerrainAltitude(latitude, longitude, false);
			return alt;

			Vector3d worldRayCastStart = body.GetWorldSurfacePosition(latitude, longitude, alt + 1000);
			// a point a bit below it, to aim down to the terrain:
			Vector3d worldRayCastStop = body.GetWorldSurfacePosition(latitude, longitude, alt + 900);
			RaycastHit hit;
			if (Physics.Raycast(worldRayCastStart, (worldRayCastStop - worldRayCastStart), out hit, float.MaxValue, 32768))
			{
				// Ensure hit is on the topside of planet, near the worldRayCastStart, not on the far side.
				if (Mathf.Abs(hit.distance) <= alt + 1000)
				{
					// Okay a hit was found, use it instead of PQS alt:
					alt = ((body.Radius + 1000) - hit.distance);
				}
				else
					alt = 0;
			}
			return alt;
		}

		internal static Vector3 GetTerrainNormal(CelestialBody body, double latitude, double longitude, double altitude)
		{
			Vector3d worldRayCastStart = body.GetWorldSurfacePosition(latitude, longitude, altitude + 1000);
			// a point a bit below it, to aim down to the terrain:
			Vector3d worldRayCastStop = body.GetWorldSurfacePosition(latitude, longitude, altitude + 900);
			RaycastHit hit;
			if (Physics.Raycast(worldRayCastStart, (worldRayCastStop - worldRayCastStart), out hit, float.MaxValue, 32768))
				return hit.normal;
			else
				return Vector3.up;
		}
	}
}
