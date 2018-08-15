using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.UI.Screens;

namespace BonVoyage
{
	public class ActiveRover
	{
		public string status;
		public double toTravel;

		public Vessel vessel;
		//private ConfigNode vesselConfigNode;
		private double lastTime;
		public double LastTime { get { return lastTime; } }
		private double targetLatitude;
		private double targetLongitude;

		private double averageSpeed;
		private double amphibiousAirSpeed;
		private double amphibiousWaterSpeed;

		private double speedMultiplier;
		private RaycastHit hit;

		public bool OverWater
		{
			get
			{
				//if ((object)hit != null)
				//	return hit.distance - 1000 < 0;
				//else
					return vessel.mainBody.ocean && (vessel.situation == Vessel.Situations.SPLASHED || ScienceUtil.GetExperimentBiome(this.vessel.mainBody, targetLatitude, targetLongitude) == "Water" || this.vessel.Splashed);
			}
		}

		public double AverageSpeed
		{
			get
			{
				if (OverWater)
					return Math.Max(AmphibiousAirSpeed, AmphibiousWaterSpeed) * speedMultiplier;
				else
					return Math.Max(averageSpeed, AmphibiousAirSpeed) * speedMultiplier;
			}
		}

		public double AmphibiousAirSpeed { get { return amphibiousAirSpeed; } }
		public double AmphibiousWaterSpeed { get { return amphibiousWaterSpeed; } }

		private double distanceTravelled;
		private double distanceToTarget;
		public double yetToTravel { get { return distanceToTarget - distanceTravelled; } }

		private bool solarPowered;
		public bool bvActive;
		private bool isManned;
		private ConfigNode BVModule;
		private List<PathUtils.WayPoint> path;
		private double totalPowerAvailable;
		private double totalPowerRequired;
		private double powerRequired;
		private double chargeAmount;

		/// <summary>
		/// Initializes a new instance of the <see cref="BonVoyage.ActiveRover"/> class.
		/// </summary>
		/// <param name="v">Vessel.</param>
		/// <param name="module">Bon Voyage Module.</param>
		/// <param name="vcf">Vessel Config Node.</param>
		public ActiveRover(Vessel v, ConfigNode module)
		{
			vessel = v;

			BVModule = module;

			bvActive = bool.Parse (BVModule.GetValue ("isActive"));

			// Workaround for update from versions prior to 1.0
			try
			{
				isManned = bool.Parse (BVModule.GetValue ("isManned"));
			}
			catch
			{
				isManned = true;
			}

			solarPowered = bool.Parse (BVModule.GetValue ("solarPowered"));

			lastTime = double.Parse (BVModule.GetValue ("lastTime"));
			distanceTravelled = double.Parse (BVModule.GetValue ("distanceTravelled"));
			distanceToTarget = double.Parse (BVModule.GetValue ("distanceToTarget"));
			targetLatitude = double.Parse (BVModule.GetValue ("targetLatitude"));
			targetLongitude = double.Parse (BVModule.GetValue ("targetLongitude"));

			averageSpeed = double.Parse(BVModule.GetValue ("averageSpeed"));
			amphibiousAirSpeed = double.Parse(BVModule.GetValue("amphibiousAirSpeed"));
			amphibiousWaterSpeed = double.Parse(BVModule.GetValue("amphibiousWaterSpeed"));

			totalPowerAvailable = double.Parse(BVModule.GetValue("totalPowerAvailable"));
			totalPowerRequired = double.Parse(BVModule.GetValue("totalPowerRequired"));
			chargeAmount = double.Parse(BVModule.GetValue("chargeAmount"));

			try
			{
				powerRequired = double.Parse(BVModule.GetValue("powerRequired"));
			}
			catch
			{
				powerRequired = 0;
			}
			path = PathUtils.DecodePath(BVModule.GetValue("pathEncoded"));
			speedMultiplier = 1.0;
		}

		/// <summary>
		/// Update rover.
		/// </summary>
		/// <param name="currentTime">Current time.</param>
		public void Update(double currentTime)
		{
			if (vessel.isActiveVessel)
			{
				status = "current";
				return;
			}

			if (!bvActive || vessel.loaded)
			{
				status = "idle";
				return;
			}

			DoRayCast(vessel.latitude, vessel.longitude, vessel.altitude);

			Vector3d vesselPos = vessel.mainBody.position - vessel.GetWorldPos3D();
			Vector3d toKerbol = vessel.mainBody.position - FlightGlobals.Bodies[0].position;
			double angle = Vector3d.Angle(vesselPos, toKerbol);

			// Speed penalties at twighlight and at night
			if (angle > 90 && isManned && totalPowerRequired < totalPowerAvailable + chargeAmount)
				speedMultiplier = 0.25;
			else if (angle > 85 && isManned && totalPowerRequired < totalPowerAvailable + chargeAmount)
				speedMultiplier = 0.5;
			else if (angle > 80 && isManned && totalPowerRequired < totalPowerAvailable + chargeAmount)
				speedMultiplier = 0.75;
			else
				speedMultiplier = 1.0;

			// No moving at night, or when there's not enougth solar light for solar powered rovers
			if (angle > 90 && solarPowered && ((totalPowerAvailable + chargeAmount) < totalPowerRequired))
			{
				status = "awaiting sunlight";
				lastTime = currentTime;
				BVModule.SetValue("lastTime", currentTime.ToString());

				return;
			}

			double deltaT = currentTime - lastTime;

			double deltaS = AverageSpeed * deltaT;

			double bearing = GeoUtils.InitialBearing(
				vessel.latitude,
				vessel.longitude,
				targetLatitude,
				targetLongitude
			);
			distanceTravelled += deltaS;
			if (distanceTravelled >= distanceToTarget)
			{
				//				vessel.latitude = targetLatitude;
				//				vessel.longitude = targetLongitude;
				if (!MoveSafe(targetLatitude, targetLongitude))
				{
					distanceTravelled -= deltaS;
					//vessel.RequestResource(vessel.rootPart, PartResourceLibrary.ElectricityHashcode, powerRequired * deltaT, false);
				}
				else
				{
					distanceTravelled = distanceToTarget;

					bvActive = false;
					BVModule.SetValue ("isActive", "False");
					BVModule.SetValue ("distanceTravelled", distanceToTarget.ToString ());
					BVModule.SetValue ("pathEncoded", "");

//					BVModule.GetNode ("EVENTS").GetNode ("Activate").SetValue ("active", "True");
//					BVModule.GetNode ("EVENTS").GetNode ("Deactivate").SetValue ("active", "False");

					if (BonVoyage.Instance.AutoDewarp)
					{
						if (TimeWarp.CurrentRate > 3)
							TimeWarp.SetRate (3, true);
						if (TimeWarp.CurrentRate > 0)
							TimeWarp.SetRate (0, false);
						ScreenMessages.PostScreenMessage (vessel.vesselName + " has arrived to destination at " + vessel.mainBody.name);
					}
					HoneyImHome ();
				}
				status = "idle";
			}
			else
			{
				int step = Convert.ToInt32(Math.Floor(distanceTravelled / PathFinder.StepSize));
				double remainder = distanceTravelled % PathFinder.StepSize;

				if (step < path.Count - 1)
					bearing = GeoUtils.InitialBearing(
						path[step].latitude,
						path[step].longitude,
						path[step + 1].latitude,
						path[step + 1].longitude
					);
				else
					bearing = GeoUtils.InitialBearing(
						path[step].latitude,
						path[step].longitude,
						targetLatitude,
						targetLongitude
					);

				double[] newCoordinates = GeoUtils.GetLatitudeLongitude(
					path[step].latitude,
					path[step].longitude,
					bearing,
					remainder,
					vessel.mainBody.Radius
				);

				//				vessel.latitude = newCoordinates[0];
				//				vessel.longitude = newCoordinates[1];
				if (!MoveSafe(newCoordinates[0], newCoordinates[1]))
				{
					distanceTravelled -= deltaS;
					status = "idle";
				}
				else
				{
					status = "roving";
					//vessel.RequestResource(vessel.rootPart, PartResourceLibrary.ElectricityHashcode, powerRequired * deltaT, false);
				}
			}
//			vessel.altitude = GeoUtils.TerrainHeightAt(vessel.latitude, vessel.longitude, vessel.mainBody);
			Save (currentTime);
		}

		/// <summary>
		/// Save data to ProtoVessel.
		/// </summary>
		public void Save(double currentTime)
		{
			lastTime = currentTime;

			BVModule.SetValue("distanceTravelled", (distanceTravelled).ToString());
			BVModule.SetValue("lastTime", currentTime.ToString());
		
            vessel.protoVessel.latitude = vessel.latitude;
            vessel.protoVessel.longitude = vessel.longitude;
            vessel.protoVessel.altitude = vessel.altitude;
            vessel.protoVessel.landedAt = vessel.mainBody.bodyName;
            vessel.protoVessel.displaylandedAt = vessel.mainBody.bodyDisplayName.Replace("^N", "");		
		}

		/// <summary>
		/// Prevent crazy torpedoing active vessel :D
		/// </summary>
		/// <returns><c>true</c>, if rover was moved, <c>false</c> otherwise.</returns>
		/// <param name="latitude">Latitude.</param>
		/// <param name="longitude">Longitude.</param>
		private bool MoveSafe(double latitude, double longitude)
		{
			double altitude = GeoUtils.TerrainHeightAt(latitude, longitude, vessel.mainBody);

			if (FlightGlobals.ActiveVessel != null)
			{
				Vector3d newPos = vessel.mainBody.GetWorldSurfacePosition (latitude, longitude, altitude);
				Vector3d actPos = FlightGlobals.ActiveVessel.GetWorldPos3D ();

				double distance = Vector3d.Distance (newPos, actPos);
				if (distance <= 2400)
				{
					return false;
				}
//				VesselRanges ranges = active.vesselRanges.GetSituationRanges(Vessel.Situations.LANDED || Vessel.Situations.FLYING);
//				vessel.GoOffRails ();
//				vessel.Load ();
			}

			vessel.latitude = latitude;
			vessel.longitude = longitude;

			altitude = altitude + vessel.heightFromTerrain;

			if (vessel.mainBody.ocean)
				altitude = Math.Max(altitude, 0);

			vessel.altitude = altitude + vessel.heightFromTerrain;

			if (vessel.mainBody.ocean)
			{
				if (altitude <= 0)
				{
					vessel.Splashed = true;
					vessel.Landed = false;
					vessel.situation = Vessel.Situations.SPLASHED;
				}
				else
				{
					vessel.Splashed = false;
					vessel.Landed = true;
					vessel.situation = Vessel.Situations.LANDED;
				}
			}

			vessel.displaylandedAt = ScienceUtil.GetExperimentBiome(this.vessel.mainBody, latitude, longitude);
			//Quaternion rotation;
			//Vector3d from = vessel.vesselTransform.up;
			//Vector3d to = GetTerrainNormal(latitude, longitude, altitude);
			//rotation = Quaternion.FromToRotation(from, to);
			//vessel.SetRotation(rotation);

			return true;
		}

		public Vector3 DoRayCast(double latitude, double longitude, double altitude)
		{
			Vector3d worldRayCastStart = vessel.mainBody.GetWorldSurfacePosition(latitude, longitude, altitude + 1000);
			// a point a bit below it, to aim down to the terrain:
			Vector3d worldRayCastStop = vessel.mainBody.GetWorldSurfacePosition(latitude, longitude, altitude + 900);

			if (Physics.Raycast(worldRayCastStart, (worldRayCastStop - worldRayCastStart), out hit, float.MaxValue, 32768))
				return hit.normal;
			else
				return Vector3.up;
		}

		/// <summary>
		/// Notify that rover has arrived
		/// </summary>
		private void HoneyImHome() {
			MessageSystem.Message message = new MessageSystem.Message (
                "Rover arrived",
				//------------------------------------------
                "<color=#74B4E2>" + vessel.vesselName + "</color>" +
                " has arrived to destination\n<color=#AED6EE>LAT:" +
                targetLatitude.ToString ("F2") + "</color>\n<color=#AED6EE>LON:" +
                targetLongitude.ToString ("F2") +
                "</color>\n<color=#82BCE5>At " + vessel.mainBody.name + ".</color>\n" +
                "Distance travelled: " +
                "<color=#74B4E2>" + distanceTravelled.ToString ("N") + "</color> meters",
				//------------------------------------------
                MessageSystemButton.MessageButtonColor.GREEN,
                MessageSystemButton.ButtonIcons.COMPLETE
            );
			MessageSystem.Instance.AddMessage (message);
		}
	}
}
