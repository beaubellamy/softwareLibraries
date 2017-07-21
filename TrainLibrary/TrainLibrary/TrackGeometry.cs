using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TrainLibrary
{
    public class TrackGeometry
    {
        /* Create a tools Class. */
        //Tools tool = new Tools();
        /* Create a processing Class. */
        Processing processing = new Processing();


        public int corridorNumber;
        public string corridorName;
        public GeoLocation point = new GeoLocation();
        public double elevation;
        public double kilometreage;
        public double virtualKilometreage;
        public bool isLoopHere;
        public double temporarySpeedRestriction;
        public bool isTSRHere;

        /* Constructors */

        /// <summary>
        /// Default track geometry constructor
        /// </summary>
        public TrackGeometry()
        {
            this.corridorNumber = 0;
            this.corridorName = "not specified";
            this.point.latitude = -33.8519;   //Sydney Harbour Bridge
            this.point.longitude = 151.2108;
            this.elevation = 0;
            this.kilometreage = 0;
            this.virtualKilometreage = 0;
            this.isLoopHere = false;
            this.temporarySpeedRestriction = 0;
            this.isTSRHere = false;
        }

        /// <summary>
        /// Track Geometry Constructor
        /// </summary>
        /// <param name="corridorNumber">Integer value representing the corridor</param>
        /// <param name="corridorName">The corridor name</param>
        /// <param name="latitude">The geographic latitude of the track</param>
        /// <param name="longitude">The geographic longitude of the track.</param>
        /// <param name="elevation">The elevation of the track.</param>
        /// <param name="kilometreage">The kilometreage of the track.</param>
        /// <param name="virtualKilometreage">The cummulative kilometreage for a single corridor track.</param>
        /// <param name="loop">A flag indicating if a loop is located at this location.</param>
        public TrackGeometry(int corridorNumber, string corridorName, double latitude, double longitude, double elevation, double kilometreage, double virtualKilometreage, bool loop)
        {
            this.corridorNumber = corridorNumber;
            this.corridorName = corridorName;
            this.point.latitude = latitude;
            this.point.longitude = longitude;
            this.elevation = elevation;
            this.kilometreage = kilometreage;
            this.virtualKilometreage = virtualKilometreage;
            this.isLoopHere = loop;
            this.temporarySpeedRestriction = 0;
            this.isTSRHere = false;
        }

        /// <summary>
        /// Finds the kilometreage point on the track that is closest to the supplied geographic location (latitude, longitude)
        /// </summary>
        /// <param name="TrackGeometry">List of TrackGeometry objects.</param>
        /// <param name="Location">Geographic location with latitude and longitude.</param>
        /// <returns>The kilometreage of the closest point to the track geometry.</returns>
        public double findClosestTrackGeometryPoint(List<TrackGeometry> trackGeometry, GeoLocation Location)
        {
            /* Set up initial values. */
            int minimumIndex = 0;
            double minimumDistance = double.MaxValue;
            double distance = 0;
            GeoLocation trackPoint = new GeoLocation();

            for (int trackIdx = 0; trackIdx < trackGeometry.Count(); trackIdx++)
            {
                /* Set the current track geometry point. */
                trackPoint = trackGeometry[trackIdx].point;
                /* Calcualte the distance between the current track point and the location supplied. */
                distance = TrainLibrary.Processing.calculateGreatCircleDistance(trackPoint, Location);
                
                /* Determine when the minimum distance is reached. */
                if (distance < minimumDistance)
                {
                    minimumDistance = distance;
                    minimumIndex = trackIdx;
                }

            }

            /* Return the kilometreage of the point that is closest to the location supplied. */
            return trackGeometry[minimumIndex].virtualKilometreage;
        }

        /// <summary>
        /// Finds the kilometreage point on the track that is closest to the supplied kilometerage point.
        /// </summary>
        /// <param name="TrackGeometry">List of TrackGeometry objects.</param>
        /// <param name="location">Train kilometerage.</param>
        /// <returns>The index of the closest track kilometreage point to the train kilometreage.</returns>
        public int findClosestTrackGeometryPoint(List<TrackGeometry> trackGeometry, double location)
        {
            /* Set up initial values. */
            int minimumIndex = 0;
            double minimumDistance = double.MaxValue;
            double distance = 0;
            double trackKm = 0;

            for (int trackIdx = 0; trackIdx < trackGeometry.Count(); trackIdx++)
            {
                /* Set the current track geometry point. */
                trackKm = trackGeometry[trackIdx].virtualKilometreage;
                /* Calcualte the distance between the current track point and the location supplied. */
                distance = Math.Abs(trackKm - location);

                /* Determine when the minimum distance is reached. */
                if (distance < minimumDistance)
                {
                    minimumDistance = distance;
                    minimumIndex = trackIdx;
                }

            }

            if (minimumDistance == double.MaxValue)
                return -1;

            return minimumIndex;
        }

        /// <summary>
        /// Match the train location with the closest point on the track for the real track kmPost.
        /// </summary>
        /// <param name="TrainJourney">A list of a points describing a single train journey.</param>
        /// <param name="track">The track geometry information.</param>
        public void matchTrainLocationToTrackGeometry(List<TrainJourney> TrainJourney, List<TrackGeometry> track)
        {
            foreach (TrainJourney journey in TrainJourney)
            {// was .kmpost
                /* Find the closest km marker in the track geometry to the current train location. */
                journey.kilometreage = findClosestTrackGeometryPoint(track, journey.location);

            }

        }


    } // Class TrackGeometry
}
