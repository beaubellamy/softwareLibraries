using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace TrainLibrary
{
    /// <summary>
    /// Summary description for Class1
    /// </summary>
    public class Processing
    {
        /* Mean radius of the Earth */
        public const double EarthRadius = 6371000.0;   // metres
        public const double TrainLength = 1.5;         // kilometres

        /* Distances */
        public const double metresToKilometers = 1.0 / 1000;
        public const double KilometersToMetres = 1000;

        /* Speeds */
        public const double kphToMetresPerMinute = 1000.0 / 60;
        public const double kphToMetresPerSec = 1000.0 / 3600;

        /* Times */
        public const double secToMinutes = 60;
        public const double secToHours = 3600;
        public const double MinutesToSec = 1.0 / 60;
        public const double HoursToMinutes = 1.0 / 60;
        public const double HoursToSeconds = 1.0 / 3600;


        public static TrackGeometry track = new TrackGeometry();

        /// <summary>
        /// Convert degrees in to radians
        /// </summary>
        /// <param name="degrees">Angle in degrees.</param>
        /// <returns>Angle in radians.</returns>
        public static double degress2radians(double degrees)
        {
            return degrees * Math.PI / 180;
        }

        /// <summary>
        /// Calculate the shortes distance between two geographical locations using the great circle formula.
        /// </summary>
        /// <param name="latitude1">Latitude of location 1.</param>
        /// <param name="longitude1">Longitude of location 1.</param>
        /// <param name="latitude2">Latitude of location 2.</param>
        /// <param name="longitude2">Longitude of location 2.</param>
        /// <returns>The Distance between the two points in metres.</returns>
        public static double calculateGreatCircleDistance(double latitude1, double longitude1, double latitude2, double longitude2)
        {

            double arcsine = Math.Sin(degress2radians((latitude2 - latitude1) / 2)) * Math.Sin(degress2radians((latitude2 - latitude1) / 2)) +
                Math.Cos(degress2radians(latitude1)) * Math.Cos(degress2radians(latitude2)) *
                Math.Sin(degress2radians((longitude2 - longitude1) / 2)) * Math.Sin(degress2radians((longitude2 - longitude1) / 2));
            double arclength = 2 * Math.Atan2(Math.Sqrt(arcsine), Math.Sqrt(1 - arcsine));

            return EarthRadius * arclength;

        }

        /// <summary>
        /// Calculate the shortes distance between two geographical locations using the great circle formula.
        /// </summary>
        /// <param name="point1">The geographic location of the first point.</param>
        /// <param name="point2">The geographic location of the second point.</param>
        /// <returns></returns>
        public static double calculateGreatCircleDistance(GeoLocation point1, GeoLocation point2)
        {

            double arcsine = Math.Sin(degress2radians((point2.latitude - point1.latitude) / 2)) * Math.Sin(degress2radians((point2.latitude - point1.latitude) / 2)) +
                Math.Cos(degress2radians(point1.latitude)) * Math.Cos(degress2radians(point2.latitude)) *
                Math.Sin(degress2radians((point2.longitude - point1.longitude) / 2)) * Math.Sin(degress2radians((point2.longitude - point1.longitude) / 2));
            double arclength = 2 * Math.Atan2(Math.Sqrt(arcsine), Math.Sqrt(1 - arcsine));

            return EarthRadius * arclength;

        }

        /// <summary>
        /// Determine if the single train journey has a single direction. When the journey has 
        /// multilpe directions, the part of the journey that has the largest total length in
        /// a single direction is returned.
        /// </summary>
        /// <param name="journey">The complete train journey.</param>
        /// <param name="trackGeometry">The geometry of the track.</param>
        /// <returns>A list of train details objects describing the longest distance the train has 
        /// travelled in a single direction.</returns>
        public static List<TrainJourney> longestDistanceTravelledInOneDirection(List<TrainJourney> journey, List<TrackGeometry> trackGeometry)
        {
            /* Set up intial conditions */
            double movingAverage = 0;
            double previousAverage = 0;
            double distance = 0;
            double increasingDistance = 0;
            double decreasingDistance = 0;

            int start, end;
            int newStart, count;
            double maxValue = 0;

            /* Create lists to add each journey for each change in direction. */
            List<double> distances = new List<double>();
            List<int> startIdx = new List<int>();
            List<int> endIdx = new List<int>();

            /* Set the number of points to average over. */
            int numPoints = 10;

            if (journey.Count <= numPoints)
                return journey;

            /* Set the kmPosts to the closest points on the geometry alignment. */
            track.matchTrainLocationToTrackGeometry(journey, trackGeometry);
            

            start = 0;

            for (int journeyIdx = 0; journeyIdx < journey.Count() - numPoints; journeyIdx++)
            {
                /* Calculate the moving average of the kmposts ahead of current position. */
                distance = journey[journeyIdx + numPoints].kmPost - journey[journeyIdx].kmPost;
                movingAverage = distance / numPoints;

                /* Check the direction has not changed. */
                if (Math.Sign(movingAverage) == Math.Sign(previousAverage) || Math.Sign(movingAverage) == 0 || Math.Sign(previousAverage) == 0)
                {
                    /* Increment the assumed distance travelled in current direction. */
                    if (movingAverage > 0)
                        increasingDistance = increasingDistance + movingAverage;

                    else if (movingAverage < 0)
                        decreasingDistance = decreasingDistance - movingAverage;

                }
                else
                {
                    /* There has been a change in direction. */
                    end = journeyIdx;

                    /* Add the total distance achieved from the previous km posts to the list. */
                    if (previousAverage > 0)
                    {
                        distances.Add(increasingDistance);
                        startIdx.Add(start);
                        endIdx.Add(end);
                        increasingDistance = 0;
                    }
                    else if (previousAverage < 0)
                    {
                        distances.Add(decreasingDistance);
                        startIdx.Add(start);
                        endIdx.Add(end);
                        decreasingDistance = 0;
                    }

                    /* Reset the new start postion. */
                    start = journeyIdx++;
                }

                previousAverage = movingAverage;

            }

            /* Add the last total distance achieved to the list. */
            end = journey.Count() - 1;
            if (previousAverage > 0)
            {
                distances.Add(increasingDistance);
                startIdx.Add(start);
                endIdx.Add(end);
            }
            else if (previousAverage < 0)
            {
                distances.Add(decreasingDistance);
                startIdx.Add(start);
                endIdx.Add(end);
            }
            else
            {
                /* Condition when last average is 0, determine which total to add to the list. */
                if (increasingDistance > decreasingDistance)
                {
                    distances.Add(increasingDistance);
                    startIdx.Add(start);
                    endIdx.Add(end);
                }
                else
                {
                    distances.Add(decreasingDistance);
                    startIdx.Add(start);
                    endIdx.Add(end);
                }
            }

            if (distances.Count() == 1)
                return journey;

            /* Determine the largest distance to return that section of the journey */
            maxValue = distances.Max();
            int index = distances.ToList().IndexOf(maxValue);
            newStart = startIdx[index];
            count = endIdx[index] - newStart + 1;

            /* Return the part of the journey that has the largest total length in a single direction. */
            return journey.GetRange(newStart, count);

        }

        /// <summary>
        /// Calcualte the single train journey length.
        /// </summary>
        /// <param name="journey">The journey points for the train.</param>
        /// <returns>The total distance travelled in metres.</returns>
        public static double calculateTrainJourneyDistance(List<TrainJourney> journey)
        {
            double distance = 0;

            for (int pointIdx = 1; pointIdx < journey.Count; pointIdx++)
            {
                /* Create the conequtive points */
                GeoLocation point1 = journey[pointIdx - 1].location;
                GeoLocation point2 = journey[pointIdx].location;

                /* Calcualte the great circle distance. */
                distance = distance + calculateGreatCircleDistance(point1, point2);
            }

            return distance;

        }

        /// <summary>
        /// Function determines the direction of the train using the first and last km posts.
        /// </summary>
        /// <param name="train">A train object containing kmPost information</param>
        /// <returns>Enumerated direction of the train km's.</returns>
        public static direction getTrainDirection(Train train)
        {
            /* NOTE: This function does not take into account any train journey data that have 
             * multiple changes of direction. This should not be seen when the 'Cleaned Data' 
             * is deleviered by Enetrprise services.
             * This is currently corrected for in longestDistanceTravelledInOneDirection()
             */

            /* Determine the distance and sign from the first point to the last point */
            double journeyDistance = train.journey[train.journey.Count() - 1].kmPost - train.journey[0].kmPost;

            if (journeyDistance > 0)
                return direction.IncreasingKm;
            else
                return direction.DecreasingKm;


        }

        /// <summary>
        /// Populate the geometry km information based on the calculated distance from the first km post.
        /// </summary>
        /// <param name="train">A train object.</param>
        public static void populateGeometryKm(List<TrainJourney> journey, List<TrackGeometry> trackGeometry)
        {
            /* Determine the direction of the km's the train is travelling. */
            GeoLocation trainPoint = new GeoLocation();

            /* The first km point is populated by the parent function ICEData.CleanData(). */
            for (int journeyIdx = 0; journeyIdx < journey.Count(); journeyIdx++)
            {
                /* Find the kilometerage of the closest point on the track and associate it with the current train location.*/
                trainPoint = new GeoLocation(journey[journeyIdx]);
                journey[journeyIdx].kilometreage = track.findClosestTrackGeometryPoint(trackGeometry, trainPoint);
                /* This method reduces any error in the actual klometreage when
                 * calculating the straight line distance over a few kilometres.
                 */
            }


        }

        /// <summary>
        /// Populate the loop location information for each point in the train journey.
        /// </summary>
        /// <param name="train">A train object containing the journey details.</param>
        /// <param name="trackGeometry">The track geometry object indicating the location of the loops.</param>
        public static void populateLoopLocations(List<TrainJourney> trainJourney, List<TrackGeometry> trackGeometry)
        {
            /* Create a track geometry object. */
            TrackGeometry track = new TrackGeometry();
            int index = 0;
            double trainPoint = 0;

            /* Cycle through the train journey. */
            foreach (TrainJourney journey in trainJourney)
            {
                trainPoint = journey.kilometreage;
                /* Find the index of the closest point on the track to the train. */
                index = track.findClosestTrackGeometryPoint(trackGeometry, trainPoint);
                /* Populate the loop */
                journey.isLoopHere = trackGeometry[index].isLoopHere;

            }

        }

        /// <summary>
        /// This function cycles through each train and determines if a TSR had applied to any part of the journey.
        /// </summary>
        /// <param name="trains">A list of trains containing the journey for each.</param>
        /// <param name="TSRs">A list of TSR objects.</param>
        public static void populateAllTrainsTemporarySpeedRestrictions(List<Train> trains, List<TSRObject> TSRs)
        {
            bool intermediateTSRFlag;

            foreach (Train train in trains)
            {
                foreach (TrainJourney journey in train.journey)
                {
                    /* Set the TSR flag for the next location. */
                    intermediateTSRFlag = false;

                    /* Establish the TSR that applies to the train position. */
                    for (int tsrIndex = 0; tsrIndex < TSRs.Count(); tsrIndex++)
                    {
                        /* Determine if the TSR is applicable to the train by location and date. */
                        if (journey.kilometreage >= TSRs[tsrIndex].startKm && journey.kilometreage <= TSRs[tsrIndex].endKm &&
                            journey.dateTime >= TSRs[tsrIndex].IssueDate && journey.dateTime <= TSRs[tsrIndex].LiftedDate)
                        {
                            intermediateTSRFlag = true;
                        }
                    }
                    journey.isTSRHere = intermediateTSRFlag;

                }
            }
        }

        /// <summary>
        /// Linear interpolation to a target point.
        /// </summary>
        /// <param name="targetX">Target invariant location to be interpolated to.</param>
        /// <param name="X0">Lower invariant position to interpolate between.</param>
        /// <param name="X1">Upper invariant position to interpolate between.</param>
        /// <param name="Y0">Lower variant to interpolate between.</param>
        /// <param name="Y1">Upper variant to interpolate between.</param>
        /// <returns>The interpolate variant value at the target invariant location.</returns>
        public static double linear(double targetX, double X0, double X1, double Y0, double Y1)
        {
            /* Take the average when the invariant location does not change. */
            if ((X1 - X0) == 0)
                return (Y0 + Y1) / 2;

            return Y0 + (targetX - X0) * (Y1 - Y0) / (X1 - X0);

        }

        /// <summary>
        ///  Interpolate the train speed to a specified interval using a linear interpolation.
        /// </summary>
        /// <param name="trains">List of train objects containing the parameters for each train journey.</param>
        /// <param name="trackGeometry">The list of Track geometry data to align the train location.</param>
        /// <returns>List of train objects with interpolated values at the specified interval.</returns>
        public static List<Train> interpolateTrainData(List<Train> trains, List<TrackGeometry> trackGeometry, 
            double startKm, double endKm, double interpolationInterval)
        {
            /* Placeholders for the interpolated distance markers. */
            double previousKm = 0;
            double currentKm = 0;
            /* Place holder to calculate the time for each interpolated value. */
            DateTime time = new DateTime();
            /* Flag to indicate when to collect the next time value. */
            bool timeChange = true;

            /* Additional loop and TSR details. */
            int geometryIdx = 0;
            bool loop = false;
            bool TSR = false;
            double TSRspeed = 0;

            /* Index values for the interpolation parameters */
            int index0 = -1;
            int index1 = -1;

            /* Interplation parameters. */
            double interpolatedSpeed = 0;
            double X0, X1, Y0, Y1;

            /* Create a new list of trains for the journies interpolated values. */
            List<Train> newTrainList = new List<Train>();

            /* Cycle through each train to interpolate between points. */
            for (int trainIdx = 0; trainIdx < trains.Count(); trainIdx++)
            {

                /* Create a new journey list of interpolated values. */
                List<TrainJourney> interpolatedJourney = new List<TrainJourney>();

                List<TrainJourney> journey = trains[trainIdx].journey;

                /* Set the start of the interpolation. */
                currentKm = startKm;
                previousKm = currentKm;

                while (currentKm < endKm)
                {

                    /* Find the closest kilometerage markers either side of the current interpolation point. */
                    index0 = findClosestLowerKm(currentKm, journey);
                    index1 = findClosestGreaterKm(currentKm, journey);

                    /* If a valid index is found, extract the existing journey parameters and interpolate. */
                    if (index0 >= 0 && index1 >= 0)
                    {
                        X0 = journey[index0].kilometreage;
                        X1 = journey[index1].kilometreage;
                        Y0 = journey[index0].speed;
                        Y1 = journey[index1].speed;
                        if (timeChange)
                        {
                            time = journey[index0].dateTime;
                            timeChange = false;
                        }

                        /* Perform linear interpolation. */
                        interpolatedSpeed = linear(currentKm, X0, X1, Y0, Y1);
                        /* Interpolate the time */
                        time = DateTime.FromOADate(Processing.linear(currentKm, X0, X1, journey[index0].dateTime.ToOADate(), journey[index1].dateTime.ToOADate()));

                        /* Interpolate the time. */
                        /* This creates inconsistencies when the train is accelerating from a stop, creating instances where the train appears to go back in time. */
                        //time = time.AddHours(Processing.calculateTimeInterval(previousKm, currentKm, interpolatedSpeed));
                       
                    }
                    else
                    {
                        /* Boundary conditions for interpolating the data prior to and beyond the existing journey points. */
                        time = journey.Where(t => t.dateTime > DateTime.MinValue).Min(t => t.dateTime);
                        interpolatedSpeed = 0;
                    }

                    geometryIdx = trackGeometry[0].findClosestTrackGeometryPoint(trackGeometry, currentKm);

                    if (geometryIdx >= 0)
                    {
                        /* Check if there is a loop at this location. */
                        loop = trackGeometry[geometryIdx].isLoopHere;

                        /* Check if there is a TSR at this location. */
                        TSR = trackGeometry[geometryIdx].isTSRHere;
                        TSRspeed = trackGeometry[geometryIdx].temporarySpeedRestriction;
                    }

                    /* Create the interpolated data object and add it to the list. */
                    TrainJourney item = new TrainJourney(time, interpolatedSpeed, currentKm, currentKm, trackGeometry[geometryIdx].elevation, loop, TSR);
                    interpolatedJourney.Add(item);

                    /* Create a copy of the current km marker and increment. */
                    previousKm = currentKm;
                    currentKm = currentKm + interpolationInterval * Processing.metresToKilometers;
                    
                    /* Determine if we need to extract the time from the data or interpolate it. */
                    if (index1 >= 0)
                        if (currentKm >= journey[index1].kilometreage)
                            timeChange = true;

                }

                /* Add the interpolated list to the list of new train objects. */
                Train trainItem = new Train(trains[trainIdx].Category, trains[trainIdx].trainID, trains[trainIdx].locoID,
                    trains[trainIdx].trainOperator, trains[trainIdx].commodity, trains[trainIdx].powerToWeight,
                    interpolatedJourney, trains[trainIdx].trainDirection);

                newTrainList.Add(trainItem);

            }

            /* Return the completed interpolated train data. */
            return newTrainList;
        }

        /// <summary>
        /// Calculate the aggregated average speed of all trains.
        /// </summary>
        /// <param name="trains">A list of trains to be aggregated in a single group.</param>
        /// <param name="CategorySim">The simulted train for the specified analysis Category.</param>
        /// <param name="trackGeometry">The track alignment information for the train journey.</param>
        /// <returns>An average train containing information about the average speed at each location.</returns>
        public AverageTrain averageTrain(List<Train> trains, List<TrainJourney> CategorySim, List<TrackGeometry> trackGeometry, 
            double startKm, double endKm, double interpolationInterval, double loopSpeedThreshold, double loopBoundaryThreshold, double TSRwindowBoundary)
        {

            bool loopBoundary = false;
            bool TSRBoundary = false;
            bool slowTrainsOrTSR = false;
            List<bool> TSRList = new List<bool>();
            List<bool> slowTrains = new List<bool>();
            double actualTime = 0;
            double simulatedTime = 0;

            /* Set up the average train journey lists. */
            List<double> kilometreage = new List<double>();
            List<double> elevation = new List<double>();
            List<double> averageSpeed = new List<double>();
            List<bool> isInLoopBoundary = new List<bool>();
            List<bool> isInTSRboundary = new List<bool>();
            List<bool> trackSlowTrains = new List<bool>();

            double kmPost = 0;
            double altitude = 0;
            List<double> speed = new List<double>();
            double sum = 0;
            double aveSpeed = 0;

            /* Determine the number of points in the average train journey. */
            int size = (int)((endKm - startKm) / (interpolationInterval * Processing.metresToKilometers));

            TrainJourney journey = new TrainJourney();

            /* Cycle through each location to average the valid values. */
            for (int journeyIdx = 0; journeyIdx < size; journeyIdx++)
            {
                /* Determine the current location and elevation of the alignemnt at this point. */
                kmPost = startKm + interpolationInterval * Processing.metresToKilometers * journeyIdx;
                altitude = trackGeometry[track.findClosestTrackGeometryPoint(trackGeometry, kmPost)].elevation;

                speed.Clear();
                TSRList.Clear();
                slowTrains.Clear();
                sum = 0;

                /* Cycle through each train in the list. */
                foreach (Train train in trains)
                {
                    journey = train.journey[journeyIdx];
                    /* Assume a loop boundary does not apply until we check. */
                    loopBoundary = false;

                    /* Does a TSR apply */
                    if (!withinTemporarySpeedRestrictionBoundaries(train, journey.kilometreage, startKm, endKm, TSRwindowBoundary))
                    {
                        TSRList.Add(false);
                        /* Is the train within a loop boundary */
                        if (!isTrainInLoopBoundary(train, journey.kilometreage, startKm, endKm, interpolationInterval, loopBoundaryThreshold))
                        {
                            slowTrains.Add(false);

                            speed.Add(journey.speed);
                            sum = sum + journey.speed;
                        }
                        else
                        {
                            loopBoundary = true;

                            if (journey.speed > (loopSpeedThreshold * CategorySim[journeyIdx].speed))
                            {
                                slowTrains.Add(false);

                                speed.Add(journey.speed);
                                sum = sum + journey.speed;
                            }
                            else
                            {
                                /* The train is too slow to include in the analysis. */
                                slowTrains.Add(true);
                            }
                        }

                    }
                    else
                    {
                        /* We dont want to include the speed in the aggregation if the train is within the
                         * bundaries of a TSR and is forced to slow down.  
                         */
                        TSRList.Add(true);
                        slowTrains.Add(true);

                        /* Keep track of the loop boundary for later inspection, if neccessary. */
                        if (isTrainInLoopBoundary(train, journey.kilometreage, startKm, endKm, interpolationInterval, loopBoundaryThreshold))
                            loopBoundary = true;

                    }

                }

                /* If all trains passing a loop were too slow, track this to allow applying the simulation speed at these locations. */
                if (slowTrains.Where(t => t == true).Count() == trains.Count())
                    slowTrainsOrTSR = true;
                else
                    slowTrainsOrTSR = false;

                /* If the TSR applied for the whole analysis period, the simulation speed is used. */
                if (TSRList.Where(t => t == true).Count() == trains.Count())
                {
                    aveSpeed = 0;
                    TSRBoundary = true;
                }
                else
                {
                    /* Calculate the average speed at each location. */
                    if (speed.Count() == 0 || sum == 0)
                        aveSpeed = 0;
                    else
                        aveSpeed = speed.Where(x => x > 0.0).Average();

                    TSRBoundary = false;
                }

                /* Add to each list for this location. */
                kilometreage.Add(kmPost);
                elevation.Add(altitude);
                averageSpeed.Add(aveSpeed);
                isInLoopBoundary.Add(loopBoundary);
                isInTSRboundary.Add(TSRBoundary);

                trackSlowTrains.Add(slowTrainsOrTSR);


                /* Accumulate the transit time. */
                if (!TSRBoundary || !slowTrainsOrTSR)
                {
                    if (aveSpeed > 0 && CategorySim[journeyIdx].speed > 0)
                    {
                        actualTime = actualTime + ((interpolationInterval * Processing.metresToKilometers) / aveSpeed);
                        simulatedTime = simulatedTime + ((interpolationInterval * Processing.metresToKilometers) / CategorySim[journeyIdx].speed);
                    }
                }


            }

            /* Calculate the pro-rata ratio for applying the simualted speeds. */
            double proRataTSRRatio = actualTime / simulatedTime;

            /* Make sure the ratio is less than 1. */
            if (proRataTSRRatio > 1)
                proRataTSRRatio = 1 / proRataTSRRatio;

            /* Re-assign the pro-rata speeds to the TSR locations or the slow train locations. */
            for (int index = 0; index < averageSpeed.Count(); index++)
            {
                if (isInTSRboundary[index] || trackSlowTrains[index])
                    averageSpeed[index] = proRataTSRRatio * CategorySim[index].speed;
            }

            /* Create the new average train object. */
            AverageTrain averageTrain = new AverageTrain(trains[0].Category, trains[0].trainDirection, trains.Count(), kilometreage, elevation, averageSpeed, isInLoopBoundary, isInTSRboundary);

            return averageTrain;

        }

        /// <summary>
        /// Calculate the weighted average of all simulations. This simulation is then used for 
        /// comparison when calculating the combined Categories (weighted average train).
        /// </summary>
        /// <param name="simulations">List of simulations for each Category analysed.</param>
        /// <param name="averageTrains">A List of the average train data, only used for the weighting.</param>
        /// <returns>A list of average train data describing the combined Categories.</returns>
        public static List<Train> getWeightedAverageSimulation(List<Train> simulations, List<AverageTrain> averageTrains)
        {
            /* The list of trains (2) that will be returned */
            List<Train> weightedAvergeTrain = new List<Train>();

            /* The denominator for the weighting calulations */
            int increasingTrainCount = averageTrains.Where(t => t.direction == direction.IncreasingKm).Select(t => t.trainCount).Sum();
            int decreasingTrainCount = averageTrains.Where(t => t.direction == direction.DecreasingKm).Select(t => t.trainCount).Sum();

            List<TrainJourney> increasingJourney = new List<TrainJourney>();
            List<TrainJourney> decreasingJourney = new List<TrainJourney>();

            double speedIncreasing = 0;
            double speedDecreasing = 0;

            /* If there are only two simulations, there is no need to calculate weighting */
            if (simulations.Count() == 2)
                return simulations;

            for (int journeyIdx = 0; journeyIdx < simulations[0].journey.Count(); journeyIdx++)
            {
                /* The journey for each simulation should be the same length, So loop 
                 * through the journey 
                 */
                if (simulations.Count() == 4)
                {
                    /* Assumes 2 individual Categories for increasing and decreasing directions. */
                    speedIncreasing = (simulations[0].journey[journeyIdx].speed * averageTrains[0].trainCount +
                        simulations[2].journey[journeyIdx].speed * averageTrains[2].trainCount) / increasingTrainCount;

                    speedDecreasing = (simulations[1].journey[journeyIdx].speed * averageTrains[1].trainCount +
                        simulations[3].journey[journeyIdx].speed * averageTrains[3].trainCount) / decreasingTrainCount;
                }
                else if (simulations.Count() == 6)
                {
                    /* Assumes 3 individual Categories for increasing and decreasing directions. */
                    speedIncreasing = (simulations[0].journey[journeyIdx].speed * averageTrains[0].trainCount +
                           simulations[2].journey[journeyIdx].speed * averageTrains[2].trainCount +
                           simulations[4].journey[journeyIdx].speed * averageTrains[4].trainCount) / increasingTrainCount;

                    speedDecreasing = (simulations[1].journey[journeyIdx].speed * averageTrains[1].trainCount +
                        simulations[3].journey[journeyIdx].speed * averageTrains[3].trainCount +
                        simulations[5].journey[journeyIdx].speed * averageTrains[5].trainCount) / decreasingTrainCount;
                }
                else
                {
                    /* Assumes 4 individual Categories for increasing and decreasing directions. */
                    speedIncreasing = (simulations[0].journey[journeyIdx].speed * averageTrains[0].trainCount +
                           simulations[2].journey[journeyIdx].speed * averageTrains[2].trainCount +
                           simulations[4].journey[journeyIdx].speed * averageTrains[4].trainCount +
                           simulations[6].journey[journeyIdx].speed * averageTrains[6].trainCount) / increasingTrainCount;

                    speedDecreasing = (simulations[1].journey[journeyIdx].speed * averageTrains[1].trainCount +
                        simulations[3].journey[journeyIdx].speed * averageTrains[3].trainCount +
                        simulations[5].journey[journeyIdx].speed * averageTrains[5].trainCount +
                        simulations[7].journey[journeyIdx].speed * averageTrains[7].trainCount) / decreasingTrainCount;
                }

                /* Assumed the same properties as the existing simulations */
                GeoLocation location = simulations[0].journey[journeyIdx].location;
                DateTime increasingTime = simulations[0].journey[journeyIdx].dateTime;
                DateTime decreasingTime = simulations[1].journey[journeyIdx].dateTime;
                double kilometreage = simulations[0].journey[journeyIdx].kilometreage;
                double elevation = simulations[0].journey[journeyIdx].elevation;

                /* Add to the journey for the increasing weighted average */
                TrainJourney itemIncreasing = new TrainJourney(location, increasingTime, speedIncreasing, kilometreage, kilometreage, elevation);
                increasingJourney.Add(itemIncreasing);

                /* Add to the journey for the decreasing weighted average */
                TrainJourney itemDecreasing = new TrainJourney(location, decreasingTime, speedDecreasing, kilometreage, kilometreage, elevation);
                decreasingJourney.Add(itemDecreasing);
            }

            /* Add the weighted average trains to the list */
            Train itemInc = new Train(increasingJourney, Category.Simulated, direction.IncreasingKm);
            weightedAvergeTrain.Add(itemInc);
            Train itemDec = new Train(decreasingJourney, Category.Simulated, direction.DecreasingKm);
            weightedAvergeTrain.Add(itemDec);

            return weightedAvergeTrain;
        }

        /// <summary>
        /// Determine if the train is approaching, leaving or within a loop.
        /// </summary>
        /// <param name="train">The train object containing the journey details.</param>
        /// <param name="targetLocation">The specific location being considered.</param>
        /// <returns>True, if the train is within the boundaries of the loop window.</returns>
        public bool isTrainInLoopBoundary(Train train, double targetLocation, double startKm, double endKm, double interpolationInterval, double loopBoundaryThreshold)
        {
            /* Find the indecies of the boundaries of the loop. */
            double lookBack = targetLocation - loopBoundaryThreshold;
            double lookForward = targetLocation + loopBoundaryThreshold;
            int lookBackIdx = train.indexOfGeometryKm(train.journey, lookBack);
            int lookForwardIdx = train.indexOfGeometryKm(train.journey, lookForward);

            /* Check the indecies are valid */
            if (lookBack < startKm && lookBackIdx == -1)
                lookBackIdx = 0;
            if (lookForward > endKm && lookForwardIdx == -1)
            {
                if (train.trainDirection == direction.IncreasingKm)
                    lookForwardIdx = train.journey.Count() - 1;
                else
                    lookForwardIdx = 0;
            }

            /* Determine if a loop is within the loop window of the current position. */
            if (lookBackIdx >= 0 && lookForwardIdx >= 0)
            {
                for (int journeyIdx = lookBackIdx; journeyIdx < lookForwardIdx; journeyIdx++)
                {
                    TrainJourney journey = train.journey[journeyIdx];

                    if (journey.isLoopHere)
                        return true;

                }
            }
            return false;
        }

        /// <summary>
        /// Determine the properties of the TSR if one applies.
        /// </summary>
        /// <param name="train">The train object containing the journey details.</param>
        /// <param name="targetLocation">The specific location being considered.</param>
        /// <returns>TSR object containting the TSR flag and the associated speed. </returns>
        public bool withinTemporarySpeedRestrictionBoundaries(Train train, double targetLocation, double startKm, double endKm, double TSRwindowBoundary)
        {

            bool isTSRHere = false;

            /* Find the indecies of the boundaries of the loop. */
            double lookBack = targetLocation - TSRwindowBoundary;
            double lookForward = targetLocation + TSRwindowBoundary;
            /* Add the train length to the forward direction to mimic the fact that 
             * the train can not start to accelerate until it has cleared the boundary. 
             */
            if (train.trainDirection == direction.IncreasingKm)
                lookForward += TrainLength;
            else
                lookForward -= TrainLength;

            int lookBackIdx = train.indexOfGeometryKm(train.journey, lookBack);
            int lookForwardIdx = train.indexOfGeometryKm(train.journey, lookForward);

            /* Check the indecies are valid */
            if (lookBack < startKm && lookBackIdx == -1)
                lookBackIdx = 0;
            if (lookForward > endKm && lookForwardIdx == -1)
                lookForwardIdx = train.journey.Count() - 1;

            /* Determine if a loop is within the loop window of the current position. */
            if (lookBackIdx >= 0 && lookForwardIdx >= 0)
            {
                for (int journeyIdx = lookBackIdx; journeyIdx < lookForwardIdx; journeyIdx++)
                {
                    TrainJourney journey = train.journey[journeyIdx];

                    if (journey.isTSRHere)
                        isTSRHere = true;
                }
            }
            return isTSRHere;
        }

        /// <summary>
        /// Find the index of the closest kilometerage that is less than the target point.
        /// </summary>
        /// <param name="target">The target kilometerage.</param>
        /// <param name="journey">The list of train details containig the journey parameters.</param>
        /// <returns>The index of the closest point that is less than the target point. 
        /// Returns -1 if a point does not exist.</returns>
        public static int findClosestLowerKm(double target, List<TrainJourney> journey)
        {
            /* Set the initial values. */
            double minimum = double.MaxValue;
            double difference = double.MaxValue;
            int index = 0;

            /* Cycle through the journey parameters. */
            for (int journeyIdx = 0; journeyIdx < journey.Count(); journeyIdx++)
            {
                /* Find the difference if the value is lower. */
                if (journey[journeyIdx].kilometreage < target)
                    difference = Math.Abs(journey[journeyIdx].kilometreage - target);

                /* Find the minimum difference. */
                if (difference < minimum)
                {
                    minimum = difference;
                    index = journeyIdx;
                }

            }

            if (difference == double.MaxValue)
                return -1;

            return index;
        }

        /// <summary>
        /// Find the index of the closest kilometerage that is larger than the target point.
        /// </summary>
        /// <param name="target">The target kilometerage.</param>
        /// <param name="journey">The list of train details containig the journey parameters.</param>
        /// <returns>The index of the closest point that is larger than the target point. 
        /// Returns -1 if a point does not exist.</returns>
        public static int findClosestGreaterKm(double target, List<TrainJourney> journey)
        {
            /* Set the initial values. */
            double minimum = double.MaxValue;
            double difference = double.MaxValue;
            int index = 0;

            /* Cycle through the journey parameters. */
            for (int journeyIdx = 0; journeyIdx < journey.Count(); journeyIdx++)
            {
                /* Find the difference if the value is lower. */
                if (journey[journeyIdx].kilometreage > target)
                    difference = Math.Abs(journey[journeyIdx].kilometreage - target);

                /* Find the minimum difference. */
                if (difference < minimum)
                {
                    minimum = difference;
                    index = journeyIdx;
                }
            }

            if (difference == double.MaxValue)
                return -1;

            return index;
        }

        /// <summary>
        /// Calculate the time interval between two locations based on the speed.
        /// </summary>
        /// <param name="startPositon">Starting kilometreage.</param>
        /// <param name="endPosition">Final kilometreage.</param>
        /// <param name="speed">Average speed between locations.</param>
        /// <returns>The time taken to traverse the distance in hours.</returns>
        public static double calculateTimeInterval(double startPositon, double endPosition, double speed)
        {

            if (speed > 0)
                return Math.Abs(endPosition - startPositon) / speed;    // hours.
            else
                return 0;
        }

        /// <summary>
        /// This function cleans the data from large gaps in the data and ensures the trains 
        /// are all travelling in a single direction with a minimum total distance.
        /// </summary>
        /// <param name="record">List of Train record objects</param>
        /// <param name="trackGeometry">A list of track Geometry objects</param>
        /// <returns>List of Train objects containing the journey details of each train.</returns>
        public static List<Train> CleanData(List<TrainRecord> record, List<TrackGeometry> trackGeometry,
            double timeThreshold, double distanceThreshold, double minimumJourneyDistance, analysisCategory analysisCategory,
            double Category1LowerBound = 0,double Category1UpperBound = 1,double Category2LowerBound = 1,double Category2UpperBound = 2)
        {
            /* Note: this function will not be needed when Enterprise Services delivers the interpolated 
             * date directly to the database. We can access this data directly, then analyse.
             */

            bool removeTrain = false;
            double distance = 0;
            double journeyDistance = 0;

            /* Create the lists for the processed train data. */
            List<Train> cleanTrainList = new List<Train>();
            List<TrainJourney> journey = new List<TrainJourney>();

            GeoLocation point1 = null;
            GeoLocation point2 = null;

            /* Add the first point to the train journey. */
            journey.Add(new TrainJourney(record[0]));

            for (int trainIndex = 1; trainIndex < record.Count(); trainIndex++)
            {
                /* Compare next train details with current train details to establish if its a new train. */
                if (record[trainIndex].trainID.Equals(record[trainIndex - 1].trainID) &&
                    record[trainIndex].locoID.Equals(record[trainIndex - 1].locoID) &&
                    (record[trainIndex].dateTime - record[trainIndex - 1].dateTime).TotalMinutes < timeThreshold)
                {

                    /* If the current and previous record represent the same train journey, add it to the list. */
                    journey.Add(new TrainJourney(record[trainIndex]));

                    point1 = new GeoLocation(record[trainIndex - 1]);
                    point2 = new GeoLocation(record[trainIndex]);

                    distance = calculateGreatCircleDistance(point1, point2);

                    if (distance > distanceThreshold)
                    {
                        /* If the distance between successive km points is greater than the
                         * threshold then we want to remove this train from the data. 
                         */
                        removeTrain = true;
                    }

                }
                else
                {
                    /* Check uni directionality of the train */
                    journey = longestDistanceTravelledInOneDirection(journey, trackGeometry);
                    /* Calculate the total length of the journey */
                    journeyDistance = calculateTrainJourneyDistance(journey);

                    /* Populate the train parameters. */
                    Train item = new Train();
                    item.journey = journey;
                    item.trainDirection = getTrainDirection(item);

                    /* remove the train if the direction is not valid. */
                    if (item.trainDirection == direction.Invalid)
                        removeTrain = true;

                    /* The end of the train journey has been reached. */
                    if (!removeTrain && journeyDistance > minimumJourneyDistance)
                    {
                        /* If all points are acceptable and the train travels the minimum distance, 
                         * add the train journey to the cleaned list. 
                         */
                        item.trainID = record[trainIndex - 1].trainID;
                        item.locoID = record[trainIndex - 1].locoID;
                        item.trainOperator = record[trainIndex - 1].trainOperator;
                        item.commodity = record[trainIndex - 1].commodity;
                        item.powerToWeight = record[trainIndex - 1].powerToWeight;

                        /* Determine the analysis Category. */
                        if (analysisCategory == analysisCategory.TrainPowerToWeight)
                        {
                            if (item.powerToWeight > Category1LowerBound && item.powerToWeight <= Category1UpperBound)
                                item.Category = Category.Underpowered;
                            else if (item.powerToWeight > Category2LowerBound && item.powerToWeight <= Category2UpperBound)
                                item.Category = Category.Overpowered;
                            else
                                item.Category = Category.Actual;

                        }
                        else if (analysisCategory == analysisCategory.TrainOperator)
                        {
                            item.Category = convertTrainOperatorToCategory(item.trainOperator);
                        }
                        else
                        {
                            item.Category = convertCommodityToCategory(item.commodity);
                        }


                        /* Determine the actual km, and populate the loops and TSR information. */
                        populateGeometryKm(item.journey, trackGeometry);
                        populateLoopLocations(item.journey, trackGeometry);

                        /* Sort the journey in ascending order. */
                        item.journey = item.journey.OrderBy(t => t.kilometreage).ToList();

                        cleanTrainList.Add(item);

                    }

                    /* Reset the parameters for the next train. */
                    removeTrain = false;
                    journeyDistance = 0;
                    journey.Clear();

                    /* Add the first record of the new train journey. */
                    journey.Add(new TrainJourney(record[trainIndex]));

                }

                /* The end of the records have been reached. */
                if (trainIndex == record.Count() - 1 && !removeTrain)
                {
                    /* Check uni directionality of the last train */
                    journey = longestDistanceTravelledInOneDirection(journey, trackGeometry);
                    /* Calculate the total length of the journey */
                    journeyDistance = calculateTrainJourneyDistance(journey);

                    /* Populate the train parameters. */
                    Train lastItem = new Train();
                    lastItem.journey = journey;
                    lastItem.trainDirection = getTrainDirection(lastItem);

                    /* remove the train if the direction is not valid. */
                    if (lastItem.trainDirection == direction.Invalid)
                        removeTrain = true;

                    if (!removeTrain && journeyDistance > minimumJourneyDistance)
                    {
                        lastItem.trainID = record[trainIndex - 1].trainID;
                        lastItem.locoID = record[trainIndex - 1].locoID;
                        lastItem.trainOperator = record[trainIndex - 1].trainOperator;
                        lastItem.commodity = record[trainIndex - 1].commodity;
                        lastItem.powerToWeight = record[trainIndex - 1].powerToWeight;

                        /* Determine the analysis Category. */
                        if (analysisCategory == analysisCategory.TrainPowerToWeight)
                        {
                            if (lastItem.powerToWeight > Category1LowerBound && lastItem.powerToWeight <= Category1UpperBound)
                                lastItem.Category = Category.Underpowered;
                            else if (lastItem.powerToWeight > Category2LowerBound && lastItem.powerToWeight <= Category2UpperBound)
                                lastItem.Category = Category.Overpowered;
                            else
                                lastItem.Category = Category.Actual;

                        }
                        else if (analysisCategory == analysisCategory.TrainOperator)
                        {
                            lastItem.Category = convertTrainOperatorToCategory(lastItem.trainOperator);
                        }
                        else
                        {
                            lastItem.Category = convertCommodityToCategory(lastItem.commodity);
                        }

                        /* If all points are aceptable, add the train journey to the cleaned list. */
                        populateGeometryKm(lastItem.journey, trackGeometry);
                        populateLoopLocations(lastItem.journey, trackGeometry);

                        /* Sort the journey in ascending order. */
                        lastItem.journey = lastItem.journey.OrderBy(t => t.kilometreage).ToList();

                        cleanTrainList.Add(lastItem);
                    }

                }

            }
           
            return cleanTrainList;

        }

        /// <summary>
        /// This function creates the individual train journeies and adds them to the list. The 
        /// function ensures the train is consistent and has a single direction of travel.
        /// </summary>
        /// <param name="record">List of Train record objects</param>
        /// <param name="trackGeometry">A list of track Geometry objects</param>
        /// <returns>List of Train objects containing the journey details of each train.</returns>
        public static List<Train> MakeTrains(List<TrainRecord> record, List<TrackGeometry> trackGeometry,
            double timeThreshold, double distanceThreshold, double minimumJourneyDistance, analysisCategory analysisCategory,
            double Category1LowerBound = 0,double Category1UpperBound = 1,double Category2LowerBound = 1,double Category2UpperBound = 2)
        {
            /* Note: this function is designed to replace the cleanTrains function when the 
             * interpolated data is delivered by Enterprise Services.
             */

            /* Create the lists for the processed train data. */
            List<Train> TrainList = new List<Train>();
            List<TrainJourney> journey = new List<TrainJourney>();

            /* Add the first point to the train journey. */
            journey.Add(new TrainJourney(record[0]));

            for (int trainIndex = 1; trainIndex < record.Count(); trainIndex++)
            {
                /* Compare next train details with current train details to establish if its a new train. */
                if (record[trainIndex].trainID.Equals(record[trainIndex - 1].trainID) &&
                    record[trainIndex].locoID.Equals(record[trainIndex - 1].locoID) &&
                    (record[trainIndex].dateTime - record[trainIndex - 1].dateTime).TotalMinutes < timeThreshold)
                {

                    /* If the current and previous record represent the same train journey, add it to the list. */
                    journey.Add(new TrainJourney(record[trainIndex]));

                }
                else
                {
                    /* The end of the train journey has been reached. */

                    /* Check uni directionality of the train */
                    journey = longestDistanceTravelledInOneDirection(journey, trackGeometry);

                    /* Assign the train parameters. */
                    Train item = new Train();
                    item.journey = journey;
                    item.trainDirection = getTrainDirection(item);
                    item.trainID = record[trainIndex - 1].trainID;
                    item.locoID = record[trainIndex - 1].locoID;
                    item.trainOperator = record[trainIndex - 1].trainOperator;
                    item.commodity = record[trainIndex - 1].commodity;
                    item.powerToWeight = record[trainIndex - 1].powerToWeight;

                    /* Determine the train Category. */
                    if (analysisCategory == analysisCategory.TrainPowerToWeight)
                    {
                        if (item.powerToWeight > Category1LowerBound && item.powerToWeight <= Category1UpperBound)
                            item.Category = Category.Underpowered;
                        else if (item.powerToWeight > Category2LowerBound && item.powerToWeight <= Category2UpperBound)
                            item.Category = Category.Overpowered;
                        else
                            item.Category = Category.Actual;

                    }
                    else if (analysisCategory == analysisCategory.TrainOperator)
                    {
                        item.Category = convertTrainOperatorToCategory(item.trainOperator);
                    }
                    else
                    {
                        item.Category = convertCommodityToCategory(item.commodity);
                    }


                    /* Determine the actual km, and populate the loops information. */
                    populateGeometryKm(item.journey, trackGeometry);
                    populateLoopLocations(item.journey, trackGeometry);

                    /* Sort the journey in ascending order. */
                    item.journey = item.journey.OrderBy(t => t.kilometreage).ToList();

                    TrainList.Add(item);

                    /* Reset the parameters for the next train. */
                    journey.Clear();

                    /* Add the first record of the new train journey. */
                    journey.Add(new TrainJourney(record[trainIndex]));

                }

                if (trainIndex == record.Count() - 1)
                {
                    /* The end of the records have been reached. */

                    /* Check uni directionality of the last train */
                    journey = longestDistanceTravelledInOneDirection(journey, trackGeometry);

                    /*  Assign the train parameters. */
                    Train lastItem = new Train();

                    lastItem.journey = journey;
                    lastItem.trainDirection = getTrainDirection(lastItem);

                    lastItem.trainID = record[trainIndex - 1].trainID;
                    lastItem.locoID = record[trainIndex - 1].locoID;
                    lastItem.trainOperator = record[trainIndex - 1].trainOperator;
                    lastItem.commodity = record[trainIndex - 1].commodity;
                    lastItem.powerToWeight = record[trainIndex - 1].powerToWeight;

                    /* Determine the train Category. */
                    if (analysisCategory == analysisCategory.TrainPowerToWeight)
                    {
                        if (lastItem.powerToWeight > Category1LowerBound && lastItem.powerToWeight <= Category1UpperBound)
                            lastItem.Category = Category.Underpowered;
                        else if (lastItem.powerToWeight > Category2LowerBound && lastItem.powerToWeight <= Category2UpperBound)
                            lastItem.Category = Category.Overpowered;
                        else
                            lastItem.Category = Category.Actual;

                    }
                    else if (analysisCategory == analysisCategory.TrainOperator)
                    {
                        lastItem.Category = convertTrainOperatorToCategory(lastItem.trainOperator);
                    }
                    else
                    {
                        lastItem.Category = convertCommodityToCategory(lastItem.commodity);
                    }

                    /* Determine the actual km, and populate the loops information.  */
                    populateGeometryKm(lastItem.journey, trackGeometry);
                    populateLoopLocations(lastItem.journey, trackGeometry);

                    /* Sort the journey in ascending order. */
                    lastItem.journey = lastItem.journey.OrderBy(t => t.kilometreage).ToList();

                    TrainList.Add(lastItem);
                }
            }

            return TrainList;
        }

        /// <summary>
        /// Convert The analysis Category to the train Operator.
        /// </summary>
        /// <param name="Category">The analsyis Category.</param>
        /// <returns>The train operator corresponding to the analysis Category.</returns>
        public static trainOperator convertCategoryToTrainOperator(Category Category)
        {
            trainOperator trainOperator = trainOperator.Unknown;

            /* Extract the list of train operators. */
            List<trainOperator> operatorList = Enum.GetValues(typeof(trainOperator)).Cast<trainOperator>().ToList();

            /* Match the opertor to the Category. */
            foreach (trainOperator Operator in operatorList)
            {
                if (Operator.ToString().Equals(Category.ToString()))
                    trainOperator = Operator;
            }

            return trainOperator;
        }

        /// <summary>
        /// Convert The train Category to the train commodity.
        /// </summary>
        /// <param name="Category">The analsyis Category.</param>
        /// <returns>The train commodity corresponding to the analysis Category.</returns>
        public static trainCommodity convertCategoryToCommodity(Category Category)
        {
            trainCommodity trainCommodity = trainCommodity.Unknown;

            /* Extract the list of train operators. */
            List<trainCommodity> commodityList = Enum.GetValues(typeof(trainCommodity)).Cast<trainCommodity>().ToList();

            /* Match the opertor to the Category. */
            foreach (trainCommodity commodity in commodityList)
            {
                if (commodity.ToString().Equals(Category.ToString()))
                    trainCommodity = commodity;
            }

            return trainCommodity;
        }

        /// <summary>
        /// Convert the train operator to the analysis Category.
        /// </summary>
        /// <param name="trainOperator">The train operator.</param>
        /// <returns>The analysis Category corresponding to the train operator.</returns>
        public static Category convertTrainOperatorToCategory(trainOperator trainOperator)
        {
            Category trainCategory = Category.Unknown;

            /* Extract the list of Categories. */
            List<Category> CategoryList = Enum.GetValues(typeof(Category)).Cast<Category>().ToList();

            /* Match the Category to the opertor. */
            foreach (Category cat in CategoryList)
            {
                if (cat.ToString().Equals(trainOperator.ToString()))
                    trainCategory = cat;
            }

            return trainCategory;
        }

        /// <summary>
        /// Convert the train commodity to the analysis Category.
        /// </summary>
        /// <param name="trainOperator">The train operator.</param>
        /// <returns>The analysis Category corresponding to the train operator.</returns>
        public static Category convertCommodityToCategory(trainCommodity commodity)
        {
            Category trainCategory = Category.Unknown;

            /* Extract the list of Categories. */
            List<Category> CategoryList = Enum.GetValues(typeof(Category)).Cast<Category>().ToList();

            /* Match the Category to the opertor. */
            foreach (Category cat in CategoryList)
            {
                if (cat.ToString().Equals(commodity.ToString()))
                    trainCategory = cat;
            }

            return trainCategory;
        }

        /// <summary>
        /// Convert the opertor string to the trainOperator class - INCOMPLETE
        /// </summary>
        /// <param name="Operator">A 3 character string designation for the traqin operator.</param>
        /// <returns>A trainOperator object.</returns>
        public static trainOperator getWagonOperator(string Operator)
        {
            /* Still to be completed. */
            // ASR, AVA, AWR, FAL,FLK, NRC, SBR - Not sure what operators these are?
            string[] ARTC = { };
            string[] Aurizon = { "QR" }; 
            string[] AustralianRailwaysHistoricalSociety = { }; 
            string[] CityRail = { }; 
            string[] Countrylink = { }; 
            string[] Freightliner = { }; 
            string[] GenesseeWyoming = { "GWI" }; 
            string[] GreatSouthernRail = { }; 
            string[] Interail = { "INT" };
            string[] JohnHollandRail = { }; 
            string[] LauchlanValleyRailSociety = { }; 
            string[] PacificNational = { "PAT", "PNC", "PND", "PNL", "PNT" }; 
            string[] QUBE = { }; 
            string[] RailTransportMuseum = { }; 
            string[] RailCorp = { }; 
            string[] SCT = { "SCT" }; 
            string[] SouthernShorthaulRail = { };
            string[] SydneyRailService = { }; 
            string[] TheRailMotorService = { }; 
            string[] VLinePassenger = { };

            if (ARTC.Contains(Operator))
                return trainOperator.ARTC;
            else if (Aurizon.Contains(Operator))
                return trainOperator.Aurizon;
            else if (AustralianRailwaysHistoricalSociety.Contains(Operator))
                return trainOperator.AustralianRailwaysHistoricalSociety;
            else if (CityRail.Contains(Operator))
                return trainOperator.CityRail;
            else if (Countrylink.Contains(Operator))
                return trainOperator.Countrylink;
            else if (Freightliner.Contains(Operator))
                return trainOperator.Freightliner;
            else if (GenesseeWyoming.Contains(Operator))
                return trainOperator.GenesseeWyoming;
            else if (GreatSouthernRail.Contains(Operator))
                return trainOperator.GreatSouthernRail;
            else if (Interail.Contains(Operator))
                return trainOperator.Interail;
            else if (JohnHollandRail.Contains(Operator))
                return trainOperator.JohnHollandRail;
            else if (LauchlanValleyRailSociety.Contains(Operator))
                return trainOperator.LauchlanValleyRailSociety;
            else if (PacificNational.Contains(Operator))
                return trainOperator.PacificNational;
            else if (QUBE.Contains(Operator))
                return trainOperator.QUBE;
            else if (RailTransportMuseum.Contains(Operator))
                return trainOperator.RailTransportMuseum;
            else if (RailCorp.Contains(Operator))
                return trainOperator.RailCorp;
            else if (SCT.Contains(Operator))
                return trainOperator.SCT;
            else if (SouthernShorthaulRail.Contains(Operator))
                return trainOperator.SouthernShorthaulRail;
            else if (SydneyRailService.Contains(Operator))
                return trainOperator.SydneyRailService;
            else if (TheRailMotorService.Contains(Operator))
                return trainOperator.TheRailMotorService;
            else if (VLinePassenger.Contains(Operator))
                return trainOperator.VLinePassenger;
            else
                return trainOperator.Unknown;

        }

        /// <summary>
        /// Convert the commodity string to the commodity object - INCOMPLETE
        /// </summary>
        /// <param name="commodity">A 3 character string designation for the commodity.</param>
        /// <returns>A trainCommodity object.</returns>
        public static trainCommodity getWagonCommodity(string commodity)
        {
            /* Still to be completed. */
            // GDS, SUP - Not sure what commodities these are.
            string[] Clinker = { "CLS" };
            string[] Coal = { "CLE" };
            string[] Freight = { "FRE", "GEN" };
            string[] Express = { "EXP" };
            string[] Grain = { "GRN" };
            string[] Intermodal = { "INT" };
            string[] Minerals = { "MIN" };
            string[] Passenger = { "PAS" };
            string[] Shuttle = { "SHT" };
            string[] Steel = { "STL" };
            string[] Work = { "WRk" };
            
            if (Clinker.Contains(commodity))
                return trainCommodity.Clinker;
            else if (Coal.Contains(commodity))
                return trainCommodity.Coal;
            else if (Freight.Contains(commodity))
                return trainCommodity.GeneralFreight;
            else if (Express.Contains(commodity))
                return trainCommodity.Express;
            else if (Grain.Contains(commodity))
                return trainCommodity.Grain;
            else if (Intermodal.Contains(commodity))
                return trainCommodity.Intermodal;
            else if (Minerals.Contains(commodity))
                return trainCommodity.Mineral;
            else if (Passenger.Contains(commodity))
                return trainCommodity.Passenger;
            else if (Shuttle.Contains(commodity))
                return trainCommodity.Shuttle;
            else if (Steel.Contains(commodity))
                return trainCommodity.Steel;
            else if (Work.Contains(commodity))
                return trainCommodity.Work;
            else
                return trainCommodity.Unknown;

        }



    } // Class Processing
}
