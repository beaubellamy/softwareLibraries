﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RGiesecke.DllExport;
using System.Runtime.InteropServices;

namespace TrainLibrary
{
    /*
     * Enumerated types to assit in giving the train classes appropriate 
     * descriptions for detailed analysis.
     */

    /// <summary>
    /// A list of available analysis seperation Categories. 
    /// </summary>
    public enum analysisCategory { TrainOperator, TrainCommodity, TrainType, TrainPowerToWeight, Unknown };

    /// <summary>
    /// Data Source type
    /// </summary>
    public enum dataSource { StandardIceData, AzureData, AzureExtractData };

    /// <summary>
    /// Enumerated direction of the train km's.
    /// </summary>
    public enum direction { IncreasingKm, DecreasingKm, Invalid, Unknown };

    /// <summary>
    /// A List of valid train operators.
    /// </summary>
    public enum trainOperator
    {
        ARCInfrastructure, ARTC, Aurizon, AustralianRailGroup, AustralianRailwaysHistoricalSociety, AustralianTransportNetwork, AvailableRollingStock, 
        CityRail, Countrylink, ElZorroTransport, Freightliner, GenesseeWyoming, GreatSouthernRail, Interail, JohnHollandRail, LauchlanValleyRailSociety, Limited3801, 
        MetroTrainsMelbourne, PacificNational, QUBE, QueenslandRail, RailTransportMuseum, RailCorp, SCT, SouthernShorthaulRail, SpecialistBulkRail,
        SydneyRailService, TheRailMotorService, Transport4NSW, VLinePassenger, GroupRemaining, Combined, Simulated, Unknown
    };

   
    /// <summary>
    /// A list of available commodities.
    /// </summary>
    public enum trainCommodity
    {
        Clinker, Coal, Express, GeneralFreight, Grain, Goods, Intermodal, Interstate, Mineral, Passenger, Shuttle, Shunt, Steel, TrailerRail, Work,
        GroupRemaining, Unknown
    };

    /// <summary>
    /// A list of available train types.
    /// </summary>
    public enum trainType
    {
        AdelaideMelbourne, AdelaidePerth, AdelaideSydney, AdeliadeBrisbane, BrisbaneMelbourne, BrisbanePerth, BrisbaneSydney, GP, PX,
        MelbournePerth, MelbourneSydney, PerthSydney, NonStandard, Simulated, GroupRemaining, Unknown,

        AP1, AP2, AP8, GP1, MP1, MP2, MP4, MP5, MP7, MP9, MB4, BM4, BM7, MB7, SB1,
        PA8, PG1, PM1, PM4, PM5, PM6, PM7, PM9, PS5, PS6, PS7, PX4, SP5, SP7
    };

    /// <summary>
    /// A list of analysis Categories, comprising of train operators, power to weight ratios.
    /// {TrainOperator List, TrainCommodity, TrainType, power to weight Categories}
    /// </summary>
    public enum Category
    {
        /* Train Operators. */
        ARCInfrastructure, ARTC, Aurizon, AustralianRailGroup, AustralianRailwaysHistoricalSociety, AustralianTransportNetwork, AvailableRollingStock, 
        CityRail, Countrylink, ElZorroTransport, Freightliner, GenesseeWyoming, GreatSouthernRail, Interail, JohnHollandRail, LauchlanValleyRailSociety, Limited3801, 
        MetroTrainsMelbourne, PacificNational, QUBE, QueenslandRail, RailTransportMuseum, RailCorp, SCT, SouthernShorthaulRail, SpecialistBulkRail,
        SydneyRailService, TheRailMotorService, Transport4NSW, VLinePassenger,  
        /* Commodities. */
        GeneralFreight, Coal, Express, Grain, Goods, Mineral, Shuttle, Steel, Shunt, Clinker, Intermodal, Passenger, Interstate, TrailerRailGrain, Work,
        /* Train Types */
        AdelaideMelbourne, AdelaidePerth, AdelaideSydney, AdeliadeBrisbane, BrisbaneMelbourne, BrisbanePerth, BrisbaneSydney, GP, PX,
        MelbournePerth, MelbourneSydney, PerthSydney,
        /* Power to weight catagories. */
        Underpowered, Overpowered, Alternative,
        /* Other */
        NonStandard, GroupRemaining, Combined, Actual, Simulated, Unknown,

        AP1, AP2, AP8, GP1, MP1, MP2, MP4, MP5, MP7, MP9, MB4, BM4, BM7, MB7, SB1, 
        PA8, PG1, PM1, PM4, PM5, PM6, PM7, PM9, PS5, PS6, PS7, PX4, SP5, SP7
    };

    /*
     * Train Classes to store the ICE data, simulation data and the aggregated data
     */

    /// <summary>
    /// A Train class to describe each individual train.
    /// </summary>
    public class Train
    {
        public Category Category;
        public string trainID;
        public trainType trainType;
        public string locoID;
        public trainOperator trainOperator;
        public trainCommodity commodity;
        public double powerToWeight;
        public List<TrainJourney> journey;
        public direction trainDirection;
        public bool include;

        /// <summary>
        /// Overloaded Equal operator
        /// </summary>
        /// <param name="train1">The initial train object.</param>
        /// <param name="train2">The train object to compare with</param>
        /// <returns>Returns True if the objects are the same trains.</returns>
        public static bool operator ==(Train train1, Train train2)
        {
            if (ReferenceEquals(train1, train2))
                return true;
            
            if (ReferenceEquals(train1, null))
                return false;
            
            if (ReferenceEquals(train2, null))
                return false;
            
            /* Ignore the comparison of the train journey, when comparing trains to avoid the 
             * differences between an interpolated trains with gaps and interpolating 
             * through the gaps for the utilisation. */
            return (train1.Category == train2.Category &&
                train1.trainID == train2.trainID &&
                train1.trainType == train2.trainType &&
                train1.locoID == train2.locoID &&
                train1.trainOperator == train2.trainOperator &&
                train1.commodity == train2.commodity &&
                train1.powerToWeight == train2.powerToWeight &&
                train1.trainDirection == train2.trainDirection &&
                train1.include == train2.include);

        }

        /// <summary>
        /// Overloaded Not Equal operator
        /// </summary>
        /// <param name="train1">The initial train object.</param>
        /// <param name="train2">The train object to compare with</param>
        /// <returns>Returns True if the objects are the same trains.</returns>
        public static bool operator !=(Train train1, Train train2)
        {
            return !(train1 == train2);
        }

        /// <summary>
        /// Overloaded Equals method
        /// </summary>
        /// <param name="other">The train object to compare.</param>
        /// <returns>Returns True if the objects are the same trains.</returns>
        public bool Equals(Train other)
        {
            if (ReferenceEquals(null, other))
                return false;
            
            if (ReferenceEquals(this, other))
                return true;

            /* Ignore the comparison of the train journey, when comparing trains to avoid the 
             * differences between an interpoalted trains with gaps and interpoalting 
             * through the gaps for the utilisation. */
            return (Category.Equals(other.Category) &&
                trainID.Equals(other.trainID) &&
                trainType.Equals(other.trainType) &&
                locoID.Equals(other.locoID) &&
                trainOperator.Equals(other.trainOperator) &&
                commodity.Equals(other.commodity) &&
                powerToWeight.Equals(other.powerToWeight) &&
                trainDirection.Equals(other.trainDirection) &&
                include.Equals(other.include));
        }

        /// <summary>
        /// Override Equals method
        /// </summary>
        /// <param name="obj">System object to compare</param>
        /// <returns>Returns True if the objects are the same trains.</returns>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj))
                return false;
            
            if (ReferenceEquals(this, obj))
                return true;
            
            /* Ensure the objects are the same type and that they can be cast to the train object. */
            return obj.GetType() == GetType() && Equals((Train)obj);
        }

        /// <summary>
        /// Implementation of custom HashCode for the equals methods.
        /// </summary>
        /// <returns>returns a hashcode for the train opbject.</returns>
        public override int GetHashCode()
        {
            /* Return the hash codes for the relevant components of the train object. */
            return Category.GetHashCode() +
                trainID.GetHashCode() +
                trainType.GetHashCode() +
                locoID.GetHashCode() +
                trainOperator.GetHashCode() +
                commodity.GetHashCode() +
                powerToWeight.GetHashCode() +
                trainDirection.GetHashCode()+
                include.GetHashCode();

        }

        /// <summary>
        /// Default train constructor
        /// </summary>
        public Train()
        {
            this.Category = Category.Unknown;
            this.trainID = "none";
            this.trainType = trainType.Unknown;
            this.locoID = "none";
            this.trainOperator = trainOperator.Unknown;
            this.commodity = trainCommodity.Unknown;
            this.powerToWeight = 0;
            this.journey = new List<TrainJourney>();
            this.trainDirection = direction.Unknown;
            this.include = false;
        }

        /// <summary>
        /// Train constructor for a standard train read from data.
        /// </summary>
        /// <param name="Category">Analysis Category, described by operator or power to weight ratio,</param>
        /// <param name="trainId">The Train ID.</param>
        /// <param name="locoID">The locomotive ID.</param>
        /// <param name="trainOperator">Identification of the train operator.</param>
        /// <param name="commodity">identification of the commidity the train is carrying.</param>
        /// <param name="power">The power to weight ratio of the train.</param>
        /// <param name="journey">The list of journey details describing the points along the trains journey.</param>
        /// <param name="direction">The direction of travel indicated by the direction the kilometreage is progressing.</param>
        /// <param name="include">A flag indicating if the train is to be include in the analysis.</param>
        public Train(Category Category, string trainId, trainType trainType, string locoID, trainOperator trainOperator, trainCommodity commodity, double power, List<TrainJourney> journey, direction direction, bool include)
        {
            this.Category = Category;
            this.trainID = trainId;
            this.trainType = trainType;
            this.locoID = locoID;
            this.trainOperator = trainOperator;
            this.commodity = commodity;
            this.powerToWeight = power;
            this.journey = journey;
            this.trainDirection = direction;
            this.include = include;
        }

        /// <summary>
        /// Train constructor for the interpolated train data.
        /// </summary>
        /// <param name="Category">Analysis Category, described by operator or power to weight ratio,</param>
        /// <param name="trainId">The Train ID.</param>
        /// <param name="locoID">The locomotive ID.</param>
        /// <param name="trainOperator">Identification of the train operator.</param>
        /// <param name="commodity">identification of the commidity the train is carrying.</param>
        /// <param name="power">The power to weight ratio of the train.</param>
        /// <param name="journey">The list of journey details describing the points along the trains journey.</param>
        /// <param name="direction">The direction of travel indicated by the direction the kilometreage is progressing.</param>
        public Train(Category Category, string trainId, trainType trainType, string locoID, trainOperator trainOperator, trainCommodity commodity, double power, List<TrainJourney> journey, direction direction)
        {
            /* Designed for interpolated train */
            this.Category = Category;
            this.trainID = trainId;
            this.trainType = trainType;
            this.locoID = locoID;
            this.trainOperator = trainOperator;
            this.commodity = commodity;
            this.powerToWeight = power;
            this.journey = journey;
            this.trainDirection = direction;
            this.include = true;
        }

        /// <summary>
        /// Train constructor for the simiulated train data.
        /// </summary>
        /// <param name="journey">The list of journey details describing the points along the trains journey.</param>
        /// <param name="Category">Analysis Category, described by operator or power to weight ratio,</param>
        /// <param name="direction">The direction of travel indicated by the direction the kilometreage is progressing.</param>
        public Train(List<TrainJourney> journey, Category Category, direction direction)
        {
            this.Category = Category;
            this.trainID = "Simulated";
            this.trainType = trainType.Simulated;
            this.locoID = "Simulated";
            this.trainOperator = trainOperator.Simulated;
            this.commodity = trainCommodity.Unknown;
            this.powerToWeight = 0;
            this.journey = journey;
            this.trainDirection = direction;
            this.include = true;
        }

        /// <summary>
        /// Determine the index of the geomerty data for the supplied kilometreage.
        /// </summary>
        /// <param name="TrainJourney">List of train details objects containt the journey details of the train.</param>
        /// <param name="targetKm">The target location to find in the geomerty data.</param>
        /// <returns>The index of the target kilometreage in the geomerty data, -1 if the target is not found.</returns>
        public int indexOfGeometryKm(List<TrainJourney> TrainJourney, double targetKm)
        {
            /* Loop through the train journey. */
            for (int journeyIdx = 0; journeyIdx < TrainJourney.Count(); journeyIdx++)
            {
                /* Match the current location with the geometry information. */
                if (Math.Abs(TrainJourney[journeyIdx].kilometreage - targetKm) * 1e10 < 1)
                    return journeyIdx;
            }

            return -1;
        }

        /// <summary>
        /// Convert Train object to an averageTrain object.
        /// </summary>
        /// <returns>An equivalent average train object.</returns>
        public AverageTrain ToAverageTrain()
        {
            /* Create the average train object. */
            AverageTrain averageSimulation = new AverageTrain();

            /* Check the current train object is valid. */
            if (this.journey.Count() == 0)
                return averageSimulation;

            /* Populate train characteristics. */
            averageSimulation.trainCategory = this.Category;
            averageSimulation.direction = this.trainDirection;
            averageSimulation.trainCount = 1;

            /* Populate the train journey properties. */
            for (int index = 0; index < this.journey.Count; index++)
            {
                averageSimulation.kilometreage.Add(this.journey[index].kilometreage);
                averageSimulation.elevation.Add(this.journey[index].elevation);
                averageSimulation.averageSpeed.Add(this.journey[index].speed);
                averageSimulation.isInLoopBoundary.Add(this.journey[index].isLoopHere);
                averageSimulation.isInTSRboundary.Add(this.journey[index].isTSRHere);
                averageSimulation.speedStDev.Add(0);
                averageSimulation.sampleCount.Add(1);
            }

            return averageSimulation;
        }
    }

    /// <summary>
    /// A Train Journey class to describe data for each point in trains journey.
    /// </summary>
    public class TrainJourney
    {
        public GeoLocation location;
        public DateTime dateTime;
        public double speed;
        public double kmPost;
        public double kilometreage;
        public double elevation;
        public bool isLoopHere;
        public bool isTSRHere;
        public bool interpolate;
        
        
        /// <summary>
        /// Default Train journey constructor
        /// </summary>
        public TrainJourney()
        {
            this.location = new GeoLocation();
            this.dateTime = DateTime.MinValue;
            this.speed = 0;
            this.kmPost = 0;
            this.kilometreage = 0;
            this.elevation = 0;
            this.isLoopHere = false;
            this.isTSRHere = false;
            this.interpolate = true;
        }

        /// <summary>
        /// Train journey constructor for train record items after processing.
        /// </summary>
        /// <param name="record">A train record item containing all the information from the data.</param>
        public TrainJourney(TrainRecord record)
        {
            this.location = record.location;
            this.dateTime = record.dateTime;
            this.speed = record.speed;
            this.kmPost = record.kmPost;
            this.kilometreage = record.kmPost;
            this.elevation = 0;
            this.isLoopHere = false;
            this.isTSRHere = false;
            this.interpolate = true;
        }

        /// <summary>
        /// Train journey constructor for train record items after processing.
        /// </summary>
        /// <param name="record">A train record item containing all the information from the data.</param>
        /// <param name="interpolate">A flag to indicate if the interpolation should be completed 
        /// between this point and the previous point.</param>
        public TrainJourney(TrainRecord record, bool interpolate)
        {
            this.location = record.location;
            this.dateTime = record.dateTime;
            this.speed = record.speed;
            this.kmPost = record.kmPost;
            this.kilometreage = record.kmPost;
            this.elevation = 0;
            this.isLoopHere = false;
            this.isTSRHere = false;
            this.interpolate = interpolate;
        }

        /// <summary>
        /// Train journey constructor for a standard train, built from the fields in the data.
        /// </summary>
        /// <param name="location">Geolocation object describing the latitude and longitude of a data point</param>
        /// <param name="date">Date and time the data point was registered.</param>
        /// <param name="speed">The instantaneous speed of the train at the time of data recording.</param>
        /// <param name="kmPost">The closest kilometreage marker to the current position.</param>
        /// <param name="kilometreage">The calaculated kilometreage of the current train position.</param>
        /// <param name="elevation">The elevation of the train at the current location, this is taken from the geometry information.</param>
        /// <param name="loop">Identification of the presence of a loop or signal at the current position.</param>
        /// <param name="TSR">Identification of the presence of a TSR at the current position.</param>
        public TrainJourney(GeoLocation location, DateTime date, double speed, double kmPost, double kilometreage, double elevation, bool loop, bool TSR)
        {
            this.location = location;
            this.dateTime = date;
            this.speed = speed;
            this.kmPost = kmPost;
            this.kilometreage = kilometreage;
            this.elevation = elevation;
            this.isLoopHere = loop;
            this.isTSRHere = TSR;
            this.interpolate = true;
        }

        /// <summary>
        /// Train journey constructor for a train after interpolating the data.
        /// </summary>
        /// <param name="date">Date and time the data point was registered.</param>
        /// <param name="speed">The instantaneous speed of the train at the time of data recording.</param>
        /// <param name="kmPost">The closest kilometreage marker to the current position.</param>
        /// <param name="kilometreage">The calaculated kilometreage of the current train position.</param>
        /// <param name="elevation">The elevation of the train at the current location, this is taken from the geometry information.</param>
        /// <param name="loop">Identification of the presence of a loop or signal at the current position.</param>
        /// <param name="TSR">Identification of the presence of a TSR at the current position.</param>
        public TrainJourney(DateTime date, double speed, double kmPost, double virtualKm, double elevation, bool loop, bool TSR)
        {
            /* For interpolated Trains */
            this.location = null;
            this.dateTime = date;
            this.speed = speed;
            this.kmPost = kmPost;
            this.kilometreage = virtualKm;
            this.elevation = elevation;
            this.isLoopHere = loop;
            this.isTSRHere = TSR;
            this.interpolate = true;
        }

        /// <summary>
        /// Train journey constructor for a train after interpolating the data.
        /// </summary>
        /// <param name="date">Date and time the data point was registered.</param>
        /// <param name="speed">The instantaneous speed of the train at the time of data recording.</param>
        /// <param name="kmPost">The closest kilometreage marker to the current position.</param>
        /// <param name="kilometreage">The calaculated kilometreage of the current train position.</param>
        /// <param name="elevation">The elevation of the train at the current location, this is taken from the geometry information.</param>
        /// <param name="loop">Identification of the presence of a loop or signal at the current position.</param>
        /// <param name="TSR">Identification of the presence of a TSR at the current position.</param>
        /// <param name="interpolate">Flag indicating if interpolation of this point was conducted.</param>
        public TrainJourney(DateTime date, double speed, double kmPost, double virtualKm, double elevation, bool loop, bool TSR, bool interpolate)
        {
            /* For interpolated Trains */
            this.location = null;
            this.dateTime = date;
            this.speed = speed;
            this.kmPost = kmPost;
            this.kilometreage = virtualKm;
            this.elevation = elevation;
            this.isLoopHere = loop;
            this.isTSRHere = TSR;
            this.interpolate = interpolate;
        }

        /// <summary>
        /// Train Journey data for simualted train.
        /// </summary>
        /// <param name="location">Geolocation object describing the latitude and longitude of a data point</param>
        /// <param name="date">Date and time the data point was registered.</param>
        /// <param name="speed">The instantaneous speed of the train at the time of data recording.</param>
        /// <param name="kmPost">The closest kilometreage marker to the current position.</param>
        /// <param name="kilometreage">The calculated kilometreage of the current train position.</param>
        /// <param name="singleLineKm">The calculated consecutive kilometreage of the current train position.</param>
        /// <param name="elevation">The elevation of the train at the current location, this is taken from the geometry information.</param>
        public TrainJourney(GeoLocation location, DateTime date, double speed, double kmPost, double singleLineKm, double elevation)
        {
            this.location = location;
            this.dateTime = date;
            this.speed = speed;
            this.kmPost = kmPost;
            this.kilometreage = singleLineKm;
            this.elevation = elevation;
            this.isLoopHere = false;
            this.isTSRHere = false;
            this.interpolate = true;
        }

    }

    /// <summary>
    /// Train Record class to record each item from the data.
    /// </summary>
    public class TrainRecord
    {
        public string trainID;
        public string locoID;
        public DateTime dateTime;
        public GeoLocation location;
        public trainOperator trainOperator;
        public trainCommodity commodity;
        public double kmPost;
        public double speed;
        public double powerToWeight;

        /// <summary>
        /// Default train record constructor.
        /// </summary>
        public TrainRecord()
        {
            this.trainID = null;
            this.locoID = null;
            this.dateTime = DateTime.MinValue;
            this.location = new GeoLocation();
            this.trainOperator = trainOperator.Unknown;
            this.commodity = trainCommodity.Unknown;
            this.kmPost = 0;
            this.speed = 0;
            this.powerToWeight = 0;
        }

        /// <summary>
        /// Train record constructor, built from the fields in the data file.
        /// </summary>
        /// <param name="trainID">The train identification.</param>
        /// <param name="locoID">The lead locomotive identification</param>
        /// <param name="time">The data and time the data point was recorded.</param>
        /// <param name="location">the geographic location of the train.</param>
        /// <param name="Operator">Identification of the train operator.</param>
        /// <param name="commodity">Identification of the commodity the tran is carrying.</param>
        /// <param name="kmPost">The closest kilometreage marker of the current position.</param>
        /// <param name="speed">The instantaneous speed of the train at the time of recording the data.</param>
        /// <param name="power">The power to weight ratio of the train.</param>
        public TrainRecord(string trainID, string locoID, DateTime time, GeoLocation location, trainOperator Operator, trainCommodity commodity, double kmPost, double speed, double power)
        {
            this.trainID = trainID;
            this.locoID = locoID;
            this.dateTime = time;
            this.location = location;
            this.trainOperator = Operator;
            this.commodity = commodity;
            this.kmPost = kmPost;
            this.speed = speed;
            this.powerToWeight = power;
        }


    }

    /// <summary>
    /// A class to describe the aggregated train data.
    /// </summary>
    public class AverageTrain
    {
        public Category trainCategory;
        public direction direction;
        public int trainCount;
        public List<double> kilometreage;
        public List<double> elevation;
        public List<double> averageSpeed;
        public List<bool> isInLoopBoundary;
        public List<bool> isInTSRboundary;
        public List<double> speedStDev;
        public List<int> sampleCount;

        /// <summary>
        /// Default average train constructor.
        /// </summary>
        public AverageTrain()
        {
            this.trainCategory = Category.Unknown;
            this.direction = direction.Unknown;
            this.trainCount = 0;
            this.kilometreage = new List<double>();
            this.elevation = new List<double>();
            this.averageSpeed = new List<double>();
            this.isInLoopBoundary = new List<bool>();
            this.isInTSRboundary = new List<bool>();
            this.speedStDev = new List<double>();
            this.sampleCount = new List<int>();
        }

        /// <summary>
        /// Average train constructor built from the aggregated data.
        /// </summary>
        /// <param name="Category">The aggregation Category of the average data</param>
        /// <param name="direction">The direction of travel of the average train.</param>
        /// <param name="count">The number of train included in the aggregation.</param>
        /// <param name="kilometreage">A list of interpolated kilometreage of the trains journey.</param>
        /// <param name="elevation">A list of elevations at the kilometreage points for the trains journey.</param>
        /// <param name="averageSpeed">The calculted average speed of the train at each kilometreage.</param>
        /// <param name="loop">Identification if the a loop is withing the boundary threshold of the current position.</param>
        /// <param name="TSR">Identification if the a TSR is withing the boundary threshold of the current position.</param>
        /// <param name="speedStDev">The standard deviation of the speed at each data point location.</param>
        /// <param name="sampleCount">A count of the number of samples used in the aggregation of the each data point.</param>
        public AverageTrain(Category Category, direction direction, int count, List<double> kilometreage, List<double> elevation, List<double> averageSpeed, List<bool> loop, List<bool> TSR, List<double> speedStDev, List<int> sampleCount)
        {
            this.trainCategory = Category;
            this.direction = direction;
            this.trainCount = count;
            this.kilometreage = kilometreage;
            this.elevation = elevation;
            this.averageSpeed = averageSpeed;
            this.isInLoopBoundary = loop;
            this.isInTSRboundary = TSR;
            this.speedStDev = speedStDev;
            this.sampleCount = sampleCount;
        }

    }

    /// <summary>
    /// A class to describe each data point in a trains journey.
    /// </summary>
    public class processTrainDataPoint
    {
        public string TrainID;
        public string locoID;
        public double PW_ratio;
        public DateTime trainDate;
        public string trainOperator;
        public string commodity;
        public string trainDirection;
        public double kmMarker;
        public double speed;
        public double transitTime;
        public bool isLoop;
        public bool isTSR;
        public GeoLocation location;
        public double alignmentElevation;
        public bool isLargeGap;
        public double simulationSpeed;
        public double simulationTime;
        public double averageSpeed;         // useful for validating the results in Tableau
        public double averageTime;          // useful for validating the results in Tableau

        /// <summary>
        /// Default process train data point
        /// </summary>
        public processTrainDataPoint()
        {
            
            this.TrainID = "unspecified";
            this.locoID = "unspecified";
            this.PW_ratio = 0;
            this.trainDate = DateTime.MinValue;
            this.trainOperator = trainCommodity.Unknown.ToString();
            this.commodity = trainCommodity.Unknown.ToString();
            this.trainDirection = direction.Unknown.ToString();
            this.kmMarker = 0;
            this.speed = 0;
            this.transitTime = 0;
            this.isLoop = false;
            this.isTSR = false;
            this.location = new GeoLocation();
            this.alignmentElevation = 0;
            this.isLargeGap = false;
            this.simulationSpeed = 0;
            this.simulationTime = 0;
            this.averageSpeed = 0;
            this.averageTime = 0;
        }

        /// <summary>
        /// Processed train data point built constructor
        /// </summary>
        /// <param name="train">Representation of the trainID.</param>
        /// <param name="loco">Representation of the loco ID.</param>
        /// <param name="pw">Calculation of the power to weight ratio of the train consist.</param>
        /// <param name="date">estimated train date.</param>
        /// <param name="customer">object describing the train operator.</param>
        /// <param name="commodity">object describing the identified commodity.</param>
        /// <param name="direction">The direction of travel for the train</param>
        /// <param name="km">The km marker the train is located at</param>
        /// <param name="speed">The speed the train is travelling when at current km marker.</param>
        /// <param name="time">The time spent travelling from the last km marker to the current.</param>
        /// <param name="loop">A flag indicating if the train is assumed to be approaching or leaving a loop location.</param>
        /// <param name="tsr">A flag indicating if the train is assumed to be approaching or leaving a TSR location.</param>
        /// <param name="location">The geographic location of the km marker</param>
        /// <param name="elev">The estiated elevation of the alignment in metres.</param>
        /// <param name="gap">A flag indicating if the km marker has been identified as being within a large gap for the origial train data.</param>
        /// <param name="simSpeed">The value of the simulated speed at this location.</param>
        /// <param name="simTime">The simulated time spent travelling from the last km marker to the current.</param>
        /// <param name="aveSpeed">The speed of an average train at this location calculated from the program.</param>
        /// <param name="aveTime">The time spent travelling from the last km marker to the current for an avearage train.</param>
        public processTrainDataPoint(string train, string loco, double pw, DateTime date, trainOperator customer, 
            trainCommodity commodity, direction direction, double km, double speed, double time, bool loop, 
            bool tsr, GeoLocation location, double elev, bool gap, double simSpeed, double simTime, double aveSpeed, double aveTime)
        {
            this.TrainID = train;
            this.locoID = loco;
            this.PW_ratio = 0;
            this.trainDate = date;
            this.trainOperator = customer.ToString();
            this.commodity = commodity.ToString();
            this.trainDirection = direction.ToString();
            this.kmMarker = km;
            this.speed = speed;
            this.transitTime = time;
            this.isLoop = loop;
            this.isTSR = tsr;
            this.location = location;
            this.alignmentElevation = elev;
            this.isLargeGap = gap;
            this.simulationSpeed = simSpeed;
            this.simulationTime = simTime;
            this.averageSpeed = aveSpeed;
            this.averageTime = aveTime;
        }

    }

    /// <summary>
    /// A class describing the pair of trains that interact in a conflict.
    /// Ie. one train stops and the other goes through.
    /// </summary>
    public class TrainPair
    {
        public Train stoppedTrain;
        public Train throughTrain;
        public LoopLocation loopLocation;
        public double stoppedLocation;
        public double restartLocation;

        public double distanceToTrackSpeed;
        public double timeForStoppedTrainToReachTrackSpeed;
        public double simulatedTrainToReachTrackSpeedLocation;
        public double timeBetweenClearingLoopAndRestart;
        public double transactionTime;

        /// <summary>
        /// Default train pair constructor.
        /// </summary>
        public TrainPair()
        {
            this.stoppedTrain = null;
            this.throughTrain = null;
            this.loopLocation = null;
            this.stoppedLocation = 0;
            this.restartLocation = 0;

            this.distanceToTrackSpeed = 0;
            this.timeForStoppedTrainToReachTrackSpeed = 0;
            this.simulatedTrainToReachTrackSpeedLocation = 0;
            this.timeBetweenClearingLoopAndRestart = 0;
            this.transactionTime = 0;
        }

        /// <summary>
        /// Train pair constructor for each conflit at a loop.
        /// </summary>
        /// <param name="stopped">The stopped trains.</param>
        /// <param name="through">The train that continues through the loop.</param>
        /// <param name="location">The location of the loop.</param>
        /// <param name="stop">The kilometreage where the train stops.</param>
        /// <param name="restart">The kilometreage where the train restarts.</param>
        public TrainPair(Train stopped, Train through, LoopLocation location, double stop, double restart)
        {
            this.stoppedTrain = stopped;
            this.throughTrain = through;
            this.loopLocation = location;
            this.stoppedLocation = stop;
            this.restartLocation = restart;

            this.distanceToTrackSpeed = 0;
            this.timeForStoppedTrainToReachTrackSpeed = 0;
            this.simulatedTrainToReachTrackSpeedLocation = 0;
            this.timeBetweenClearingLoopAndRestart = 0;
            this.transactionTime = 0;
        }

        /// <summary>
        /// Function gets the corresponding simulation to the instance of the stopped train.
        /// </summary>
        /// <param name="simulations">A list of simulations</param>
        /// <returns>The simulation that corresponds to the stopped train.</returns>
        public Train matchToSimulation(List<Train> simulations)
        {
            /* Cycle through all the simulations */
            foreach (Train train in simulations)
            {
                if (train.Category == Processing.convertTrainOperatorToCategory(this.stoppedTrain.trainOperator) &&
                    train.trainDirection == this.stoppedTrain.trainDirection)
                    return train;
            }

            /* This section should return an weighted average simualtion.
             * The weighted average simualtion need to be added to the list prior to calling the fucntion.
             */
            if (this.stoppedTrain.trainDirection == direction.IncreasingKm)
                return simulations[0];
            else
                return simulations[1];

        }

    }

    /// <summary>
    /// A class to hold the journey characterstics of the volume.
    /// </summary>
    public class volumeMovement
    {
        public string trainID;
        public trainOperator trainOperator;
        public trainCommodity commodity;
        public string wagonID;
        public List<string> Origin;         /* Location name, location SA4 Region, Location State, Location Area */
        public List<string> Via;            /* Location name, location SA4 Region, Location State, Location Area */
        public List<string> Destination;    /* Location name, location SA4 Region, Location State, Location Area */
        public string OriginDestination;
        public double netWeight;
        public double grossWeight;
        public DateTime attachmentTime;
        public DateTime detachmentTime;
        public bool hasBeenCounted;

        /// <summary>
        /// Default volumeMovement constructor
        /// </summary>
        /// <param name="volume">Wagon details object.</param>
        public volumeMovement(wagonDetails volume)
        {
            this.trainID = volume.TrainID;
            this.trainOperator = volume.trainOperator;
            this.commodity = volume.commodity;
            this.wagonID = volume.wagonID;
            this.Origin = new List<string> { volume.origin, null, null, null };
            this.Via = new List<string> { volume.plannedDestination, null, null, null };
            this.Destination = new List<string> { volume.destination, null, null, null };
            this.OriginDestination = this.Origin[0] + "-" + this.Destination[0];
            this.netWeight = volume.netWeight;
            this.grossWeight = volume.grossWeight;
            this.attachmentTime = volume.attachmentTime;
            this.detachmentTime = volume.detachmentTime;

            this.hasBeenCounted = false;

        }

        /// <summary>
        /// VolumeMovement constructor.
        /// </summary>
        /// <param name="volume">Base volume object.</param>
        public volumeMovement(volumeMovement volume)
        {
            this.trainID = volume.trainID;
            this.trainOperator = volume.trainOperator;
            this.commodity = volume.commodity;
            this.wagonID = volume.wagonID;
            this.Origin = new List<string> { volume.Origin[0], null, null, null };
            this.Via = new List<string> { volume.Via[0], null, null, null };
            this.Destination = new List<string> { volume.Destination[0], null, null, null };
            this.OriginDestination = this.Origin[0] + "-" + this.Destination[0];
            this.netWeight = volume.netWeight;
            this.grossWeight = volume.grossWeight;
            this.attachmentTime = volume.attachmentTime;
            this.detachmentTime = volume.detachmentTime;

            this.hasBeenCounted = false;

        }

        /// <summary>
        /// VolumeMovement constructor.
        /// </summary>
        /// <param name="wagonID">Wagon class and number.</param>
        /// <param name="Origin">Wagon origin.</param>
        /// <param name="plannedDestination">Wagon planned destination.</param>
        /// <param name="Destination">Wagon actual destination.</param>
        /// <param name="weight">Net weight carried by the wagon.</param>
        public volumeMovement(string trainID, trainOperator trainOperator, trainCommodity commodity, string wagonID, List<string> Origin, List<string> plannedDestination, List<string> Destination, double netWeight, double grossWeight, DateTime attachmentTime, DateTime detachmentTime)
        {
            this.trainID = trainID;
            this.trainOperator = trainOperator;
            this.commodity = commodity; 
            this.wagonID = wagonID;
            this.Origin = Origin;
            this.Via = plannedDestination;
            this.Destination = Destination;
            this.OriginDestination = this.Origin[0] + "-" + this.Destination[0];
            this.netWeight = netWeight;
            this.grossWeight = grossWeight;
            this.attachmentTime = attachmentTime;
            this.detachmentTime = detachmentTime;

            this.hasBeenCounted = false;

        }

        /// <summary>
        /// VolumeMovement constructor.
        /// </summary>
        /// <param name="wagonID">Wagon class and number.</param>
        /// <param name="Origin">Wagon origin.</param>
        /// <param name="plannedDestination">Wagon planned destination.</param>
        /// <param name="Destination">Wagon actual destination.</param>
        /// <param name="weight">Net weight carried by the wagon.</param>
        public volumeMovement(string trainID, trainOperator trainOperator, trainCommodity commodity, string wagonID, string Origin, string plannedDestination, string Destination, double netWeight, double grossWeight, DateTime attachmentTime, DateTime detachmentTime)
        {
            this.trainID = trainID;
            this.trainOperator = trainOperator;
            this.commodity = commodity;
            this.wagonID = wagonID;
            this.Origin = new List<string> { Origin, null, null, null };
            this.Via = new List<string> { plannedDestination, null, null, null };
            this.Destination = new List<string> { Destination, null, null, null };
            this.OriginDestination = this.Origin[0] + "-" + this.Destination[0];
            this.netWeight = netWeight;
            this.grossWeight = grossWeight;
            this.attachmentTime = attachmentTime;
            this.detachmentTime = detachmentTime;

            this.hasBeenCounted = false;

        }
        
        /// <summary>
        /// VolumeMovement constructor.
        /// </summary>
        /// <param name="wagonID">Wagon class and number.</param>
        /// <param name="Origin">Wagon origin.</param>
        /// <param name="plannedDestination">Wagon planned destination.</param>
        /// <param name="Destination">Wagon actual destination.</param>
        /// <param name="weight">Net weight carried by the wagon.</param>
        /// <param name="hasBeenCounted">Flag indicating if the volume has been counted in the final volume movement list.</param>
        public volumeMovement(string trainID, trainOperator trainOperator, trainCommodity commodity, string wagonID, string Origin, string plannedDestination, string Destination, double netWeight, double grossWeight, DateTime attachmentTime, DateTime detachmentTime, bool hasBeenCounted)
        {
            this.trainID = trainID;
            this.trainOperator = trainOperator;
            this.commodity = commodity;
            this.wagonID = wagonID;
            this.Origin = new List<string> { Origin, null, null, null };
            this.Via = new List<string> { plannedDestination, null, null, null };
            this.Destination = new List<string> { Destination, null, null, null };
            this.OriginDestination = this.Origin[0] + "-" + this.Destination[0];
            this.netWeight = netWeight;
            this.grossWeight = grossWeight;
            this.attachmentTime = attachmentTime;
            this.detachmentTime = detachmentTime;
            this.hasBeenCounted = hasBeenCounted;

        }


    };

    /// <summary>
    /// A class to hold the wagon Characteristics for each wagon journey.
    /// </summary>
    public class wagonDetails
    {

        public string TrainID;
        public DateTime trainDate;
        public trainOperator trainOperator;
        public trainCommodity commodity;
        public string wagonID;
        public string origin;
        public string plannedDestination;
        public string destination;
        public DateTime attachmentTime;
        public DateTime detachmentTime;
        public double netWeight;
        public double grossWeight;

        /// <summary>
        /// Default Wagon Constrcutor.
        /// </summary>
        /// <param name="wagon">Wagon structure containing the origin, destination and volume carries among other properties.</param>
        public wagonDetails(wagonDetails wagon)
        {
            this.TrainID = wagon.TrainID;
            this.trainDate = wagon.trainDate;
            this.trainOperator = wagon.trainOperator;
            this.commodity = wagon.commodity;
            
            this.wagonID = wagon.wagonID;
            this.origin = wagon.origin;
            this.plannedDestination = wagon.plannedDestination;
            this.destination = wagon.destination;
            this.attachmentTime = wagon.attachmentTime;
            this.detachmentTime = wagon.detachmentTime;
            this.netWeight = wagon.netWeight;
            this.grossWeight = wagon.grossWeight;

            /* Fix the known issues with the location codes. */
            fixIssues(this.origin, this.plannedDestination, this.destination);
        }

        /// <summary>
        /// Wagon Constructor.
        /// </summary>
        /// <param name="wagonID">The Wagon class ID.</param>
        /// <param name="origin">The Origin location code of the wagon.</param>
        /// <param name="plannedDestination">The planned destination code of the wagon.</param>
        /// <param name="destination">The destination code of the wagon.</param>
        /// <param name="netWeight">The net weight carried to the destination by the wagon.</param>
        public wagonDetails(string TrainID, DateTime trainDate, trainOperator trainOperator, trainCommodity commodity,
            string wagonID, string origin, string plannedDestination, string destination, double netWeight, double grossWeight)
        {
            this.TrainID = TrainID;
            this.trainDate = trainDate;
            this.trainOperator = trainOperator;
            this.commodity = commodity;
            
            this.wagonID = wagonID;
            this.origin = origin;
            this.plannedDestination = plannedDestination;
            this.destination = destination;
            this.attachmentTime = DateTime.MinValue;
            this.detachmentTime = DateTime.MinValue;
            this.netWeight = netWeight;
            this.grossWeight = grossWeight;

            /* Fix the known issues with the location codes. */
            fixIssues(this.origin, this.plannedDestination, this.destination);
        }

        /// <summary>
        /// Wagon Constructor.
        /// </summary>
        /// <param name="wagonID">The Wagon class ID.</param>
        /// <param name="origin">The Origin location code of the wagon.</param>
        /// <param name="plannedDestination">The planned destination code of the wagon.</param>
        /// <param name="destination">The destination code of the wagon.</param>
        /// <param name="attachmentTime">The time the wagon was attached to the Train.</param>
        /// <param name="detachmentTime">The time the wagon was detached from the Train.</param>
        /// <param name="netWeight">The net weight carried to the destination by the wagon.</param>
        public wagonDetails(string TrainID, DateTime trainDate, trainOperator trainOperator, trainCommodity commodity,
            string wagonID, string origin, string plannedDestination, string destination, DateTime attachmentTime, DateTime detachmentTime, double netWeight, double grossWeight)
        {
            this.TrainID = TrainID;

            this.trainDate = trainDate;
            this.trainOperator = trainOperator;
            this.commodity = commodity;
            
            this.wagonID = wagonID;
            this.origin = origin;
            this.plannedDestination = plannedDestination;
            this.destination = destination;
            this.attachmentTime = attachmentTime;
            this.detachmentTime = detachmentTime;
            this.netWeight = netWeight;
            this.grossWeight = grossWeight;

            /* Fix the known issues with the location codes. */
            fixIssues(this.origin, this.plannedDestination, this.destination);
        }

        /// <summary>
        /// Fix the known issues with the location codes.
        /// </summary>
        /// <param name="plannedDestination">The planned destination code of the wagon.</param>
        /// <param name="destination">The destination code of the wagon.</param>
        private void fixIssues(string origin, string plannedDestination, string destination)
        {

            /* Issue 1:
             * The location code 'LAV' does not exist. It is assumed that this refers to SCT-Laverton 
             * as the next origin location is 'SCT' (SCT-Laverton). 
             */
            if (origin.Equals("LAV"))
                this.origin = "SCT";

            if (plannedDestination.Equals("LAV"))
                this.plannedDestination = "SCT";

            if (destination.Equals("LAV"))
                this.destination = "SCT";

            /* Issue 2:
             * When the location code 'CNM' appears in the destination, the next origin location is 'PGM'.
             * These locations are approximtly 20 km apart, with no indication of how the wagon was 
             * transported between these locations. Therefore, it is assumed that the two locations 
             * are the same. The 'PGM' location has been chosen to be the reference location. 
             */
            if (origin.Equals("CNM"))
                this.origin = "PGM";

            if (plannedDestination.Equals("CNM"))
                this.plannedDestination = "PGM";

            if (destination.Equals("CNM"))
                this.destination = "PGM";

            /* Issue 3:
             * The location codes 'SDY' (South Dynon) and 'CNL' (Canal Yard [A.K.A. NRC Steel]) refer 
             * to locations that are only a few km apart. when these two location appear in successive 
             * wagon movements there is typically no indication of how the wagon was moved between 
             * locations. Therefore these locations are consisdered the same, ie SDY.
             */
            if (origin.Equals("CNL"))
                this.origin = "SDY";

            if (plannedDestination.Equals("CNL"))
                this.plannedDestination = "SDY";

            if (destination.Equals("CNL"))
                this.destination = "SDY";

            /* Issue 4:
             * The Melbourne Operations Terminal (MOT) is adjacent to South Dynon (SDY). Occaissionally, 
             * the wagon is mioved from MOT to SDY without a train movement. The locations are within 
             * 1 km, therefore are considered to be the same location. SDY is used as the reference.
             */
            if (origin.Equals("MOT"))
                this.origin = "SDY";

            if (plannedDestination.Equals("MOT"))
                this.plannedDestination = "SDY";

            if (destination.Equals("MOT"))
                this.destination = "SDY";

            /* Issue 5:
             * When the location code 'DYS' appears in the destination, occasionally the next origin 
             * location is 'SCT'. These locations are approximtly 20 km apart, with no indication of how 
             * the wagon was transported between these locations. The assumption is that the trains are 
             * typically SCT trains transporting from DYS to SCT - Laverton, with these movements not 
             * recorded. Therefore, it is assumed that the two locations are the same. The 'DYS' location 
             * has been chosen to be the reference location because it is the common location for all 
             * trains, North Dynon. 
             */
            //if (origin.Equals("SCT"))
            //    this.origin = "DYS";

            //if (plannedDestination.Equals("SCT"))
            //    this.plannedDestination = "DYS";

            //if (destination.Equals("SCT"))
            //    this.destination = "DYS";


            /* Issue 6:
             * There is no defined location for the location code 'RHS'. There are 20 different actual 
             * destinations reached when the planned destination code is 'RHS'. The wagons movement
             * that do not reach the actual destination account for 8 % of these movements, while
             * 88% end up reaching 'STM', St Mary's (Mt Druit).
             * Therefore, we make the assumption that the location code of 'RHS', is intended
             * to be 'STM'
             */
            if (origin.Equals("RHS"))
                this.origin = "STM";

            if (plannedDestination.Equals("RHS"))
                this.plannedDestination = "STM";

            if (destination.Equals("RHS"))
                this.destination = "STM";
        }

        /// <summary>
        /// Overloaded Equal operator
        /// </summary>
        /// <param name="wagon1">The initial wagon object.</param>
        /// <param name="wagon2">The wagon object to compare with</param>
        /// <returns>Returns True if the objects are the same wagons.</returns>
        public static bool operator ==(wagonDetails wagon1, wagonDetails wagon2)
        {
            if (ReferenceEquals(wagon1, wagon2))
                return true;

            if (ReferenceEquals(wagon1, null))
                return false;

            if (ReferenceEquals(wagon2, null))
                return false;

            /* Compare all the parameters of the wagon. */
            return (wagon1.TrainID == wagon2.TrainID &&
                wagon1.trainDate == wagon2.trainDate &&
                wagon1.trainOperator == wagon2.trainOperator &&
                wagon1.commodity == wagon2.commodity&&
                wagon1.wagonID == wagon2.wagonID &&
                wagon1.origin == wagon2.origin &&
                wagon1.plannedDestination == wagon2.plannedDestination &&
                wagon1.destination == wagon2.destination &&
                wagon1.attachmentTime == wagon2.attachmentTime &&
                wagon1.detachmentTime == wagon2.detachmentTime &&
                wagon1.netWeight == wagon2.netWeight &&
                wagon1.grossWeight == wagon2.grossWeight);

        }

        /// <summary>
        /// Overloaded Not Equal operator
        /// </summary>
        /// <param name="wagon1">The initial wagon object.</param>
        /// <param name="wagon2">The wagon object to compare with</param>
        /// <returns>Returns True if the objects are the same trains.</returns>
        public static bool operator !=(wagonDetails wagon1, wagonDetails wagon2)
        {
            return !(wagon1 == wagon2);
        }

        /// <summary>
        /// Overloaded Equals method
        /// </summary>
        /// <param name="other">The wagon object to compare.</param>
        /// <returns>Returns True if the objects are the same wagons.</returns>
        public bool Equals(wagonDetails other)
        {
            if (ReferenceEquals(null, other))
                return false;

            if (ReferenceEquals(this, other))
                return true;

            /* Compare all the parameters of the wagon. */
            return (TrainID.Equals(other.TrainID) &&
                trainDate.Equals(other.trainDate) &&
                trainOperator.Equals(other.trainOperator) &&
                commodity.Equals(other.commodity) &&
                wagonID.Equals(other.wagonID) &&
                origin.Equals(other.origin) &&
                plannedDestination.Equals(other.plannedDestination) &&
                destination.Equals(other.destination) &&
                attachmentTime.Equals(other.attachmentTime) &&
                detachmentTime.Equals(other.detachmentTime) &&
                netWeight.Equals(other.netWeight) &&
                grossWeight.Equals(other.grossWeight));
        }

        /// <summary>
        /// Override Equals method
        /// </summary>
        /// <param name="obj">System object to compare</param>
        /// <returns>Returns True if the objects are the same wagons.</returns>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj))
                return false;

            if (ReferenceEquals(this, obj))
                return true;

            /* Ensure the objects are the same type and that they can be cast to the train object. */
            return obj.GetType() == GetType() && Equals((wagonDetails)obj);
        }

        /// <summary>
        /// Implementation of custom HashCode for the equals methods.
        /// </summary>
        /// <returns>returns a hashcode for the wagon opbject.</returns>
        public override int GetHashCode()
        {
            /* Return the hash codes for the all components of the wagon object. */
            return TrainID.GetHashCode() +
                trainDate.GetHashCode() +
                trainOperator.GetHashCode() +
                commodity.GetHashCode() +
                wagonID.GetHashCode() +
                origin.GetHashCode() +
                plannedDestination.GetHashCode() +
                destination.GetHashCode() +
                attachmentTime.GetHashCode() +
                detachmentTime.GetHashCode() +
                netWeight.GetHashCode() +
                grossWeight.GetHashCode();

        }
    }

    /// <summary>
    /// A class defining the loop or signal location by kilometreage.
    /// </summary>
    public class LoopLocation
    {
        public double loopStart;
        public double loopEnd;

        /// <summary>
        /// Default loop location constructor
        /// </summary>
        public LoopLocation()
        {
            this.loopStart = 0;
            this.loopEnd = 1;
        }

        /// <summary>
        /// Loop location constructor
        /// </summary>
        /// <param name="location">A loop location object.</param>
        public LoopLocation(LoopLocation location)
        {
            this.loopStart = location.loopStart;
            this.loopEnd = location.loopEnd;
        }

        /// <summary>
        /// Loop location constructor
        /// </summary>
        /// <param name="start">The smallest kilometreage of the loop ends.</param>
        /// <param name="end">The largest kilometreage of the loop ends.</param>
        public LoopLocation(double start, double end)
        {
            this.loopStart = start;
            this.loopEnd = end;
        }

    }

    /// <summary>
    /// A Class defining the start and end of a line section.
    /// </summary>
    public class Section
    {
        public double sectionStart;
        public double sectionEnd;

        /// <summary>
        /// Default Section constructor
        /// </summary>
        public Section()
        {
            this.sectionStart = 0;
            this.sectionEnd = 0;
        }

        /// <summary>
        /// Section Constructor
        /// </summary>
        /// <param name="start">The start kilometerage of the section.</param>
        /// <param name="end">The end kilometerage of the section.</param>
        public Section(double start, double end)
        {
            this.sectionStart = start;
            this.sectionEnd = end;
        }

    }

    /// <summary>
    /// A class describing individual occupation blocks for a section
    /// </summary>
    public class OccupationBlock
    {
        public Section section;
        public DateTime startTime;
        public DateTime endTime;
        public double minutesOccupied;

        /// <summary>
        /// Default Ocupation block constructr
        /// </summary>
        public OccupationBlock()
        {
            this.section = new Section();
            this.startTime = DateTime.MinValue;
            this.endTime = DateTime.MinValue;
            this.minutesOccupied = (endTime - startTime).TotalMinutes;
        }

        /// <summary>
        /// Occupation block constructor
        /// </summary>
        /// <param name="section">The section that the occupation block belongs to.</param>
        /// <param name="startTime">The start time of the occupation.</param>
        /// <param name="endTime">The end time of the occupation</param>
        public OccupationBlock(Section section, DateTime startTime, DateTime endTime)
        {
            this.section = section;
            this.startTime = startTime;
            this.endTime = endTime;
            this.minutesOccupied = (endTime - startTime).TotalMinutes;
        }
    }
    
    /// <summary>
    /// A class describing a geographic location with latitude and longitude.
    /// </summary>
    public class GeoLocation
    {
        /* Latitude and longitude of the location */
        public double latitude;
        public double longitude;

        /// <summary>
        /// Default constructor.
        /// </summary>
        public GeoLocation()
        {
            // Default: Sydney Harbour Bridge
            this.latitude = -33.8519;
            this.longitude = 151.2108;
        }

        /// <summary>
        /// Geolocation constructor
        /// </summary>
        /// <param name="lat">latitude of the location.</param>
        /// <param name="lon">longitude of the location.</param>
        public GeoLocation(double lat, double lon)
        {
            this.latitude = lat;
            this.longitude = lon;
        }

        /// <summary>
        /// Geolocation constructor
        /// </summary>
        /// <param name="record">Train record item containing the latitude and longitude.</param>
        public GeoLocation(TrainRecord record)
        {
            this.latitude = record.location.latitude;
            this.longitude = record.location.longitude;
        }

        /// <summary>
        /// Geolocation constructor
        /// </summary>
        /// <param name="journey">Train journey item containing the latitude and longitude.</param>
        public GeoLocation(TrainJourney journey)
        {
            this.latitude = journey.location.latitude;
            this.longitude = journey.location.longitude;
        }

    }

    /// <summary>
    /// A class describing the parameters associated with a TSR.
    /// </summary>
    public class TSRObject
    {
        public string Region;
        public DateTime IssueDate;
        public DateTime LiftedDate;
        public double startKm;
        public double endKm;
        public double TSRSpeed;

        /// <summary>
        /// Default TSRObject constructor.
        /// </summary>
        public TSRObject()
        {
            this.Region = "Unknown";
            this.IssueDate = DateTime.MinValue;
            this.LiftedDate = DateTime.MinValue;
            this.startKm = 0;
            this.endKm = 0;
            this.TSRSpeed = 0;
        }

        /// <summary>
        /// TSRObject constructor
        /// </summary>
        /// <param name="region">Region the TSR is in.</param>
        /// <param name="issued">The date the TSR was applied.</param>
        /// <param name="lifted">The Date the TSR was lifted, if applicable.</param>
        /// <param name="start">The start km of the TSR.</param>
        /// <param name="finish">The end Km of the TSR.</param>
        /// <param name="speed">The speed restriction applied to the TSR.</param>
        public TSRObject(string region, DateTime issued, DateTime lifted, double start, double finish, double speed)
        {
            this.Region = region;
            this.IssueDate = issued;
            this.LiftedDate = lifted;
            this.startKm = start;
            this.endKm = finish;
            this.TSRSpeed = speed;
        }

    }

    /// <summary>
    /// A class describing the corridor settings for automated processing of train data.
    /// </summary>
    public class CorridorSettings
    {
        /* Corridor geometry file */
        public string geometryFile;

        /* Interpolation settings*/
        public double startKm;
        public double endKm;
        public double interval;
        public bool IgnoreGaps;
        public bool trainsStoppingAtLoops;
        public double loopSpeedThreshold;
        public double loopBoundaryThreshold;
        public double TSRwindowBoundary;

        /* Procesing decision settings */
        public double timeThreshold;
        public double distanceThreshold;
        public double minimumJourneyDistance;
        public analysisCategory analysisCategory;
        
        /* Simulation settings */
        public Dictionary<string, string> simulationFiles = new Dictionary<string, string>();
        public List<Category> simCategories = new List<Category>();

        /* TSR related information */
        public string TSR_file = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\TSR_report.csv";
        public List<int> basecodeList;

        /* Irrelavant for the greater hunter valley area, but usefull for 
         * future corridor expansion into the interstate network.
         */
        public double Category1LowerBound;
        public double Category1UpperBound;
        public double Category2LowerBound;
        public double Category2UpperBound;

        /// <summary>
        /// Default constructor for corridor settings
        /// </summary>
        public CorridorSettings()
        {
            /* Default to Gunnedah settings */
            gunnedah();
        }

        /// <summary>
        /// Helper function to construct the corridor settings based on the corridor name.
        /// </summary>
        /// <param name="corridor">The name of the corridor</param>
        public CorridorSettings(string corridor)
        {
            switch (corridor)
            {
                /* Set the corridor settings for the Gunnedah line.*/
                case "gunnedah":
                case "Gunnedah":
                case "MUS-NBI":
                    gunnedah();
                    break;

                /* Set the corridor settings for the Ulan line.*/
                case "ulan":
                case "Ulan":
                case "MUS-ULN":
                    ulan();
                    break;

                /* Set the corridor settings for the Hunter line.*/
                case "hunter":
                case "Hunter":
                case "KIY-WCK":
                    hunter();
                    break;

                default:
                    gunnedah();
                    break;
            }
        }

        /// <summary>
        /// Gunnedah line corridor settings
        /// </summary>
        private void gunnedah()
        {
            this.geometryFile = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Gunnedah Basin\Gunnedah Basin Geometry.csv";

            this.startKm = 280.0;
            this.endKm = 540.0;
            this.interval = 50;
            this.IgnoreGaps = false;
            this.trainsStoppingAtLoops = false;
            this.loopSpeedThreshold = 0.5;
            this.loopBoundaryThreshold = 2;
            this.TSRwindowBoundary = 2;

            this.timeThreshold = 10;
            this.distanceThreshold = 4;
            this.minimumJourneyDistance = 50;
            this.analysisCategory = analysisCategory.TrainOperator;


            this.simulationFiles["Aurizon-IncreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Gunnedah Basin\Aurizon-Increasing-60.csv";
            this.simulationFiles["Aurizon-DecreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Gunnedah Basin\Aurizon-Decreasing.csv";

            this.simulationFiles["PacificNational-IncreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Gunnedah Basin\PacificNational-Increasing.csv";
            this.simulationFiles["PacificNational-DecreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Gunnedah Basin\PacificNational-Decreasing.csv";

            this.simCategories.Add(Category.Aurizon);
            this.simCategories.Add(Category.PacificNational);

            int[] basecodes = { 10003,      /* Main North Single Line Muswellbrook to Werris Creek */
                                10018};     /* Werris Creek to The Gap Single Line + The Gap to Narrabri Jct Single Line */
            this.basecodeList = new List<int>(basecodes);
            
            // Not realy used for Hunter Valley region
            this.Category1LowerBound = 0;
            this.Category1UpperBound = 0;
            this.Category2LowerBound = 0;
            this.Category2UpperBound = 0;
        }

        /// <summary>
        /// Ulan line corridor settings
        /// </summary>
        private void ulan()
        {
            this.geometryFile = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Ulan\Ulan Geometry.csv";

            this.startKm = 280.0;
            this.endKm = 460.0;
            this.interval = 50;
            this.IgnoreGaps = false;
            this.trainsStoppingAtLoops = false;
            this.loopSpeedThreshold = 0.5;
            this.loopBoundaryThreshold = 2;
            this.TSRwindowBoundary = 2;

            this.timeThreshold = 10;
            this.distanceThreshold = 4;
            this.minimumJourneyDistance = 50;
            this.analysisCategory = analysisCategory.TrainOperator;
            
            this.simulationFiles["Aurizon-IncreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Ulan\Aurizon - Increasing.csv";
            this.simulationFiles["Aurizon-DecreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Ulan\Aurizon - Decreasing.csv";

            this.simulationFiles["Freightliner-IncreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Ulan\Freightliner - Increasing.csv";
            this.simulationFiles["Freightliner-DecreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Ulan\Freightliner - Decreasing.csv";

            this.simulationFiles["PacificNational-IncreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Ulan\PacificNational - Increasing.csv";
            this.simulationFiles["PacificNational-DecreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Ulan\PacificNational - Decreasing.csv";

            this.simCategories.Add(Category.Aurizon);
            this.simCategories.Add(Category.Freightliner);
            this.simCategories.Add(Category.PacificNational);

            int[] basecodes = { 10026 };      /* Ulan Line Single Line to Muswellbrook Ulan */
                                
            this.basecodeList = new List<int>(basecodes);


            // Not realy used for Hunter Valley region
            this.Category1LowerBound = 0;
            this.Category1UpperBound = 0;
            this.Category2LowerBound = 0;
            this.Category2UpperBound = 0;
        }

        /// <summary>
        /// Hunter line corridor settings
        /// </summary>
        private void hunter()
        {
            this.geometryFile = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Hunter Region\KIY to WCK.csv";

            this.startKm = 160.0;
            this.endKm = 290.0;
            this.interval = 50;
            this.IgnoreGaps = false;
            this.trainsStoppingAtLoops = false;
            this.loopSpeedThreshold = 0.5;
            this.loopBoundaryThreshold = 1;
            this.TSRwindowBoundary = 1;

            this.timeThreshold = 10;
            this.distanceThreshold = 4;
            this.minimumJourneyDistance = 50;
            this.analysisCategory = analysisCategory.TrainOperator;
            
            this.simulationFiles["Aurizon-IncreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Hunter Region\Aurizon - Increasing.csv";
            this.simulationFiles["Aurizon-DecreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Hunter Region\Aurizon - Decreasing.csv";

            this.simulationFiles["Freightliner-IncreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Hunter Region\Freightliner - Increasing.csv";
            this.simulationFiles["Freightliner-DecreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Hunter Region\Freightliner - Decreasing.csv";

            this.simulationFiles["PacificNational-IncreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Hunter Region\PacificNational - Increasing.csv";
            this.simulationFiles["PacificNational-DecreasingKm"] = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis\Hunter Region\PacificNational - Decreasing.csv";

            this.simCategories.Add(Category.Aurizon);
            this.simCategories.Add(Category.Freightliner);
            this.simCategories.Add(Category.PacificNational);

            int[] basecodes = { 10001,      /* Main North Up Whittingham to Maitland + Main North Up Muswellbrook to Whittingham */
                                10002,      /* Main North Down Maitland to Whittingham + Main North Down Whittingham to Muswellbrook */
                                10212,      /* Coal Road Down */
                                10213};     /* coal Road Up */
            this.basecodeList = new List<int>(basecodes);

            // Not realy used for Hunter Valley region
            this.Category1LowerBound = 0;
            this.Category1UpperBound = 0;
            this.Category2LowerBound = 0;
            this.Category2UpperBound = 0;
        }

    }

}
