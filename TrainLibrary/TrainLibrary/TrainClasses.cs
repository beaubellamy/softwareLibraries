using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TrainLibrary
{
    /*
     * Enumerated types to assit in giving the train classes appropriate 
     * descriptions for detailed analyis.
     */

    /* A list of available analysis seperation Categories. */
    public enum analysisCategory { TrainOperator, TrainCommodity, TrainPowerToWeight, Unknown };

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
        CityRail, Countrylink, Freightliner, GenesseeWyoming, GreatSouthernRail, Interail, JohnHollandRail, LauchlanValleyRailSociety, Limited3801, 
        MetroTrainsMelbourne, PacificNational, QUBE, QueenslandRail, RailTransportMuseum, RailCorp, SCT, SouthernShorthaulRail, SpecialistBulkRail,
        SydneyRailService, TheRailMotorService, Transport4NSW, VLinePassenger, GroupRemaining, Combined, Simulated, Unknown
    };

   
    /// <summary>
    /// A list of available commodities.
    /// </summary>
    public enum trainCommodity
    {
        GeneralFreight, Coal, Express,  Grain, Goods, Mineral, Shuttle, Steel, Shunt, Clinker, Intermodal, Passenger, TrailerRail, Work, GroupRemaining, Unknown
    };

    /// <summary>
    /// A list of analysis Categories, comprising of train operators, power to weight ratios.
    /// {TrainOperator List, TrainCommodity, power to weight Categories}
    /// </summary>
    public enum Category
    {
        /* Train Operators. */
        ARCInfrastructure, ARTC, Aurizon, AustralianRailGroup, AustralianRailwaysHistoricalSociety, AustralianTransportNetwork, AvailableRollingStock, 
        CityRail, Countrylink, Freightliner, GenesseeWyoming, GreatSouthernRail, Interail, JohnHollandRail, LauchlanValleyRailSociety, Limited3801, 
        MetroTrainsMelbourne, PacificNational, QUBE, QueenslandRail, RailTransportMuseum, RailCorp, SCT, SouthernShorthaulRail, SpecialistBulkRail,
        SydneyRailService, TheRailMotorService, Transport4NSW, VLinePassenger,  
        /* Commodities. */
        GeneralFreight, Coal, Express, Goods, Mineral, Shuttle, Steel, Shunt, Clinker, Intermodal, Passenger, TrailerRailGrain, Work,
        /* Power to weight catagories. */
        Underpowered, Overpowered, Alternative,
        /* Other */
        GroupRemaining, Combined, Actual, Simulated, Unknown
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
        /// <returns>Retrns True if the objects are the same trains.</returns>
        public static bool operator ==(Train train1, Train train2)
        {
            if (ReferenceEquals(train1, train2))
                return true;
            
            if (ReferenceEquals(train1, null))
                return false;
            
            if (ReferenceEquals(train2, null))
                return false;
            
            /* Ignore the comparison of the train journey, when comparing trains to avoid the 
             * differences between an interpoalted trains with gaps and interpoalting 
             * through the gaps for the utilisation. */
            return (train1.Category == train2.Category &&
                train1.trainID == train2.trainID &&
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
        /// <returns>Retrns True if the objects are the same trains.</returns>
        public static bool operator !=(Train train1, Train train2)
        {
            return !(train1 == train2);
        }

        /// <summary>
        /// Overloaded Equals method
        /// </summary>
        /// <param name="other">The train object to compare.</param>
        /// <returns>Retrns True if the objects are the same trains.</returns>
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
        /// <returns>Retrns True if the objects are the same trains.</returns>
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
        public Train(Category Category, string trainId, string locoID, trainOperator trainOperator, trainCommodity commodity, double power, List<TrainJourney> journey, direction direction, bool include)
        {
            this.Category = Category;
            this.trainID = trainId;
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
        public Train(Category Category, string trainId, string locoID, trainOperator trainOperator, trainCommodity commodity, double power, List<TrainJourney> journey, direction direction)
        {
            /* Designed for interpolated train */
            this.Category = Category;
            this.trainID = trainId;
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
        /// <param name="loop">Identification of the presence of a loop at the current position.</param>
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
        /// <param name="loop">Identification of the presence of a loop at the current position.</param>
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
        /// <param name="loop">Identification of the presence of a loop at the current position.</param>
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
            this.location = null;
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
        public AverageTrain(Category Category, direction direction, int count, List<double> kilometreage, List<double> elevation, List<double> averageSpeed, List<bool> loop, List<bool> TSR)
        {
            this.trainCategory = Category;
            this.direction = direction;
            this.trainCount = count;
            this.kilometreage = kilometreage;
            this.elevation = elevation;
            this.averageSpeed = averageSpeed;
            this.isInLoopBoundary = loop;
            this.isInTSRboundary = TSR;
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
            fixIssues(this.plannedDestination, this.destination);
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
            fixIssues(this.plannedDestination, this.destination);
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
            fixIssues(this.plannedDestination, this.destination);
        }

        /// <summary>
        /// Fix the known issues with the location codes.
        /// </summary>
        /// <param name="plannedDestination">The planned destination code of the wagon.</param>
        /// <param name="destination">The destination code of the wagon.</param>
        private void fixIssues(string plannedDestination, string destination)
        {
            /* Issue 1:
             * The location code 'LAV' does not exist. It is assumed that this refers to SCT-Laverton 
             * as the next origin location is 'SCT' (SCT-Laverton). 
             */
            if (plannedDestination.Equals("LAV"))
                this.plannedDestination = "SCT";

            /* Issue 2:
             * When the location code 'CNM' appears in the destination, the next origin location is 'PGM'.
             * These locations are approximtly 20 km apart, with no indication of how the wagon was 
             * transported between these locations. Therefore, it is assumed that the two locations 
             * are the same. The 'PGM' location has been chosen to be the reference location. 
             */
            if (destination.Equals("CNM"))
                this.destination = "PGM";

            /* Issue 3:
             * Successive wagon records can occaisionally represent movements that do not match 
             * the time stamp and movement through locations. Ignore the wagon record, when the 
             * weight is the same between successive wagon movements and the travel time of the 
             * second wagon and the difference between attachments of each wagon is less than 2 
             * min.
             */
            // Need access to the attachment and detatchment time.


        }
    }

    /// <summary>
    /// A class defining the loop location by kilometreage.
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


}
