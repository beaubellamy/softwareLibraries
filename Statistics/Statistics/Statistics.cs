using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TrainLibrary;

namespace Statistics
{
    /// <summary>
    /// Statistics class for the train objects
    /// </summary>
    public class TrainStatistics
    {

        public string Category;
        public double numberOfTrains;
        public double averageDistanceTravelled;
        public double averageSpeed;
        public double averagePowerToWeightRatio;
        public double standardDeviationP2W;

        /// <summary>
        /// Default Statisitcs Constructor
        /// </summary>
        public TrainStatistics()
        {
            this.Category = "Empty";
            this.numberOfTrains = 0;
            this.averageDistanceTravelled = 0;
            this.averageSpeed = 0;
            this.averagePowerToWeightRatio = 0;
            this.standardDeviationP2W = 0;
        }

        /// <summary>
        /// Calculates the statistics of the list of trains passed in.
        /// </summary>
        /// <param name="trains">A list of train objects.</param>
        /// <returns>A statistics object containing statistical information about the trains.</returns>
        public static TrainStatistics generateStats(List<Train> trains)
        {
            TrainStatistics stats = new TrainStatistics();

            if (trains.Count() == 0)
                return stats;
            else
            {

                stats.Category = trains[0].Category.ToString() + " " + trains[0].trainDirection.ToString();

                /* Extract the number of trains in the list */
                stats.numberOfTrains = trains.Count();

                List<double> distance = new List<double>();
                List<double> speed = new List<double>();
                List<double> power2Weight = new List<double>();

                /* Cycle through all the trains. */
                foreach (Train train in trains)
                {
                    /* Calculate the distance travelled for each train */
                    double distanceTravelled = 0;
                    if (train.journey.Where(t => t.speed > 0).Count() != 0)
                    {
                        distanceTravelled = (train.journey.Where(t => t.speed > 0).Max(t => t.kilometreage) - train.journey.Where(t => t.speed > 0).Min(t => t.kilometreage));
                        /* Calculate the average speed of the train journey. */
                        speed.Add(train.journey.Where(t => t.speed > 0).Average(t => t.speed));
                    }

                    distance.Add(distanceTravelled);

                    /* Add the power to weight ratio to the list. */
                    power2Weight.Add(train.powerToWeight);

                }

                /* Calculate the averages. */
                if (speed.Count() > 0)
                    stats.averageSpeed = speed.Average();
                else
                    stats.averageSpeed = 0;

                if (distance.Count() > 0)
                    stats.averageDistanceTravelled = distance.Average();
                else
                    stats.averageDistanceTravelled = 0;

                if (power2Weight.Count() > 0)
                {
                    stats.averagePowerToWeightRatio = power2Weight.Average();
                    double sum = power2Weight.Sum(p => Math.Pow(p - stats.averagePowerToWeightRatio, 2));

                    /* Calculate the standard deviation of the power to weight ratios. */
                    stats.standardDeviationP2W = Math.Sqrt(sum / (power2Weight.Count() - 1));
                }
                else
                {
                    stats.averagePowerToWeightRatio = 0;
                    stats.standardDeviationP2W = 0;
                }

            }
            return stats;

        }
        
        /// <summary>
        /// Calculates the statistics of a single train journey, 
        /// typically used for the simualtion statistics.
        /// </summary>
        /// <param name="trains">A single train object.</param>
        /// <returns>A statistics object containing statistical information about the train.</returns>
        public static TrainStatistics generateStats(Train train)
        {
            TrainStatistics stats = new TrainStatistics();

            if (train.journey.Where(t => t.speed > 0).Count() == 0)
                return stats;

            stats.Category = train.Category.ToString() + " " + train.trainDirection.ToString();

            /* Extract the number of trains in the list */
            stats.numberOfTrains = 1;

            /* Calculate the distance travelled for each train */
            double distanceTravelled = 0;
            double averageSpeed = 0;
            if (train.journey.Where(t => t.speed > 0).Count() > 0)
            {
                distanceTravelled = (train.journey.Where(t => t.speed > 0).Max(t => t.kilometreage) - train.journey.Where(t => t.speed > 0).Min(t => t.kilometreage));
                /* Calculate the average speed of the train journey. */
                averageSpeed = train.journey.Where(t => t.speed > 0).Average(t => t.speed);
            }

            /* Populate the averages. */
            stats.averageSpeed = averageSpeed;
            stats.averageDistanceTravelled = distanceTravelled;
            
            stats.averagePowerToWeightRatio = train.powerToWeight;

            /* Because there is only one train, the standard deviation is not relevant. */
            stats.standardDeviationP2W = 0;


            return stats;

        }

        

    }

    /// <summary>
    /// Statistics class for the train pair objects.
    /// </summary>
    public class TrainPairStatistics
    {

        public string Category;
        public double numberOfTrainsPairs;
        public double averageTimeForStoppedTrainToReachTrackSpeed;
        public double averageSimulatedTrainToReachTrackSpeedLocation;
        public double averageTimeBetweenClearingLoopAndRestart;
        public double averageTransactionTime;
        public double transactionTimeStandardDeviation;


         /// <summary>
        /// Default Statisitcs Constructor
        /// </summary>
        public TrainPairStatistics()
        {
        }


        /// <summary>
        /// Calcualtes statistical information for a list of train pairs.
        /// </summary>
        /// <param name="pair">A list of train pairs</param>
        /// <returns>A statistics object containing statistical information about the train pairs.</returns>
        public static TrainPairStatistics generateStats(List<TrainPair> pair)
        {
            TrainPairStatistics stats = new TrainPairStatistics();

            if (pair.Count == 0)
            {
                stats.Category = string.Format("{0:0.00} km - {1:0.00} km", 0,0);

                /* Extract the number of trains in the list */
                stats.numberOfTrainsPairs = 0;

                /* Calculate the average transit time components. */
                stats.averageTimeForStoppedTrainToReachTrackSpeed = 0;
                stats.averageSimulatedTrainToReachTrackSpeedLocation = 0;
                stats.averageTimeBetweenClearingLoopAndRestart = 0;
                stats.averageTransactionTime = 0;

                stats.transactionTimeStandardDeviation = 0;

            }
            else if (pair.Count == 1)
            {
                stats.Category = string.Format("{0:0.00} km - {1:0.00} km", pair[0].loopLocation.loopStart, pair[0].loopLocation.loopEnd);

                /* Extract the number of trains in the list */
                stats.numberOfTrainsPairs = 1;

                /* Calculate the average transit time components. */
                stats.averageTimeForStoppedTrainToReachTrackSpeed = pair[0].timeForStoppedTrainToReachTrackSpeed;
                stats.averageSimulatedTrainToReachTrackSpeedLocation = pair[0].simulatedTrainToReachTrackSpeedLocation;
                stats.averageTimeBetweenClearingLoopAndRestart = pair[0].timeBetweenClearingLoopAndRestart;
                stats.averageTransactionTime = pair[0].transactionTime;

                /* Calculate the sample standard deviation of the transaction time. */
                stats.transactionTimeStandardDeviation = 0;
            }
            else
            {
                stats.Category = string.Format("{0:0.00} km - {1:0.00} km", pair[0].loopLocation.loopStart, pair[0].loopLocation.loopEnd);

                /* Extract the number of trains in the list */
                stats.numberOfTrainsPairs = pair.Count();

                /* Calculate the average transit time components. */
                stats.averageTimeForStoppedTrainToReachTrackSpeed = pair.Average(p => p.timeForStoppedTrainToReachTrackSpeed);
                stats.averageSimulatedTrainToReachTrackSpeedLocation = pair.Average(p => p.simulatedTrainToReachTrackSpeedLocation);
                stats.averageTimeBetweenClearingLoopAndRestart = pair.Average(p => p.timeBetweenClearingLoopAndRestart);
                stats.averageTransactionTime = pair.Average(p => p.transactionTime);

                /* Caclculate the sum of the squares for the transaction time. */
                double sum = pair.Sum(t => Math.Pow(t.transactionTime - stats.averageTransactionTime, 2));
                /* Calculate the sample standard deviation of the transaction time. */
                stats.transactionTimeStandardDeviation = Math.Sqrt(sum / (pair.Count()-1) );
            }
            return stats;

        }

    }
}
