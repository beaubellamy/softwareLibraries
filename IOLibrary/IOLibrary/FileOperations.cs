﻿using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Reflection;
using Microsoft.Office.Interop.Excel;

using System.Data.SqlClient;

using TrainLibrary;
using Statistics;

namespace IOLibrary
{
    /// <summary>
    /// SQL parameter class to contain the SQL conection and the SQL command to execute
    /// </summary>
    public class SqlParameters
    {
        /* SQL parameters */
        public SqlConnection connection;
        public SqlCommand command;

        /// <summary>
        ///  Default Constructor
        /// </summary>
        /// <param name="connection">The generated SQL connection object.</param>
        /// <param name="command">The generated SQL command object.</param>
        public SqlParameters(SqlConnection connection, SqlCommand command)
        {
            this.connection = connection;
            this.command = command;
        }
    }


    public class FileOperations
    {
        /* ARTC location code file */
        public static string geoLocationFile = @"C:\Users\bbel1\Documents\ARTC GEO Location Details - under construction.csv";

        /* Create a dictionary of locations:
         * Key:     Location Code [3 letter code]
         * Values:  Location Code, Name, Location SA4 region, Location State.
         */
        public static Dictionary<string, List<string>> locationDictionary = new Dictionary<string, List<string>>();

        /// <summary>
        /// Read the data file and extract the neccessary information into a train record class.
        /// </summary>
        /// <param name="filename">Filename of the data file.</param>
        /// <param name="trainList">List of trains to exclude.</param>
        /// <param name="excludeListOfTrains">Boolean flag to indicate if the list of trains should be excluded or exclusively included.</param>
        /// <param name="dateRange">The dates between which to keep the data</param>
        /// <returns>List of train records describing each point in a trains journey.</returns>
        public static List<TrainRecord> readICEData(string filename, List<string> trainList, bool excludeListOfTrains, DateTime[] dateRange)
        {

            /* Read all the lines of the data file. */
            Tools.isFileOpen(filename);

            char[] delimeters = { '\t' };
            string[] fields = null;

            /* Minimum length of the operator string to distinguisg between them. */
            int operatorStringLength = 6;
            /* Set maximum speed values
             * There are occasionally datapoints that indicate the train is going faster than phyically possible 
             */
            double maxPassengerSpeed = 180;
            double maxFreightSpeed = 120;

            /* Initialise the fields of interest. */
            string TrainID = "none";
            string locoID = "none";
            string subOperator = "";
            trainOperator trainOperator = trainOperator.Unknown;
            trainCommodity commodity = trainCommodity.Unknown;
            double powerToWeight = 0.0;
            double speed = 0.0;
            double kmPost = 0.0;
            double latitude = 0.0;
            double longitude = 0.0;
            DateTime dateTime = DateTime.MinValue;

            bool header = true;
            bool includeTrain = true;

            /* List of all valid train data. */
            List<TrainRecord> IceRecord = new List<TrainRecord>();

            foreach (string line in System.IO.File.ReadLines(filename))
            {
                if (header)
                    /* Ignore the header line. */
                    header = false;
                else
                {
                    /* Seperate each record into each field */
                    fields = line.Split(delimeters);

                    TrainID = fields[6];
                    locoID = fields[1];

                    if (fields[4].Count() >= operatorStringLength)
                        subOperator = fields[4].Substring(0, operatorStringLength);
                    else
                        subOperator = fields[4].PadRight(operatorStringLength);

                    trainOperator = getOperator(subOperator);

                    commodity = getCommodity(fields[5]);

                    /* Ensure values are valid while reading them out. */
                    double.TryParse(fields[9], out speed);
                    double.TryParse(fields[8], out kmPost);
                    kmPost /= 1000;
                    double.TryParse(fields[0], out latitude);
                    double.TryParse(fields[2], out longitude);
                    DateTime.TryParse(fields[3], out dateTime);
                    double.TryParse(fields[7], out powerToWeight);

                    /* Possible TSR information as well*/
                    /* TSR region
                     * Start km
                     * end km
                     * TSR issue Data
                     * TSR lift date
                     * 
                     * This would need to be added to the trainRecord class.
                     */

                    /* Check if the train is in the exclude list */
                    if (excludeListOfTrains)
                        includeTrain = !trainList.Contains(TrainID);
                    else
                    {
                        if (trainList.Count() > 0)
                            includeTrain = trainList.Contains(TrainID);
                        else
                            includeTrain = true;
                    }

                    /* Validate the speed data */
                    if (commodity.Equals(trainCommodity.Passenger))
                    {
                        if (speed < 0 || speed > maxPassengerSpeed)
                            includeTrain = false;
                    }
                    else
                    {
                        if (speed < 0 || speed > maxFreightSpeed)
                            includeTrain = false;
                    }

                    if (dateTime >= dateRange[0] && dateTime < dateRange[1] &&
                        includeTrain)
                    {
                        TrainRecord record = new TrainRecord(TrainID, locoID, dateTime, new GeoLocation(latitude, longitude), trainOperator, commodity, kmPost, speed, powerToWeight);
                        IceRecord.Add(record);
                    }

                }
            }

            /* Return the list of records. */
            return IceRecord;
        }

        /// <summary>
        /// Read the Azure data file and extract the neccessary information into a train record class.
        /// The data file is assumed to be created from accessing the data through Azure datawarehouse, which produces a new format.
        /// This will apply to any data collected after 01/07/2018.
        /// ------------------------------------------------------
        /// </summary>
        /// <param name="filename">Filename of the data file.</param>
        /// <param name="trainList">List of trains to exclude.</param>
        /// <param name="excludeListOfTrains">Boolean flag to indicate if the list of trains should be excluded or exclusively included.</param>
        /// <param name="dateRange">The dates between which to keep the data</param>
        /// <returns>List of train records describing each point in a trains journey.</returns>
        public static List<TrainRecord> readAzureICEData(string filename, List<string> trainList, bool excludeListOfTrains, DateTime[] dateRange)
        {

            /* Read all the lines of the data file. */
            Tools.isFileOpen(filename);

            char[] delimeters = { '\t', ',' };
            string[] fields = null;

            /* Minimum length of the operator string to distinguisg between them. */
            int operatorStringLength = 6;
            /* Set maximum speed values
             * There are occasionally datapoints that indicate the train is going faster than phyically possible 
             */
            double maxPassengerSpeed = 170;
            double maxFreightSpeed = 120;

            /* Initialise the fields of interest. */
            string TrainID = "none";
            string locoID = "none";
            string subOperator = "";
            trainOperator trainOperator = trainOperator.Unknown;
            trainCommodity commodity = trainCommodity.Unknown;
            double powerToWeight = 0.0;
            double speed = 0.0;
            double kmPost = 0.0;
            double latitude = 0.0;
            double longitude = 0.0;
            DateTime dateTime = DateTime.MinValue;

            bool header = true;
            bool includeTrain = true;

            /* List of all valid train data. */
            List<TrainRecord> IceRecord = new List<TrainRecord>();

            foreach (string line in System.IO.File.ReadLines(filename))
            {
                if (header)
                    /* Ignore the header line. */
                    header = false;
                else
                {
                    /* Seperate each record into each field */
                    fields = line.Split(delimeters);

                    /* Validate the length of the line */
                    if (fields.Count() != 25)
                        continue;

                    TrainID = fields[11];
                    locoID = fields[9];

                    if (fields[24].Count() >= operatorStringLength)
                        subOperator = fields[24].Substring(0, operatorStringLength);
                    else
                        subOperator = fields[24].PadRight(operatorStringLength);

                    trainOperator = getOperator(subOperator);

                    commodity = getCommodity(fields[23]);

                    /* Ensure values are valid while reading them out. */
                    double.TryParse(fields[13], out speed);
                    double.TryParse(fields[12], out kmPost);
                    kmPost /= 1000;

                    double.TryParse(fields[15], out latitude);
                    double.TryParse(fields[16], out longitude);
                    DateTime.TryParse(fields[8], out dateTime);
                    double.TryParse(fields[19], out powerToWeight);

                    /* Check if the train is in the exclude list */
                    if (excludeListOfTrains)
                        includeTrain = !trainList.Contains(TrainID);
                    else
                    {
                        if (trainList.Count() > 0)
                            includeTrain = trainList.Contains(TrainID);
                        else
                            includeTrain = true;
                    }

                    /* Validate the speed data */
                    if (commodity.Equals(trainCommodity.Passenger))
                    {
                        if (speed < 0 || speed > maxPassengerSpeed)
                            includeTrain = false;
                    }
                    else
                    {
                        if (speed < 0 || speed > maxFreightSpeed)
                            includeTrain = false;
                    }

                    if (dateTime >= dateRange[0] && dateTime < dateRange[1] &&
                        includeTrain)
                    {
                        TrainRecord record = new TrainRecord(TrainID, locoID, dateTime, new GeoLocation(latitude, longitude), trainOperator, commodity, kmPost, speed, powerToWeight);
                        IceRecord.Add(record);
                    }

                }
            }

            /* Return the list of records. */
            return IceRecord;
        }

        /// <summary>
        /// Read the Azure data file and extract the neccessary information into a train record class.
        /// The data file is assumed to be created accessing the data through Azure datawarehouse, and then filtering out neccessary data points in Tableau.
        /// This process creates a new format different to the original datawarehouse.
        /// ------------------------------------------------------
        /// </summary>
        /// <param name="filename">Filename of the data file.</param>
        /// <param name="trainList">List of trains to exclude.</param>
        /// <param name="excludeListOfTrains">Boolean flag to indicate if the list of trains should be excluded or exclusively included.</param>
        /// <param name="dateRange">The dates between which to keep the data</param>
        /// <returns>List of train records describing each point in a trains journey.</returns>
        public static List<TrainRecord> readAzureExtractICEData(string filename, List<string> trainList, bool excludeListOfTrains, DateTime[] dateRange)
        {

            /* Read all the lines of the data file. */
            Tools.isFileOpen(filename);

            char[] delimeters = { '\t' };
            string[] fields = null;

            /* Minimum length of the operator string to distinguisg between them. */
            int operatorStringLength = 6;
            /* Set maximum speed values
             * There are occasionally datapoints that indicate the train is going faster than phyically possible 
             */
            double maxPassengerSpeed = 170;
            double maxFreightSpeed = 120;

            /* Initialise the fields of interest. */
            string TrainID = "none";
            string locoID = "none";
            string subOperator = "";
            trainOperator trainOperator = trainOperator.Unknown;
            trainCommodity commodity = trainCommodity.Unknown;
            double powerToWeight = 0.0;
            double speed = 0.0;
            double kmPost = 0.0;
            double latitude = 0.0;
            double longitude = 0.0;
            DateTime dateTime = DateTime.MinValue;

            bool header = true;
            bool includeTrain = true;

            /* List of all valid train data. */
            List<TrainRecord> IceRecord = new List<TrainRecord>();

            foreach (string line in System.IO.File.ReadLines(filename))
            {
                if (header)
                    /* Ignore the header line. */
                    header = false;
                else
                {
                    /* Seperate each record into each field */
                    fields = line.Split(delimeters);

                    /* Empty line reached */
                    if (fields[0].Equals(""))
                        continue;

                    /* Validate the length of the line */
                    if (fields.Count() != 24)
                        continue;

                    TrainID = fields[15];
                    locoID = fields[6];

                    if (fields[8].Count() >= operatorStringLength)
                        subOperator = fields[8].Substring(0, operatorStringLength);
                    else
                        subOperator = fields[8].PadRight(operatorStringLength);

                    trainOperator = getOperator(subOperator);

                    commodity = getCommodity(fields[9]);

                    /* Ensure values are valid while reading them out. */
                    double.TryParse(fields[22], out speed);
                    double.TryParse(fields[20], out kmPost);
                    kmPost /= 1000;
                    double.TryParse(fields[5], out latitude);
                    double.TryParse(fields[7], out longitude);
                    DateTime.TryParse(fields[4], out dateTime);
                    double.TryParse(fields[19], out powerToWeight);

                    /* Check if the train is in the exclude list */
                    if (excludeListOfTrains)
                        includeTrain = !trainList.Contains(TrainID);
                    else
                    {
                        if (trainList.Count() > 0)
                            includeTrain = trainList.Contains(TrainID);
                        else
                            includeTrain = true;
                    }

                    /* Validate the speed data */
                    if (commodity.Equals(trainCommodity.Passenger))
                    {
                        if (speed < 0 || speed > maxPassengerSpeed)
                            includeTrain = false;
                    }
                    else
                    {
                        if (speed < 0 || speed > maxFreightSpeed)
                            includeTrain = false;
                    }

                    if (dateTime >= dateRange[0] && dateTime < dateRange[1] &&
                        includeTrain)
                    {
                        TrainRecord record = new TrainRecord(TrainID, locoID, dateTime, new GeoLocation(latitude, longitude), trainOperator, commodity, kmPost, speed, powerToWeight);
                        IceRecord.Add(record);
                    }

                }
            }

            /* Return the list of records. */
            return IceRecord;
        }

        /// <summary>
        /// This function reads the Traxim simulation files and populates the simualtedTrain 
        /// data for comparison to the averaged ICE data.
        /// </summary>
        /// <param name="filename">The simulation filename.</param>
        /// <returns>The list of data for the simualted train.</returns>
        public static Train readSimulationData(string filename, Category simulationCategory, direction direction)
        {

            /* Read all the lines of the data file. */
            Tools.isFileOpen(filename);

            char[] delimeters = { ',', '\t' };
            string[] fields = null;

            /* Initialise the fields of interest. */
            double kilometreage = 0;
            double latitude = 0;
            double longitude = 0;
            double elevation = 0;
            double previousTime = 0;
            double time = 0;
            DateTime dateTime = DateTime.MinValue;
            double speed = 0;

            bool header = true;

            /* List of the simulated journey. */
            List<TrainJourney> simulatedJourney = new List<TrainJourney>();


            foreach (string line in System.IO.File.ReadLines(filename))
            {
                /* Seperate each record into each field */
                fields = line.Split(delimeters);

                if (header)
                {
                    header = false;
                }
                else
                {
                    /* Add the properties to their respsective fields. */
                    double.TryParse(fields[3], out elevation);
                    double.TryParse(fields[9], out speed);
                    double.TryParse(fields[14], out kilometreage);
                    double.TryParse(fields[0], out latitude);
                    double.TryParse(fields[2], out longitude);


                    if (double.TryParse(fields[8], out time))
                    {
                        if (dateTime == DateTime.MinValue)
                            dateTime = new DateTime(2016, 1, 1, 0, 0, 0);
                        else
                        {
                            dateTime = dateTime.AddSeconds(time - previousTime);
                            previousTime = time;
                        }
                    }

                    /* Add the record to the simulated journey. */
                    TrainJourney item = new TrainJourney(new GeoLocation(latitude, longitude), dateTime, speed, kilometreage, kilometreage, elevation);
                    simulatedJourney.Add(item);
                }
            }

            /* Create the simulated train. */
            Train simulatedTrain = new Train(simulatedJourney, simulationCategory, direction);

            /* Return the list of records. */
            return simulatedTrain;
        }

        /// <summary>
        /// This function reads the actual average performance file and extracts the 
        /// average train performance for each category
        /// </summary>
        /// <param name="filename">The actual Average performance file</param>
        /// <param name="numberOfCategories">The number of catagories expected in the file, This can included the combined category</param>
        /// <returns>A list of average trains to allow comparison of speeds through loops sections.</returns>
        public static List<Train> readSimulationData(string filename, int numberOfCategories)
        {
            /* Multiply the number of catagories by the number of direction for each category */
            numberOfCategories *= 2;

            /* Read all the lines of the data file. */
            Tools.isFileOpen(filename);

            char[] delimeters = { ',', '\t' };
            string[] fields = null;

            /* Initialise the fields of interest. */
            double elevation = 0;

            double previousKilometreage = 0;
            double kilometreage = 0;

            double speed = 0;
            double previousSpeed = 0;

            /* Track the averaged perfomance train properties. */
            List<Category> category = new List<Category>();
            List<direction> trainDirection = new List<direction>();

            /* Initialise the starting time for the averaged train. */
            DateTime time = new DateTime(2000, 1, 1);

            int count = 0;
            bool header = true;
            int offset = 2;

            /* List of the simulated journey. */
            List<double> kilometre = new List<double>();
            List<double> dictionarySpeeds = new List<double>();
            /* Dictionary of speeds for each kilometre value */
            Dictionary<double, List<double>> journeySpeeds = new Dictionary<double, List<double>>();

            /* Lists to contain the actual average train performance and train properties. */
            List<TrainJourney> actualPerformance = new List<TrainJourney>();
            List<List<TrainJourney>> trains = new List<List<TrainJourney>>();

            /* Read each line of the file. */
            foreach (string line in System.IO.File.ReadLines(filename))
            {
                /* Seperate each record into each field */
                fields = line.Split(delimeters);

                if (header)
                {
                    /* Extract the train characteristics. */
                    trainOperator trainOperator = trainOperator.Unknown;

                    /* Extract each of the train properties. */
                    if (count == 0)
                    {
                        for (int categoryIdx = 0; categoryIdx < numberOfCategories; categoryIdx++)
                        {
                            trainOperator = getOperator(fields[offset + categoryIdx].Split(' ')[0]);
                            category.Add(Processing.convertTrainOperatorToCategory(trainOperator));
                            trainDirection.Add(getDirection(fields[offset + categoryIdx].Split(' ')[1]));
                        }
                    }

                    /* Check if we have reached the actual data values. */
                    count++;
                    if (count >= 9)
                        header = false;
                }
                else
                {
                    /* Add the properties to their respsective fields. */
                    double.TryParse(fields[0], out kilometreage);
                    double.TryParse(fields[1], out elevation);

                    /* Clear the list of speeds. */
                    dictionarySpeeds.Clear();
                    for (int categoryIdx = 0; categoryIdx < numberOfCategories; categoryIdx++)
                    {
                        /* Extract the speeds from the file. */
                        double.TryParse(fields[offset + categoryIdx], out speed);
                        dictionarySpeeds.Add(speed);
                    }
                    /* Add the list of speeds for the current kilometreage. */
                    journeySpeeds.Add(kilometreage, new List<double>(dictionarySpeeds));
                    kilometre.Add(kilometreage);
                }
            }

            /* Transpose the journey speeds into individual train journeys. */
            speed = 0;
            for (int categoryIdx = 0; categoryIdx < numberOfCategories; categoryIdx++)
            {
                actualPerformance.Clear();
                for (int journeyIdx = 0; journeyIdx < journeySpeeds.Count(); journeyIdx++)
                {
                    /* Extract the neccesasry journey parameters for the specified train. */
                    kilometreage = kilometre[journeyIdx];
                    speed = journeySpeeds[kilometreage][categoryIdx];

                    if (journeyIdx > 0)
                    {
                        previousKilometreage = kilometre[journeyIdx - 1];
                        previousSpeed = journeySpeeds[previousKilometreage][categoryIdx];
                    }
                    else
                    {
                        previousKilometreage = kilometreage;
                        previousSpeed = speed;
                    }

                    /* Calcualte the time according to the direction of travel. */
                    if (trainDirection[categoryIdx] == direction.IncreasingKm)
                        time = time.AddHours(Processing.calculateTimeInterval(previousKilometreage, kilometreage, previousSpeed));
                    else
                        time = time.AddHours(-1 * Processing.calculateTimeInterval(previousKilometreage, kilometreage, speed));

                    /* Create and add the journey point to the trains journy. */
                    TrainJourney item = new TrainJourney(new GeoLocation(), time, speed, kilometreage, kilometreage, elevation);
                    actualPerformance.Add(item);
                }
                /* Add the train journey to the list. */
                trains.Add(new List<TrainJourney>(actualPerformance));
            }

            /* Create the list of actual average trains. */
            List<Train> actualAverageTrains = new List<Train>();

            /* Add each train journy to the actual average train list with the appropriate properties. */
            for (int categoryIdx = 0; categoryIdx < numberOfCategories; categoryIdx++)
                actualAverageTrains.Add(new Train(trains[categoryIdx], category[categoryIdx], trainDirection[categoryIdx]));


            /* Return the list of records. */
            return actualAverageTrains;
        }

        /// <summary>
        /// This function reads the file with the list of trains to exclude from the 
        /// data and stores the list in a managable list object.
        /// The file is assumed to have one train per line or have each train seperated 
        /// by a common delimiter [ , \ " \t \n]
        /// </summary>
        /// <param name="filename">The full path of the file containing the list of trains to exclude.</param>
        /// <returns>The populated list of all trains to exclude.</returns>
        public static List<string> readTrainList(string filename)
        {
            List<string> excludeTrainList = new List<string>();

            /* Read all the lines of the file. */
            Tools.isFileOpen(filename);

            string[] lines = System.IO.File.ReadAllLines(filename);
            char[] delimeters = { ',', '\t', '\n' };

            /* Seperate the fields. */
            string[] fields = lines[0].Split(delimeters);

            /* Add the trains to the list. */
            foreach (string line in lines)
                excludeTrainList.Add(line);

            return excludeTrainList;
        }

        /// <summary>
        /// Function reads in the track geometry data from file.
        /// </summary>
        /// <param name="filename">Full filename of the geometry file.</param>
        /// <returns>A list of track Geometry objects describing the track geometry.</returns>
        public static List<TrackGeometry> readGeometryfile(string filename)
        {
            Processing processing = new Processing();

            /* Create the list of track geometry objects. */
            List<TrackGeometry> trackGeometry = new List<TrackGeometry>();

            bool header = true;

            /* Read all the lines of the file. */
            char[] delimeters = { ',', '\t' };

            /* Seperate the fields. */
            string[] fields = null;

            bool firstPoint = true;

            /* Define the track geomerty parameters. */
            string geometryName = null;
            double latitude = 0.0;
            double longitude = 0.0;
            double elevation = 0.0;
            double kilometreage = 0.0;
            double virtualKilometreage = 0.0;
            bool isLoopHere = false;

            /* Define some additional helper parameters. */
            double distance = 0;
            direction direction = direction.Unknown;
            double previousLat = 0;
            double previousLong = 0;
            double previouskm = 0;
            string loop;

            /* Add the trains to the list. */
            foreach (string line in System.IO.File.ReadLines(filename))
            {
                if (header)
                    /* Ignore the header line. */
                    header = false;
                else
                {
                    /* Seperate each record into each field */
                    fields = line.Split(delimeters);
                    geometryName = fields[0];
                    double.TryParse(fields[1], out latitude);
                    double.TryParse(fields[2], out longitude);
                    double.TryParse(fields[3], out elevation);
                    double.TryParse(fields[4], out kilometreage);
                    loop = fields[6];

                    if (loop.Equals("loop", StringComparison.OrdinalIgnoreCase) || loop.Equals("signal", StringComparison.OrdinalIgnoreCase))
                        isLoopHere = true;
                    else
                        isLoopHere = false;

                    /* The virtual kilometreage starts at the first kilometreage of the track. */
                    if (firstPoint)
                    {
                        virtualKilometreage = kilometreage;
                        /* Set the 'pervious' parameters. */
                        previousLat = latitude;
                        previousLong = longitude;
                        previouskm = kilometreage;
                        firstPoint = false;
                    }
                    else
                    {
                        /* Determine the direction for the track kilometreage. */
                        if (direction == direction.Unknown)
                        {
                            if (kilometreage - previouskm > 0)
                                direction = direction.IncreasingKm;
                            else
                                direction = direction.DecreasingKm;
                        }

                        /* Calcualte the distance between succesive points and increment the virtual kilometreage. */
                        distance = Processing.calculateGreatCircleDistance(previousLat, previousLong, latitude, longitude);

                        if (direction == direction.IncreasingKm)
                            virtualKilometreage = virtualKilometreage + distance * Processing.metresToKilometers;

                        else
                            virtualKilometreage = virtualKilometreage - distance * Processing.metresToKilometers;

                        /* Set the 'previous' parameters. */
                        previousLat = latitude;
                        previousLong = longitude;

                    }

                    /* Add the geometry point to the list. */
                    TrackGeometry geometry = new TrackGeometry(0, geometryName, latitude, longitude, elevation, kilometreage, virtualKilometreage, isLoopHere);
                    trackGeometry.Add(geometry);

                }
            }


            return trackGeometry;
        }

        /// <summary>
        /// Read the wagon data file.
        /// The file assumes the data has been extracted from Tableau 
        /// and hence has a specific file format.
        /// Column  Field
        ///  0      Commodity
        ///  1      Origin
        ///  2      Planned Destination
        ///  3      Attachment Time
        ///  4      Class
        ///  5      Destination
        ///  6      Detatchment Time
        ///  7      Class Number
        ///  8      Train Operator
        ///  9      Train Date
        ///  10     Train ID
        ///  11     Distance
        ///  12     Gross Mass
        ///  13     Move Count
        ///  14     Power Ratio
        ///  15     Tare Mass
        /// </summary>
        /// <param name="filename">The wagon data file.</param>
        /// <param name="combineIntermodalAndSteel">A flag indicating if the Intermodal and Steel commodities should be combined 
        /// into an interstate commodity for analysis.</param>
        /// <returns>The list of wagon objects.</returns>
        public static List<wagonDetails> readWagonDataFile(string filename, bool combineIntermodalAndSteel = false)
        {
            /* Read the all lines of the text file. */
            char[] delimiters = { '\t' };
            bool header = true;

            DateTime trainDate = DateTime.MinValue;
            DateTime attachmentTime = DateTime.MinValue;
            DateTime detachmentTime = DateTime.MinValue;
            double tareWeight = 0;
            double grossWeight = 0;
            string wagonID = null;
            string origin = null;
            string plannedDestination = null;
            string destination = null;
            double weight = 0;

            /* Create the list of wagon objects. */
            List<wagonDetails> wagon = new List<wagonDetails>();

            /* Validate the format of the first line of the file, ignoring the header information */
            bool validFormat = false;
            string[] fields = null;

            /* Read the first line of data - assum e first line has header information. */
            fields = System.IO.File.ReadLines(filename).Skip(1).First().Split(delimiters);

            validFormat = Tools.validateFileFormat(fields);
            if (!validFormat)
            {
                /* The file format is invalid, return the empty wagon object. */
                throw new IOException("Data file has an invalid format.");
            }

            /* Extract the wagon details from the data file. */
            foreach (string line in System.IO.File.ReadLines(filename))
            {
                if (header)
                {
                    header = false;
                }
                else
                {
                    /* Split the line into the fields */
                    fields = line.Split(delimiters);

                    /* Extract the train related information. */
                    string trainID = fields[10];
                    DateTime.TryParse(fields[9], out trainDate);
                    trainOperator trainOperator = Processing.getWagonOperator(fields[8]);
                    trainCommodity commodity = Processing.getWagonCommodity(fields[4]);

                    /* Include Intermodal and Steel commodity into the Interstate commodity, when required. */
                    if (combineIntermodalAndSteel)
                        if (commodity.Equals(trainCommodity.Intermodal) || commodity.Equals(trainCommodity.Steel))
                            commodity = trainCommodity.Interstate;

                    /* Extract the wagon Identification. */
                    wagonID = fields[3] + " " + Regex.Replace(fields[7], ",", "");

                    /* Wagon Origin. */
                    origin = fields[0].ToUpper();
                    if (origin.Count() != 3)
                        Tools.messageBox("Origin location code is unknown: " + origin + " Unknown location code.");

                    /* Wagon Destination */
                    destination = fields[5].ToUpper();

                    /* Wagon planned destination. */
                    plannedDestination = fields[1].ToUpper();
                    if (plannedDestination.Count() != 3)
                    {
                        if (fields[5].Count() == 3)
                            plannedDestination = fields[5].ToUpper();
                        else
                            Tools.messageBox("Consigned Destination location code in unknown: Train: " + trainID + ", location " + plannedDestination + " Unknown location code.");
                    }

                    /* Wagon destination. */
                    destination = fields[5].ToUpper();
                    if (destination.Count() != 3)
                    {   /* If the destination field is empty, assume the wagon reaches the planned destination. */
                        if (destination.Equals(""))
                            destination = plannedDestination;
                        else
                            Tools.messageBox("Destination location code is unknown: " + destination + " Unknown location code.");
                    }

                    /* Extract remaining wagon details. */
                    DateTime.TryParse(fields[2], out attachmentTime);
                    DateTime.TryParse(fields[6], out detachmentTime);
                    double.TryParse(fields[15], out tareWeight);
                    double.TryParse(fields[12], out grossWeight);

                    /* Net weight */
                    weight = grossWeight - tareWeight;

                    if (weight < 0)
                        weight = 0;

                    /* Construct the wagon object and add to the list. */
                    wagonDetails data = new wagonDetails(trainID, trainDate, trainOperator, commodity, wagonID, origin, plannedDestination, destination, attachmentTime, detachmentTime, weight, grossWeight);
                    wagon.Add(data);

                }
            }
            /* Return the completed wagon List. */
            return wagon;
        }

        /// <summary>
        /// Read the wagon data file that is assumed to be collected from the Azure datawarehouse.
        /// This will have a difference format than previous data collection, and is likley to apply to 
        /// any data collected after 01/07/2018.
        /// </summary>
        /// <param name="filename">The wagon data file.</param>
        /// <param name="combineIntermodalAndSteel">A flag indicating if the Intermodal and Steel commodities should be combined 
        /// into an interstate commodity for analysis.</param>
        /// <returns>The list of wagon objects.</returns>
        public static List<wagonDetails> readAzureWagonDataFile(string filename, bool combineIntermodalAndSteel = false)
        {
            /* Read the all lines of the text file. */
            char[] delimiters = { ',', '\t' };

            bool header = true;

            DateTime trainDate = DateTime.MinValue;
            DateTime attachmentTime = DateTime.MinValue;
            DateTime detachmentTime = DateTime.MinValue;
            double tareWeight = 0;
            double grossWeight = 0;
            string wagonID = null;
            string origin = null;
            string plannedDestination = null;
            string destination = null;
            double weight = 0;

            int count = 0;

            /* Create the list of wagon objects. */
            List<wagonDetails> wagon = new List<wagonDetails>();

            string[] fields = null;

            /* Read the first line of data - assume first line has header information. */
            fields = System.IO.File.ReadLines(filename).Skip(1).First().Split(delimiters);

            /* Extract the wagon details from the data file. */
            foreach (string line in System.IO.File.ReadLines(filename))
            {
                count++;
                if (header)
                {
                    header = false;
                }
                else
                {

                    if (line.Equals("") || line.Contains("rows"))
                        continue;

                    /* Split the line into the fields */
                    fields = line.Split(delimiters);

                    /* Extract the train related information. */
                    string trainID = fields[1];
                    DateTime.TryParse(fields[0], out trainDate);
                    trainOperator trainOperator = Processing.getWagonOperator(fields[15]);
                    trainCommodity commodity = Processing.getWagonCommodity(fields[14]);

                    /* Include Intermodal and Steel commodity into the Interstate commodity, when required. */
                    if (combineIntermodalAndSteel)
                        if (commodity.Equals(trainCommodity.Intermodal) || commodity.Equals(trainCommodity.Steel))
                            commodity = trainCommodity.Interstate;

                    /* Extract the wagon Identification. */
                    wagonID = fields[2] + " " + Regex.Replace(fields[10], ",", "");

                    /* Wagon Origin. */
                    origin = fields[4].ToUpper();
                    if (origin.Count() != 3)
                        Tools.messageBox("Origin location code is unknown: " + origin + " Unknown location code.");

                    /* Wagon planned destination. */
                    plannedDestination = fields[5].ToUpper();
                    if (plannedDestination.Count() != 3)
                    {
                        if (fields[13].Count() == 3)
                            plannedDestination = fields[13].ToUpper();
                        else
                            Tools.messageBox("Consigned Destination location code is unknown: Train: " + trainID + ", location " + plannedDestination + " Unknown location code.");
                    }

                    /* Wagon destination. */
                    destination = fields[13].ToUpper();
                    if (destination.Count() != 3)
                    {   /* If the destination field is empty or set to -1, assume the wagon reaches the planned destination. */
                        if (destination.Equals("") || destination.Equals("-1"))
                            destination = plannedDestination;
                        else
                            Tools.messageBox("Destination location code is unknown: " + destination + " Unknown location code.");
                    }

                    /* Extract remaining wagon details. */
                    DateTime.TryParse(fields[6], out attachmentTime);
                    DateTime.TryParse(fields[7], out detachmentTime);
                    double.TryParse(fields[12], out tareWeight);
                    double.TryParse(fields[8], out grossWeight);

                    /* Net weight */
                    weight = grossWeight - tareWeight;

                    if (weight < 0)
                        weight = 0;

                    /* Construct the wagon object and add to the list. */
                    wagonDetails data = new wagonDetails(trainID, trainDate, trainOperator, commodity, wagonID, origin, plannedDestination, destination, attachmentTime, detachmentTime, weight, grossWeight);
                    wagon.Add(data);

                }
            }
            /* Return the completed wagon List. */
            return wagon;
        }

        /// <summary>
        /// Read the wagon data directly from the Azure data warehouse using an SQL connection.
        /// </summary>
        /// <param name="fromDate">The start date of the analysis period.</param>
        /// <param name="toDate">The end data of the analysis period.</param>
        /// <param name="combineIntermodalAndSteel">A flag indicating whether to combine intermodal and steel into a single commodity.</param>
        /// <returns>A list containing all the wagon data.</returns>
        public static List<wagonDetails> readSQLWagonData(DateTime fromDate, DateTime toDate, bool combineIntermodalAndSteel = false)
        {
            /* Create the list of wagon objects. */
            List<wagonDetails> wagon = new List<wagonDetails>();

            string trainID, wagonID, origin, destination, plannedDestination;
            trainOperator trainOperator;
            trainCommodity commodity;
            double tareWeight, grossWeight, weight;
            DateTime trainDate, attachmentTime, detachmentTime;

            /* Generate the SQL connection and the command to execute. */
            SqlParameters SQL = generateSQLComand(fromDate, toDate);

            /* Create an SQL environment. */
            using (SQL.connection)
            {
                SQL.connection.Open();
                /* Process the SQL command. */
                SqlDataReader reader = SQL.command.ExecuteReader();

                try
                {
                    /* Read each line into the desired wagon object. */
                    while (reader.Read())
                    {

                        /* Train related information */
                        trainDate = (DateTime)reader.GetSqlValue(0);
                        trainID = reader.GetSqlValue(1).ToString();
                        commodity = Processing.getWagonCommodity(reader.GetSqlValue(14).ToString());
                        trainOperator = Processing.getWagonOperator(reader.GetSqlValue(15).ToString());

                        /* Include Intermodal and Steel commodity into the Interstate commodity, when required. */
                        if (combineIntermodalAndSteel)
                            if (commodity.Equals(trainCommodity.Intermodal) || commodity.Equals(trainCommodity.Steel))
                                commodity = trainCommodity.Interstate;

                        /* Extract Wagon Identification. */
                        wagonID = reader.GetSqlValue(2).ToString() + " " + Regex.Replace(reader.GetSqlValue(10).ToString(), ",", "");

                        /* Wagon Origin */
                        origin = reader.GetSqlValue(4).ToString().ToUpper();
                        if (origin.Count() != 3)
                            Tools.messageBox("Origin location code is unknown: " + origin + " Unknown location code.");

                        /* Wagon Destination */
                        destination = reader.GetSqlValue(13).ToString().ToUpper();
                        if (destination.Count() != 3)
                        {   /* If the destination field is empty or set to -1, assume the wagon reaches the planned destination. */
                            if (destination.Equals("") || destination.Equals("-1"))
                                destination = reader.GetSqlValue(5).ToString().ToUpper();
                            else
                                Tools.messageBox("Destination location code is unknown: " + destination + " Unknown location code.");
                        }

                        /* Wagon planned destination */
                        plannedDestination = reader.GetSqlValue(5).ToString().ToUpper();
                        if (plannedDestination.Count() != 3)
                        {
                            if (destination.Count() == 3)
                                plannedDestination = destination;
                            else
                                Tools.messageBox("Consigned Destination location code is unknown: Train: " + trainID + ", location " + plannedDestination + " Unknown location code.");
                        }

                        double.TryParse(reader.GetSqlValue(12).ToString(), out tareWeight);
                        double.TryParse(reader.GetSqlValue(8).ToString(), out grossWeight);
                        weight = grossWeight - tareWeight;
                        /* Validate the weight. */
                        if (weight < 0)
                            weight = 0;

                        DateTime.TryParse(reader.GetSqlValue(6).ToString(), out attachmentTime);
                        DateTime.TryParse(reader.GetSqlValue(7).ToString(), out detachmentTime);

                        /* Construct the wagon object and add to the list. */
                        wagonDetails data = new wagonDetails(trainID, trainDate, trainOperator, commodity, wagonID, origin, plannedDestination, destination, attachmentTime, detachmentTime, weight, grossWeight);
                        wagon.Add(data);


                    }

                }
                finally
                {
                    /* Close the SQL command */
                    reader.Close();
                }
            }

            /* Return the completed wagon list. */
            return wagon;

        }

        /// <summary>
        /// Create the connection and generate the command to extract the wagon data.
        /// </summary>
        /// <param name="fromDate">The start data of the analysis period.</param>
        /// <param name="toDate">The end date of the analysis period.</param>
        /// <returns>An SQL object that contains the connection and the command to execute.</returns>
        private static SqlParameters generateSQLComand(DateTime fromDate, DateTime toDate)
        {
            /* Format the date strings to match data database fields. */
            string from = fromDate.ToString("yyyy-MM-dd");
            string to = toDate.ToString("yyyy-MM-dd");

            /* Generate the connecton parameters for the Azure data warehouse. */
            string server = "tcp:artc-dwcp01.database.windows.net,1433;";
            string Dbase = "DW_PRN;";
            string authentication = "Active Directory Integrated;";
            string options = "Encrypt=yes;Connection Timeout=30;";
            string connectionString = "Server=" + server + "Database=" + Dbase + "Authentication=" + authentication + options;

            /* Generate the SQL wagon data query */
            string wagonMovementQuery =
                "SELECT [Consist].[TrainDate] AS [TrainDate]," +
                "  [Consist].[TrainCode] AS [TrainCode]," +
                "  [Consist].[VehicleClassCode] AS [VehicleClassCode]," +
                "  [Consist].[VehicleID] AS [VehicleID]," +
                "  [Consist].[OriginParentLocationCode] AS [OriginParentLocationCode]," +
                "  [Consist].[PlannedDestinationParentLocationCode] AS [PlannedDestinationParentLocationCode], " +
                "  [Consist].[ActualAttachDateTime] AS [ActualAttachDateTime]," +
                "  [Consist].[ActualDetachDateTime] AS [ActualDetachDateTime], " +
                "  [Consist].[GrossMass] AS [GrossMass]," +
                "  [Consist].[Distance] AS [Distance]," +
                "  [Vehicle].[VehicleNumber] AS [VehicleNumber(Vehicle)]," +
                "  [Vehicle].[HorsePower] AS [HorsePower]," +
                "  [Vehicle].[TareMass] AS [TareMass]," +
                "  [Actual Destination Location].[ParentLocationCode] AS [ParentLocationCode]," +
                "  [Commodity].[RAMS_CommodityCode] AS [RAMS_CommodityCode]," +
                "  [Operator].[OperatorCode] AS [OperatorCode(Operator)] " +
                "FROM[FACT].[Consist] [Consist]" +
                "  INNER JOIN [FACT].[TrainJourney] [TrainJourney] ON ([Consist].[TrainID] = [TrainJourney].[TrainID])" +
                "  INNER JOIN [DIM].[Vehicle] [Vehicle] ON ([Consist].[VehicleID] = [Vehicle].[VehicleID])" +
                "  INNER JOIN [REPORT].[ParentLocation] [Actual Destination Location] ON ([Consist].[ActualDestinationParentLocationID] = [Actual Destination Location].[ParentLocationID])" +
                "  INNER JOIN [REPORT].[Commodity] [Commodity] ON ([TrainJourney].[CommodityID] = [Commodity].[CommodityID])" +
                "  INNER JOIN [DIM].[Operator] [Operator] ON ([TrainJourney].[OperatorID] = [Operator].[OperatorID]) " +
                "WHERE[Consist].[TrainDate] BETWEEN '" + from + "' AND '" + to + "'";

            /* Create the connection and the command to execute. */
            SqlConnection connection = new SqlConnection(connectionString);
            SqlCommand command = new SqlCommand(wagonMovementQuery, connection);

            SqlParameters sql = new SqlParameters(connection, command);

            return sql;

        }

        /// <summary>
        /// Read the file containing the temporary speed restriction information and 
        /// store in a manalgable list of TSR objects, which contain all neccessary 
        /// information for each TSR.
        /// </summary>
        /// <param name="filename">TSR file</param>
        /// <returns>List of TSR objects contianting the parameters for each TSR.</returns>
        public static List<TSRObject> readTSRFile(string filename, DateTime[] dateRange)
        {
            /* Read all the lines of the data file. */
            Tools.isFileOpen(filename);

            string[] lines = System.IO.File.ReadAllLines(filename);
            char[] delimeters = { ',', '\t' };

            /* Seperate the fields. */
            string[] fields = lines[0].Split(delimeters);

            /* Initialise the fields of interest. */
            string region = "none";
            DateTime issueDate = DateTime.MinValue;
            DateTime liftedDate = DateTime.MinValue;
            double startKm = 0.0;
            double endKm = 0.0;
            double speed = 0.0;

            bool header = true;

            /* List of all TSR details. */
            List<TSRObject> TSRList = new List<TSRObject>();

            foreach (string line in lines)
            {
                if (header)
                    /* Ignore the header line. */
                    header = false;
                else
                {
                    /* Seperate each record into each field */
                    fields = line.Split(delimeters);

                    region = fields[0];
                    /* needs to perform tests */
                    DateTime.TryParse(fields[1], out issueDate);
                    DateTime.TryParse(fields[2], out liftedDate);
                    double.TryParse(fields[10], out startKm);
                    double.TryParse(fields[11], out endKm);
                    double.TryParse(fields[5], out speed);

                    /* Set the lift date if the TSR applies the full time period. */
                    if (liftedDate == DateTime.MinValue)
                        liftedDate = dateRange[1];

                    /* Add the TSR properties that are within the period of analysis. */
                    if (issueDate < dateRange[1] && liftedDate >= dateRange[0])
                    {
                        TSRObject record = new TSRObject(region, issueDate, liftedDate, startKm, endKm, speed);
                        TSRList.Add(record);
                    }

                }
            }

            /* Return the list of TSR records. */
            return TSRList;
        }

        /// <summary>
        /// This function writes each interpolated train journey to an individual column in excel.
        /// This can be used to compare against previously completed corridor analysis for validation.
        /// </summary>
        /// <param name="trainRecords">The train records containing the individual train properties.</param>
        /// <param name="startKm">The start point of the interpolation</param>
        /// <param name="interpoaltionInterval">The interpolation interval in m</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void writeTrainData(List<Train> trainRecords, double startKm, double interpoaltionInterval, string aggregatedDestination)
        {

            /* Create the microsfot excel references. */
            _Workbook workbook;
            _Worksheet worksheet;

            /* Start Excel and get Application object. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();

            /* Get the reference to the new workbook. */
            workbook = (Microsoft.Office.Interop.Excel._Workbook)(excel.Workbooks.Add(""));

            /* Create the header details. */
            string[,] headerString = {{ "km", "", "Trains:" },
                                     { "", "Train ID:", "" },
                                     { "", "Loco ID:", "" },
                                     { "", "Date:", "" },
                                     { "", "Power to Weight Ratio:", "" },
                                     { "", "Commodity:", "" },
                                     { "", "Operator", ""},
                                     { "", "Direction:", "" }};


            /* Pagenate the data for writing to excel. */
            int excelPageSize = 1000000;        /* Page size of the excel worksheet. */
            int excelPages = 1;                 /* Number of Excel pages to write. */
            int headerOffset = 10;

            /* Adjust the excel page size or the number of pages to write. */
            if (trainRecords.Count() < excelPageSize)
                excelPageSize = trainRecords.Count();
            else
                excelPages = (int)Math.Round((double)trainRecords.Count() / excelPageSize + 0.5);

            /* Deconstruct the train details into excel columns. */
            string[,] TrainID = new string[1, trainRecords.Count()];
            string[,] LocoID = new string[1, trainRecords.Count()];
            double[,] powerToWeight = new double[1, trainRecords.Count()];
            string[,] commodity = new string[1, trainRecords.Count()];
            string[,] trainOperator = new string[1, trainRecords.Count()];
            string[,] direction = new string[1, trainRecords.Count()];
            DateTime[,] dateTime = new DateTime[1, trainRecords.Count()];
            double[,] kilometerage = new double[trainRecords[0].journey.Count(), 1];

            double[,] speed = new double[trainRecords[0].journey.Count(), trainRecords.Count()];

            int headerRows = headerString.GetLength(0);
            int headerColumns = headerString.GetLength(1);

            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Set the active worksheet. */
                if (excelPage >= 3)
                    worksheet = workbook.Worksheets.Add();
                else
                    worksheet = workbook.Sheets[excelPage + 1];

                workbook.Sheets[excelPage + 1].Activate();
                Range topLeft = worksheet.Cells[1, 1];
                Range bottomRight = worksheet.Cells[headerRows, headerColumns];
                worksheet.get_Range(topLeft, bottomRight).Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int trainIdx = 0; trainIdx < trainRecords.Count(); trainIdx++)
                {
                    TrainID[0, trainIdx] = trainRecords[trainIdx].trainID;
                    LocoID[0, trainIdx] = trainRecords[trainIdx].locoID;

                    /* Extract the earliest date in the journey to represent the train date. */
                    dateTime[0, trainIdx] = trainRecords[trainIdx].journey.Where(t => t.dateTime > DateTime.MinValue).ToList().Min(t => t.dateTime);

                    powerToWeight[0, trainIdx] = trainRecords[trainIdx].powerToWeight;

                    commodity[0, trainIdx] = trainRecords[trainIdx].commodity.ToString();

                    trainOperator[0, trainIdx] = trainRecords[trainIdx].trainOperator.ToString();

                    direction[0, trainIdx] = trainRecords[trainIdx].trainDirection.ToString();

                    for (int journeyIdx = 0; journeyIdx < trainRecords[trainIdx].journey.Count(); journeyIdx++)
                    {
                        kilometerage[journeyIdx, 0] = startKm + interpoaltionInterval * Processing.metresToKilometers * journeyIdx;

                        speed[journeyIdx, trainIdx] = trainRecords[trainIdx].journey[journeyIdx].speed;

                    }
                }

                /* Write the data to the active excel workseet. */
                worksheet.Range[worksheet.Cells[2, 3], worksheet.Cells[2, trainRecords.Count() + 2]].Value2 = TrainID;
                worksheet.Range[worksheet.Cells[3, 3], worksheet.Cells[3, trainRecords.Count() + 2]].Value2 = LocoID;
                worksheet.Range[worksheet.Cells[4, 3], worksheet.Cells[4, trainRecords.Count() + 2]].Value2 = dateTime;
                worksheet.Range[worksheet.Cells[5, 3], worksheet.Cells[5, trainRecords.Count() + 2]].Value2 = powerToWeight;
                worksheet.Range[worksheet.Cells[6, 3], worksheet.Cells[6, trainRecords.Count() + 2]].Value2 = commodity;
                worksheet.Range[worksheet.Cells[7, 3], worksheet.Cells[7, trainRecords.Count() + 2]].Value2 = trainOperator;
                worksheet.Range[worksheet.Cells[8, 3], worksheet.Cells[8, trainRecords.Count() + 2]].Value2 = direction;

                worksheet.Range[worksheet.Cells[headerOffset, 1], worksheet.Cells[headerOffset + trainRecords[0].journey.Count() - 1, 1]].Value2 = kilometerage;
                worksheet.Range[worksheet.Cells[headerOffset, 3], worksheet.Cells[headerOffset + trainRecords[0].journey.Count() - 1, 3 + trainRecords.Count() - 1]].Value2 = speed;

            }

            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\ICEData_InterpolatedTrains" + DateTime.Now.ToString("yyyyMMdd_HHmm") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }


            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing, false, false,
                XlSaveAsAccessMode.xlNoChange, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;
        }

        /// <summary>
        /// This function writes the train journey information (km, speed and time) to an individual column in excel. 
        /// This is intended to visualise the raw data prio to cleaning and interpolation. To write this information 
        /// for the interpolated data, use the writeTrainDataWithTime function.
        /// </summary>
        /// <param name="trainRecords">The train records containing the individual train properties.</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void writeRawTrainDataWithTime(List<Train> trainRecords, string aggregatedDestination)
        {

            /* Create the microsfot excel references. */
            _Workbook workbook;
            _Worksheet worksheet;

            /* Start Excel and get Application object. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();

            /* Get the reference to the new workbook. */
            workbook = (Microsoft.Office.Interop.Excel._Workbook)(excel.Workbooks.Add(""));

            /* Create the header details. */
            string[,] headerString = {{ "km", "", "Trains:" },
                                     { "", "Train ID:", "" },
                                     { "", "Loco ID:", "" },
                                     { "", "Date:", "" },
                                     { "", "Power to Weight Ratio:", "" },
                                     { "", "Commodity:", "" },
                                     { "", "Direction:", "" }};


            /* Pagenate the data for writing to excel. */
            int columnOffset = 4;

            int excelPageSize = (int)16380 / columnOffset;          /* Max Columns in the excel worksheet. */
            int excelPages = 1;                 /* Number of Excel pages to write. */

            int headerOffset = 9;
            int horizontalOffset = 3;

            int headerRows = headerString.GetLength(0);
            int headerColumns = headerString.GetLength(1);

            int displayColumn = horizontalOffset;

            /* Adjust the excel page size or the number of pages to write. */
            if (trainRecords.Count() < excelPageSize)
                excelPageSize = trainRecords.Count();
            else
                excelPages = (int)Math.Round((double)trainRecords.Count() / excelPageSize + 0.5);

            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Deconstruct the train details into excel columns. */
                string[,] TrainID = new string[1, excelPageSize * columnOffset];
                string[,] LocoID = new string[1, excelPageSize * columnOffset];
                double[,] powerToWeight = new double[1, excelPageSize * columnOffset];
                string[,] commodity = new string[1, excelPageSize * columnOffset];
                string[,] direction = new string[1, excelPageSize * columnOffset];
                DateTime[,] trainDate = new DateTime[1, excelPageSize * columnOffset];

                horizontalOffset = 3;
                displayColumn = horizontalOffset;

                /* Set the active worksheet. */
                if (excelPage >= 3)
                    worksheet = workbook.Worksheets.Add();
                else
                    worksheet = workbook.Sheets[excelPage + 1];

                workbook.Sheets[excelPage + 1].Activate();
                Range topLeft = worksheet.Cells[1, 1];
                Range bottomRight = worksheet.Cells[headerRows, headerColumns];
                worksheet.get_Range(topLeft, bottomRight).Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int trainIdx = 0; trainIdx < excelPageSize; trainIdx++)
                {
                    /* Calucalte the appropriate index for the train. */
                    int offsetIndex = excelPage * excelPageSize + trainIdx;

                    if (offsetIndex < trainRecords.Count())
                    {
                        int displayRow = headerOffset + trainRecords[offsetIndex].journey.Count() - 1;

                        /* Create new arrays for each parameter for each train. */
                        double[,] kilometerage = new double[trainRecords[offsetIndex].journey.Count(), 1];

                        double[,] speed = new double[trainRecords[offsetIndex].journey.Count(), 1];
                        DateTime[,] dateTime = new DateTime[trainRecords[offsetIndex].journey.Count(), 1];

                        /* Populate the train properties. */
                        TrainID[0, trainIdx * columnOffset] = trainRecords[offsetIndex].trainID;
                        LocoID[0, trainIdx * columnOffset] = trainRecords[offsetIndex].locoID;
                        trainDate[0, trainIdx * columnOffset] = trainRecords[offsetIndex].journey.Where(t => t.dateTime > DateTime.MinValue).ToList().Min(t => t.dateTime);

                        powerToWeight[0, trainIdx * columnOffset] = trainRecords[offsetIndex].powerToWeight;

                        commodity[0, trainIdx * columnOffset] = trainRecords[offsetIndex].commodity.ToString();

                        direction[0, trainIdx * columnOffset] = trainRecords[offsetIndex].trainDirection.ToString();

                        /* Fill in the empy spaces where the speed and time columns correspond to. */
                        TrainID[0, trainIdx * columnOffset + 1] = "";
                        LocoID[0, trainIdx * columnOffset + 1] = "";
                        trainDate[0, trainIdx * columnOffset + 1] = DateTime.MinValue;
                        powerToWeight[0, trainIdx * columnOffset + 1] = 0;
                        commodity[0, trainIdx * columnOffset + 1] = "";
                        direction[0, trainIdx * columnOffset + 1] = "";

                        TrainID[0, trainIdx * columnOffset + 2] = "";
                        LocoID[0, trainIdx * columnOffset + 2] = "";
                        trainDate[0, trainIdx * columnOffset + 2] = DateTime.MinValue;
                        powerToWeight[0, trainIdx * columnOffset + 2] = 0;
                        commodity[0, trainIdx * columnOffset + 2] = "";
                        direction[0, trainIdx * columnOffset + 2] = "";

                        /* Populate the train journey parameters. */
                        for (int journeyIdx = 0; journeyIdx < trainRecords[offsetIndex].journey.Count(); journeyIdx++)
                        {
                            kilometerage[journeyIdx, 0] = trainRecords[offsetIndex].journey[journeyIdx].kilometreage;

                            speed[journeyIdx, 0] = trainRecords[offsetIndex].journey[journeyIdx].speed;
                            dateTime[journeyIdx, 0] = trainRecords[offsetIndex].journey[journeyIdx].dateTime;

                        }

                        /* Reduce memory needs
                         * Reduce the speed and dataTiem decleration.
                         * Write each trains journey details individually here.
                         */
                        worksheet.Range[worksheet.Cells[headerOffset, displayColumn], worksheet.Cells[displayRow, displayColumn]].Value2 = kilometerage;

                        worksheet.Range[worksheet.Cells[headerOffset, displayColumn + 1], worksheet.Cells[displayRow, displayColumn + 1]].Value2 = speed;
                        worksheet.Range[worksheet.Cells[headerOffset, displayColumn + 2], worksheet.Cells[displayRow, displayColumn + 2]].Value2 = dateTime;
                        displayColumn = displayColumn + columnOffset;
                        horizontalOffset = horizontalOffset + columnOffset;

                    }

                    /* Write the data to the active excel workseet. */
                    worksheet.Range[worksheet.Cells[2, 3], worksheet.Cells[2, excelPageSize * columnOffset + 2]].Value2 = TrainID;
                    worksheet.Range[worksheet.Cells[3, 3], worksheet.Cells[3, excelPageSize * columnOffset + 2]].Value2 = LocoID;
                    worksheet.Range[worksheet.Cells[4, 3], worksheet.Cells[4, excelPageSize * columnOffset + 2]].Value2 = trainDate;
                    worksheet.Range[worksheet.Cells[5, 3], worksheet.Cells[5, excelPageSize * columnOffset + 2]].Value2 = powerToWeight;
                    worksheet.Range[worksheet.Cells[6, 3], worksheet.Cells[6, excelPageSize * columnOffset + 2]].Value2 = commodity;
                    worksheet.Range[worksheet.Cells[7, 3], worksheet.Cells[7, excelPageSize * columnOffset + 2]].Value2 = direction;
                }
            }

            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\TransactionTime_InterpolatedTrains" + DateTime.Now.ToString("yyyyMMdd - raw") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }


            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing, false, false,
                XlSaveAsAccessMode.xlNoChange, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;
        }

        /// <summary>
        /// This function writes each interpolated train journey to an individual column in excel.
        /// This can be used to compare against previously completed corridor analysis for validation.
        /// </summary>
        /// <param name="trainRecords">The train records containing the individual train properties.</param>
        /// <param name="startKm">The start point of the interpolation</param>
        /// <param name="interpoaltionInterval">The interpolation interval in m</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void writeTrainDataWithTime(List<Train> trainRecords, double startKm, double interpoaltionInterval, string aggregatedDestination)
        {

            /* Create the microsfot excel references. */
            _Workbook workbook;
            _Worksheet worksheet;

            /* Start Excel and get Application object. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();

            /* Get the reference to the new workbook. */
            workbook = (Microsoft.Office.Interop.Excel._Workbook)(excel.Workbooks.Add(""));

            /* Create the header details. */
            string[,] headerString = {{ "km", "", "Trains:" },
                                     { "", "Train ID:", "" },
                                     { "", "Loco ID:", "" },
                                     { "", "Date:", "" },
                                     { "", "Power to Weight Ratio:", "" },
                                     { "", "Commodity:", "" },
                                     { "", "Direction:", "" }};


            /* Pagenate the data for writing to excel. */
            int excelPageSize = 16380;          /* Max Columns in the excel worksheet. */
            int excelPages = 1;                 /* Number of Excel pages to write. */

            int headerOffset = 9;
            int horizontalOffset = 3;

            int headerRows = headerString.GetLength(0);
            int headerColumns = headerString.GetLength(1);

            int displayRow = headerOffset + trainRecords[0].journey.Count() - 1;
            int displayColumn = horizontalOffset;

            int timeOffset = 5;
            int timedataOffset = displayRow + timeOffset;

            /* Adjust the excel page size or the number of pages to write. */
            if (trainRecords.Count() < excelPageSize)
                excelPageSize = trainRecords.Count();
            else
                excelPages = (int)Math.Round((double)trainRecords.Count() / excelPageSize + 0.5);

            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Deconstruct the train details into excel columns. */
                string[,] TrainID = new string[1, excelPageSize];
                string[,] LocoID = new string[1, excelPageSize];
                double[,] powerToWeight = new double[1, excelPageSize];
                string[,] commodity = new string[1, excelPageSize];
                string[,] direction = new string[1, excelPageSize];
                DateTime[,] trainDate = new DateTime[1, excelPageSize];


                double[,] kilometerage = new double[trainRecords[0].journey.Count(), 1];

                double[,] speed = new double[trainRecords[0].journey.Count(), 1];
                DateTime[,] dateTime = new DateTime[trainRecords[0].journey.Count(), 1];

                horizontalOffset = 3;
                displayColumn = horizontalOffset;

                /* Set the active worksheet. */
                worksheet = workbook.Sheets[excelPage + 1];
                workbook.Sheets[excelPage + 1].Activate();
                Range topLeft = worksheet.Cells[1, 1];
                Range bottomRight = worksheet.Cells[headerRows, headerColumns];
                worksheet.get_Range(topLeft, bottomRight).Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int trainIdx = 0; trainIdx < excelPageSize; trainIdx++)
                {
                    /* Calucalte the appropriate index for the train. */
                    int offsetIndex = excelPage * excelPageSize + trainIdx;

                    if (offsetIndex < trainRecords.Count())
                    {
                        /* Populate the train properties. */
                        TrainID[0, trainIdx] = trainRecords[offsetIndex].trainID;
                        LocoID[0, trainIdx] = trainRecords[offsetIndex].locoID;
                        trainDate[0, trainIdx] = trainRecords[offsetIndex].journey.Where(t => t.dateTime > DateTime.MinValue).ToList().Min(t => t.dateTime);

                        powerToWeight[0, trainIdx] = trainRecords[offsetIndex].powerToWeight;

                        commodity[0, trainIdx] = trainRecords[offsetIndex].commodity.ToString();

                        direction[0, trainIdx] = trainRecords[offsetIndex].trainDirection.ToString();

                        /* Populate the train parameters. */
                        for (int journeyIdx = 0; journeyIdx < trainRecords[offsetIndex].journey.Count(); journeyIdx++)
                        {
                            kilometerage[journeyIdx, 0] = startKm + interpoaltionInterval * Processing.metresToKilometers * journeyIdx;

                            speed[journeyIdx, 0] = trainRecords[offsetIndex].journey[journeyIdx].speed;
                            dateTime[journeyIdx, 0] = trainRecords[offsetIndex].journey[journeyIdx].dateTime;

                        }

                        /* To reduce memory usage, write each trains journey details individually here. */
                        worksheet.Range[worksheet.Cells[headerOffset, horizontalOffset], worksheet.Cells[displayRow, displayColumn]].Value2 = speed;
                        worksheet.Range[worksheet.Cells[timedataOffset, horizontalOffset],
                            worksheet.Cells[timedataOffset + trainRecords[0].journey.Count() - 1, displayColumn]].Value2 = dateTime;

                        /* Adjust the offsets for each train. */
                        displayColumn++;
                        horizontalOffset++;

                    }

                    /* Write the data to the active excel workseet. */
                    worksheet.Range[worksheet.Cells[2, 3], worksheet.Cells[2, excelPageSize + 2]].Value2 = TrainID;
                    worksheet.Range[worksheet.Cells[3, 3], worksheet.Cells[3, excelPageSize + 2]].Value2 = LocoID;
                    worksheet.Range[worksheet.Cells[4, 3], worksheet.Cells[4, excelPageSize + 2]].Value2 = trainDate;
                    worksheet.Range[worksheet.Cells[5, 3], worksheet.Cells[5, excelPageSize + 2]].Value2 = powerToWeight;
                    worksheet.Range[worksheet.Cells[6, 3], worksheet.Cells[6, excelPageSize + 2]].Value2 = commodity;
                    worksheet.Range[worksheet.Cells[7, 3], worksheet.Cells[7, excelPageSize + 2]].Value2 = direction;

                    worksheet.Range[worksheet.Cells[headerOffset, 1], worksheet.Cells[displayRow, 1]].Value2 = kilometerage;
                    //worksheet.Range[worksheet.Cells[headerOffset, horizontalOffset], worksheet.Cells[displayRow, displayColumn]].Value2 = speed;

                    worksheet.Range[worksheet.Cells[timedataOffset, 1], worksheet.Cells[timedataOffset + trainRecords[0].journey.Count() - 1, 1]].Value2 = kilometerage;
                    //worksheet.Range[worksheet.Cells[timedataOffset, horizontalOffset], worksheet.Cells[timedataOffset + trainRecords[0].journey.Count() - 1, displayColumn]].Value2 = dateTime;

                }
            }

            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\TransactionTime_InterpolatedTrains" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }


            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing, false, false,
                XlSaveAsAccessMode.xlNoChange, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;
        }

        /// <summary>
        /// Write the aggregated data to a file for evaluation.
        /// </summary>
        /// <param name="averageTrains">List of aggregated train journies.</param>
        /// <param name="stats">The statstics generated for each average train</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void writeAverageData(List<AverageTrain> averageTrains, List<TrainStatistics> stats, string aggregatedDestination)
        {
            /* Start Excel and get the references to the workbook and worksheet. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            _Workbook workbook = excel.Workbooks.Add("");
            _Worksheet worksheet;

            /* Extract the statistics */
            /* Note: there is no check to confimr the order in which the statistics values are listed. */
            string[,] statisticsHeader = { { "Statistics:" },
                                         { "Number Of Trains" },
                                         { "Average Distance Travelled" },
                                         { "Average Speed" },
                                         { "Average P/W Ratio" },
                                         { "P/W standard Deviation" } };
            string[,] totalStatistics = new string[statisticsHeader.GetLength(0), 0];

            if (stats.Count() > 0)
            {
                totalStatistics = new string[statisticsHeader.GetLength(0), stats.Count()];

                /* Extract the statistics for each analysis Category */
                for (int index = 0; index < stats.Count(); index++)
                {
                    totalStatistics[0, index] = stats[index].Category;
                    totalStatistics[1, index] = stats[index].numberOfTrains.ToString();
                    totalStatistics[2, index] = stats[index].averageDistanceTravelled.ToString();
                    totalStatistics[3, index] = stats[index].averageSpeed.ToString();
                    totalStatistics[4, index] = stats[index].averagePowerToWeightRatio.ToString();
                    totalStatistics[5, index] = stats[index].standardDeviationP2W.ToString();

                }
            }

            /* Create the headers for the data. */
            List<string> directionHeader = new List<string>();      /* Direction of each train. */
            List<string> headerString = new List<string>();         /* Category of each train. */
            directionHeader.Add("");
            headerString.Add("kilometerage");
            directionHeader.Add("");
            headerString.Add("Elevation");

            for (int trainIdx = 0; trainIdx < averageTrains.Count(); trainIdx++)
            {
                directionHeader.Add(averageTrains[trainIdx].direction.ToString());
                headerString.Add(averageTrains[trainIdx].trainCategory.ToString());
            }

            directionHeader.Add("");
            directionHeader.Add("");
            headerString.Add("Loops");
            headerString.Add("TSRs");

            /* Add headers for standard devaitons */
            directionHeader.AddRange(new List<string>(new string[] { "", "" }));
            headerString.AddRange(new List<string>(new string[] { "", "" }));

            for (int trainIdx = 0; trainIdx < averageTrains.Count(); trainIdx++)
            {
                directionHeader.Add(averageTrains[trainIdx].direction.ToString());
                headerString.Add(averageTrains[trainIdx].trainCategory.ToString() + " StDev");
            }

            /* Add headers for sample counts */
            directionHeader.AddRange(new List<string>(new string[] { "", "" }));
            headerString.AddRange(new List<string>(new string[] { "", "" }));

            for (int trainIdx = 0; trainIdx < averageTrains.Count(); trainIdx++)
            {
                directionHeader.Add(averageTrains[trainIdx].direction.ToString());
                headerString.Add(averageTrains[trainIdx].trainCategory.ToString() + " sample");
            }

            /* Pagenate the data for writing to excel. */
            int numberOfPoints = averageTrains[0].kilometreage.Count();
            int headerOffset = statisticsHeader.GetLength(0) + 3;

            /* Deconstruct the train details into excel columns. */
            double[,] kilometerage = new double[numberOfPoints, 1];
            double[,] elevation = new double[numberOfPoints, 1];
            string[,] isLoophere = new string[numberOfPoints, 1];
            string[,] isTSRhere = new string[numberOfPoints, 1];

            double[,] averageSpeedArray = new double[numberOfPoints, averageTrains.Count()];
            int[,] sampleCountArray = new int[numberOfPoints, averageTrains.Count()];
            double[,] sampleStandarDeviation = new double[numberOfPoints, averageTrains.Count()];


            /* Set the active worksheet. */
            worksheet = (_Worksheet)workbook.Sheets[1];
            workbook.Sheets[1].Activate();

            /* Loop through the data for each excel page. */
            for (int i = 0; i < numberOfPoints; i++)
            {
                /* Populate the kilometerage and evlevation. */
                kilometerage[i, 0] = averageTrains[0].kilometreage[i];
                elevation[i, 0] = averageTrains[0].elevation[i];

                /* Identify where the loops or signal and TSR are. */
                if (averageTrains[0].isInLoopBoundary[i])
                    isLoophere[i, 0] = "Loop Boundary";

                if (averageTrains[0].isInTSRboundary[i])
                    isTSRhere[i, 0] = "TSR Boundary";

                /* Extract the average speed for each analysis Category */
                for (int j = 0; j < averageTrains.Count(); j++)
                {
                    averageSpeedArray[i, j] = averageTrains[j].averageSpeed[i];
                    sampleStandarDeviation[i, j] = averageTrains[j].speedStDev[i];
                    sampleCountArray[i, j] = averageTrains[j].sampleCount[i];
                }

            }

            /* Display the statistics for each Category. */
            int column = 3;
            Range topLeft = worksheet.Cells[statisticsHeader.GetLength(1), column];
            Range bottomRight = worksheet.Cells[statisticsHeader.GetLength(0), column + totalStatistics.GetLength(1) - 1];

            /* Set statistics. */
            worksheet.get_Range("A1", "A6").Value2 = statisticsHeader;
            worksheet.get_Range(topLeft, bottomRight).Value2 = totalStatistics;

            /* Set the data header. */
            topLeft = worksheet.Cells[headerOffset, 1];
            bottomRight = worksheet.Cells[headerOffset, directionHeader.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = directionHeader.ToArray();

            topLeft = worksheet.Cells[headerOffset + 1, 1];
            bottomRight = worksheet.Cells[headerOffset + 1, headerString.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = headerString.ToArray();

            int dataOffset = headerOffset + 2;
            /* Write the data to the active excel workseet. */
            worksheet.get_Range("A" + dataOffset, "A" + (dataOffset + numberOfPoints - 1)).Value2 = kilometerage;
            worksheet.get_Range("B" + dataOffset, "B" + (dataOffset + numberOfPoints - 1)).Value2 = elevation;

            /* Identify range of cells for average speed. */
            topLeft = worksheet.Cells[dataOffset, column];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count() - 1];
            worksheet.get_Range(topLeft, bottomRight).Value2 = averageSpeedArray;

            /* Identify range of cells for impact by loops. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count()];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = isLoophere;

            /* Identify range of cells for impact by TSR's. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count() + 1];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count() + 1];
            worksheet.get_Range(topLeft, bottomRight).Value2 = isTSRhere;

            /* Identify range of cells for standard deviation. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count() + 4];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count() * 2 + 3];
            worksheet.get_Range(topLeft, bottomRight).Value2 = sampleStandarDeviation;

            /* Identify range of cells for sample size. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count() * 2 + 6];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, headerString.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = sampleCountArray;
                        
            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\AverageSpeed_" + DateTime.Now.ToString("yyyyMMdd_HHmm") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, Microsoft.Office.Interop.Excel.XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing,
                false, false, Microsoft.Office.Interop.Excel.XlSaveAsAccessMode.xlNoChange,
                Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;

        }

        /// <summary>
        /// Write the aggregated data to a file for evaluation.
        /// </summary>
        /// <param name="averageTrains">List of aggregated train journies.</param>
        /// <param name="stats">The statstics generated for each average train</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        /// <param name="settings">Dictionary object contianing the settings used on the application form.</param>
        public static void writeAverageData(List<AverageTrain> averageTrains, List<TrainStatistics> stats, string aggregatedDestination, Dictionary<FieldInfo, object> settings)
        {
            /* Start Excel and get the references to the workbook and worksheet. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            _Workbook workbook = excel.Workbooks.Add("");
            _Worksheet worksheet;

            


            /* Extract the statistics */
            /* Note: there is no check to confimr the order in which the statistics values are listed. */
            string[,] statisticsHeader = { { "Statistics:" },
                                         { "Number Of Trains" },
                                         { "Average Distance Travelled" },
                                         { "Average Speed" },
                                         { "Average P/W Ratio" },
                                         { "P/W standard Deviation" } };
            string[,] totalStatistics = new string[statisticsHeader.GetLength(0), 0];

            if (stats.Count() > 0)
            {
                totalStatistics = new string[statisticsHeader.GetLength(0), stats.Count()];

                /* Extract the statistics for each analysis Category */
                for (int index = 0; index < stats.Count(); index++)
                {
                    totalStatistics[0, index] = stats[index].Category;
                    totalStatistics[1, index] = stats[index].numberOfTrains.ToString();
                    totalStatistics[2, index] = stats[index].averageDistanceTravelled.ToString();
                    totalStatistics[3, index] = stats[index].averageSpeed.ToString();
                    totalStatistics[4, index] = stats[index].averagePowerToWeightRatio.ToString();
                    totalStatistics[5, index] = stats[index].standardDeviationP2W.ToString();

                }
            }

            /* Create the headers for the data. */
            List<string> directionHeader = new List<string>();      /* Direction of each train. */
            List<string> headerString = new List<string>();         /* Category of each train. */
            directionHeader.Add("");
            headerString.Add("kilometerage");
            directionHeader.Add("");
            headerString.Add("Elevation");

            for (int trainIdx = 0; trainIdx < averageTrains.Count(); trainIdx++)
            {
                directionHeader.Add(averageTrains[trainIdx].direction.ToString());
                headerString.Add(averageTrains[trainIdx].trainCategory.ToString());
            }

            directionHeader.Add("");
            directionHeader.Add("");
            headerString.Add("Loops");
            headerString.Add("TSRs");

            /* Add headers for standard devaitons */
            directionHeader.AddRange(new List<string>(new string[] { "", "" }));
            headerString.AddRange(new List<string>(new string[] { "", "" }));

            for (int trainIdx = 0; trainIdx < averageTrains.Count(); trainIdx++)
            {
                directionHeader.Add(averageTrains[trainIdx].direction.ToString());
                headerString.Add(averageTrains[trainIdx].trainCategory.ToString() + " StDev");
            }

            /* Add headers for sample counts */
            directionHeader.AddRange(new List<string>(new string[] { "", "" }));
            headerString.AddRange(new List<string>(new string[] { "", "" }));

            for (int trainIdx = 0; trainIdx < averageTrains.Count(); trainIdx++)
            {
                directionHeader.Add(averageTrains[trainIdx].direction.ToString());
                headerString.Add(averageTrains[trainIdx].trainCategory.ToString() + " sample");
            }

            /* Pagenate the data for writing to excel. */
            int numberOfPoints = averageTrains[0].kilometreage.Count();
            int headerOffset = statisticsHeader.GetLength(0) + 3;

            /* Deconstruct the train details into excel columns. */
            double[,] kilometerage = new double[numberOfPoints, 1];
            double[,] elevation = new double[numberOfPoints, 1];
            string[,] isLoophere = new string[numberOfPoints, 1];
            string[,] isTSRhere = new string[numberOfPoints, 1];

            double[,] averageSpeedArray = new double[numberOfPoints, averageTrains.Count()];
            int[,] sampleCountArray = new int[numberOfPoints, averageTrains.Count()];
            double[,] sampleStandarDeviation = new double[numberOfPoints, averageTrains.Count()];


            /* Set the active worksheet. */
            worksheet = (_Worksheet)workbook.Sheets[1];
            workbook.Sheets[1].Activate();

            /* Loop through the data for each excel page. */
            for (int i = 0; i < numberOfPoints; i++)
            {
                /* Populate the kilometerage and evlevation. */
                kilometerage[i, 0] = averageTrains[0].kilometreage[i];
                elevation[i, 0] = averageTrains[0].elevation[i];

                /* Identify where the loops or signal and TSR are. */
                if (averageTrains[0].isInLoopBoundary[i])
                    isLoophere[i, 0] = "Loop Boundary";

                if (averageTrains[0].isInTSRboundary[i])
                    isTSRhere[i, 0] = "TSR Boundary";

                /* Extract the average speed for each analysis Category */
                for (int j = 0; j < averageTrains.Count(); j++)
                {
                    averageSpeedArray[i, j] = averageTrains[j].averageSpeed[i];
                    sampleStandarDeviation[i, j] = averageTrains[j].speedStDev[i];
                    sampleCountArray[i, j] = averageTrains[j].sampleCount[i];
                }

            }

            /* Display the statistics for each Category. */
            int column = 3;
            Range topLeft = worksheet.Cells[statisticsHeader.GetLength(1), column];
            Range bottomRight = worksheet.Cells[statisticsHeader.GetLength(0), column + totalStatistics.GetLength(1) - 1];

            /* Set statistics. */
            worksheet.get_Range("A1", "A6").Value2 = statisticsHeader;
            worksheet.get_Range(topLeft, bottomRight).Value2 = totalStatistics;

            /* Set the data header. */
            topLeft = worksheet.Cells[headerOffset, 1];
            bottomRight = worksheet.Cells[headerOffset, directionHeader.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = directionHeader.ToArray();

            topLeft = worksheet.Cells[headerOffset + 1, 1];
            bottomRight = worksheet.Cells[headerOffset + 1, headerString.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = headerString.ToArray();

            int dataOffset = headerOffset + 2;
            /* Write the data to the active excel workseet. */
            worksheet.get_Range("A" + dataOffset, "A" + (dataOffset + numberOfPoints - 1)).Value2 = kilometerage;
            worksheet.get_Range("B" + dataOffset, "B" + (dataOffset + numberOfPoints - 1)).Value2 = elevation;

            /* Identify range of cells for average speed. */
            topLeft = worksheet.Cells[dataOffset, column];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count() - 1];
            worksheet.get_Range(topLeft, bottomRight).Value2 = averageSpeedArray;

            /* Identify range of cells for impact by loops. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count()];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = isLoophere;

            /* Identify range of cells for impact by TSR's. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count() + 1];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count() + 1];
            worksheet.get_Range(topLeft, bottomRight).Value2 = isTSRhere;

            /* Identify range of cells for standard deviation. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count() + 4];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count() * 2 + 3];
            worksheet.get_Range(topLeft, bottomRight).Value2 = sampleStandarDeviation;

            /* Identify range of cells for sample size. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count() * 2 + 6];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, headerString.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = sampleCountArray;

            /* Create a new sheet for the settings to be stored. */
            Worksheet worksheet2 = workbook.Sheets.Add(Type.Missing, Type.Missing, 1, Type.Missing) as Worksheet;
            /* Write the settings parameters to a new sheet. */
            writeSettings(worksheet2, settings);

            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\AverageSpeed_" + DateTime.Now.ToString("yyyyMMdd_HHmm") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, Microsoft.Office.Interop.Excel.XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing,
                false, false, Microsoft.Office.Interop.Excel.XlSaveAsAccessMode.xlNoChange,
                Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;

        }

        /// <summary>
        /// A preliminary function to write the data to a file in a format 
        /// that will be condusive to visualisation in Tableau.
        /// </summary>
        /// <param name="trainData">The train data in the required format</param>
        /// <param name="destinationFolder">The destination folder to save the file.</param>
        public static void writeProcessTrainDataPoints(List<processTrainDataPoint> trainData, string destinationFolder)
        {
           
            /* Maximum number of rows in an excel worksheet is 1,048,576 (round down to a nice number) */
            int maxExcelRows = 1048500;
            string loop = "";
            string tsr = "";
            string gap = "";

            /* Create the microsfot excel references. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            Workbook workbook;
            Worksheet worksheet;

            /* Get the reference to the new workbook. */
            workbook = (Workbook)(excel.Workbooks.Add(""));

            /* Create the header details. */
            string[] headerString = { "Train Date", "Train ID", "Loco ID", "Power to Weight", "Operator", "Commodity",
                                      "Direction", "Kilometreage", "Speed", "Time", "Latitude", "Longitude",
                                      "Elevation", "isLoop", "isTSR", "isGap", "Simulation Speed", "Simualtion Time",
                                      "Average Speed", "Average Time"};

            /* Get the page size of the excel worksheet. */
            int header = 2;
            int excelPageSize = trainData.Count();
            int excelPages = 1;

            /* Set the number of pages required. */
            if (trainData.Count() > maxExcelRows)
            {
                excelPageSize = 1000000;
                excelPages = (int)Math.Round((double)trainData.Count() / excelPageSize + 0.5);
            }

            /* Deconstruct the volume details into excel columns. */
            string[,] trainID = new string[excelPageSize, 1];
            string[,] LocoID = new string[excelPageSize, 1];
            double[,] power = new double[excelPageSize, 1];
            DateTime[,] trainDate = new DateTime[excelPageSize, 1];
            string[,] trainOperator = new string[excelPageSize, 1];
            string[,] commodity = new string[excelPageSize, 1];
            string[,] direction = new string[excelPageSize, 1];
            double[,] kilometreage = new double[excelPageSize, 1];
            double[,] Speed = new double[excelPageSize, 1];
            double[,] Time = new double[excelPageSize, 1];
            string[,] isLoop = new string[excelPageSize, 1];
            string[,] isTSR = new string[excelPageSize, 1];
            double[,] Latitude = new double[excelPageSize, 1];
            double[,] Longitude = new double[excelPageSize, 1];
            double[,] Elevation = new double[excelPageSize, 1];
            string[,] isGap = new string[excelPageSize, 1];
            double[,] simualtionSpeed = new double[excelPageSize, 1];
            double[,] simulationTime = new double[excelPageSize, 1];
            double[,] averageSpeed = new double[excelPageSize, 1];
            double[,] averageTime = new double[excelPageSize, 1];

            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Set the active worksheet. */
                worksheet = workbook.Sheets.Add(Type.Missing, Type.Missing, 1, Type.Missing) as Worksheet;
                worksheet.Name = "Page " + (excelPage + 1);

                worksheet.get_Range("A1", "T1").Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int j = 0; j < excelPageSize; j++)
                {
                    loop = "";
                    tsr = "";
                    gap = "";

                    /* Check we dont try to read more data than there really is. */
                    int checkIdx = j + excelPage * excelPageSize;
                    if (checkIdx < trainData.Count())
                    {

                        trainID[j, 0] = trainData[checkIdx].TrainID;
                        LocoID[j, 0] = trainData[checkIdx].locoID;
                        power[j, 0] = trainData[checkIdx].PW_ratio;
                        trainDate[j, 0] = trainData[checkIdx].trainDate;
                        trainOperator[j, 0] = trainData[checkIdx].trainOperator.ToString();
                        commodity[j, 0] = trainData[checkIdx].commodity.ToString();
                        direction[j, 0] = trainData[checkIdx].trainDirection.ToString();
                        kilometreage[j, 0] = trainData[checkIdx].kmMarker;
                        Speed[j, 0] = trainData[checkIdx].speed;
                        Time[j, 0] = trainData[checkIdx].transitTime;
                        Latitude[j, 0] = trainData[checkIdx].location.latitude;
                        Longitude[j, 0] = trainData[checkIdx].location.longitude;
                        Elevation[j, 0] = trainData[checkIdx].alignmentElevation;
                        simualtionSpeed[j, 0] = trainData[checkIdx].simulationSpeed;
                        simulationTime[j, 0] = trainData[checkIdx].simulationTime;
                        averageSpeed[j, 0] = trainData[checkIdx].averageSpeed;
                        averageTime[j, 0] = trainData[checkIdx].averageTime;

                        if (trainData[checkIdx].isLoop)
                            loop = "Loop";

                        if (trainData[checkIdx].isTSR)
                            tsr = "TSR";

                        if (!trainData[checkIdx].isLargeGap)
                            gap = "Gap";

                        isLoop[j, 0] = loop;
                        isTSR[j, 0] = tsr;
                        isGap[j, 0] = gap;

                    }
                    else
                    {
                        /* The end of the data has been reached. Populate the remaining elements. */
                        trainID[j, 0] = "";
                        LocoID[j, 0] = "";
                        power[j, 0] = 0;
                        trainDate[j, 0] = DateTime.MinValue;
                        trainOperator[j, 0] = "";
                        commodity[j, 0] = "";
                        direction[j, 0] = "";
                        kilometreage[j, 0] = 0;
                        Speed[j, 0] = 0;
                        Time[j, 0] = 0;
                        Latitude[j, 0] = 0;
                        Longitude[j, 0] = 0;
                        Elevation[j, 0] = 0;
                        simualtionSpeed[j, 0] = 0;
                        simulationTime[j, 0] = 0;
                        averageSpeed[j, 0] = 0;
                        averageTime[j, 0] = 0;

                        isLoop[j, 0] = "";
                        isTSR[j, 0] = "";
                        isGap[j, 0] = "";
                    }
                }

                /* Write the data to the active excel workseet. */
                worksheet.get_Range("A" + header, "A" + (header + excelPageSize - 1)).Value2 = trainDate;
                worksheet.get_Range("B" + header, "B" + (header + excelPageSize - 1)).Value2 = trainID;
                worksheet.get_Range("C" + header, "C" + (header + excelPageSize - 1)).Value2 = LocoID;
                worksheet.get_Range("D" + header, "D" + (header + excelPageSize - 1)).Value2 = power;
                worksheet.get_Range("E" + header, "E" + (header + excelPageSize - 1)).Value2 = trainOperator;
                worksheet.get_Range("F" + header, "F" + (header + excelPageSize - 1)).Value2 = commodity;
                worksheet.get_Range("G" + header, "G" + (header + excelPageSize - 1)).Value2 = direction;
                worksheet.get_Range("H" + header, "H" + (header + excelPageSize - 1)).Value2 = kilometreage;
                worksheet.get_Range("I" + header, "I" + (header + excelPageSize - 1)).Value2 = Speed;
                worksheet.get_Range("J" + header, "J" + (header + excelPageSize - 1)).Value2 = Time;
                worksheet.get_Range("K" + header, "K" + (header + excelPageSize - 1)).Value2 = Latitude;
                worksheet.get_Range("L" + header, "L" + (header + excelPageSize - 1)).Value2 = Longitude;
                worksheet.get_Range("M" + header, "M" + (header + excelPageSize - 1)).Value2 = Elevation;
                worksheet.get_Range("N" + header, "N" + (header + excelPageSize - 1)).Value2 = isLoop;
                worksheet.get_Range("O" + header, "O" + (header + excelPageSize - 1)).Value2 = isTSR;
                worksheet.get_Range("P" + header, "P" + (header + excelPageSize - 1)).Value2 = isGap;
                worksheet.get_Range("Q" + header, "Q" + (header + excelPageSize - 1)).Value2 = simualtionSpeed;
                worksheet.get_Range("R" + header, "R" + (header + excelPageSize - 1)).Value2 = simulationTime;
                worksheet.get_Range("S" + header, "S" + (header + excelPageSize - 1)).Value2 = averageSpeed;
                worksheet.get_Range("T" + header, "T" + (header + excelPageSize - 1)).Value2 = averageTime;

            }   // end of excel page loop



            string saveFilename = destinationFolder + @"\processTrainData_" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
                File.Delete(saveFilename);

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing,
                false, false, XlSaveAsAccessMode.xlNoChange,
                Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;



        }

        /// <summary>
        /// Write algorithm parameters used to a new sheet in the excel workbook.
        /// </summary>
        /// <param name="sheet">The new sheet to write the parameters to.</param>
        /// <param name="settings">A dictionary of the parameter name and values.</param>
        public static void writeSettings(_Worksheet sheet, Dictionary<FieldInfo, object> settings)
        {
            /* Activate the current sheet. */
            sheet.Activate();
            
            /* Column reference */
            string namecol, valuecol;
            int counter = 1;

            foreach (var param in settings)
            {
                /* Dynamicaly create the cell references */
                namecol = "A" + counter.ToString();
                valuecol = "B" + counter.ToString();

                /* If the field is date time, process the start and end date values */
                if (param.Key.FieldType.Name.Equals("DateTime[]"))
                {
                    DateTime[] dates = (DateTime[])param.Value;
                    
                    /* Process start date. */
                    sheet.get_Range(namecol, namecol).Value2 = "Start Date:";
                    sheet.get_Range(valuecol, valuecol).Value2 = dates[0].ToShortDateString();

                    /* Increment the cell reference */
                    counter++;
                    namecol = "A" + counter.ToString();
                    valuecol = "B" + counter.ToString();

                    /* Process the end date. */
                    sheet.get_Range(namecol, namecol).Value2 = "End Date:";
                    sheet.get_Range(valuecol, valuecol).Value2 = dates[1].ToShortDateString();




                }
                /* If the object is a list, we assume this is the list of simulation files. */
                else if (param.Key.FieldType.Name.Equals("List`1"))
                {

                    List<string> files = (List<string>)param.Value;
                    foreach (string file in files)
                    {
                        /* Update the cell reference */
                        namecol = "A" + counter.ToString();
                        valuecol = "B" + counter.ToString();

                        /* Identify the file name. */
                        sheet.get_Range(namecol, namecol).Value2 = "Simulation file:";
                        if (param.Value != null)
                            sheet.get_Range(valuecol, valuecol).Value2 = file;
                        else
                            sheet.get_Range(valuecol, valuecol).Value2 = "";

                        /* Increment the counter for the cell reference. */
                        counter++;
                        
                    }
                }
                /* Process the remaining parameters. */
                else
                {                   
                    /* Update the cell reference. */
                    namecol = "A" + counter.ToString();
                    valuecol = "B" + counter.ToString();

                    /* Process the parameter. */
                    sheet.get_Range(namecol, namecol).Value2 = param.Key.Name;
                    if (param.Value != null)
                        sheet.get_Range(valuecol, valuecol).Value2 = param.Value.ToString();
                    else
                        sheet.get_Range(valuecol, valuecol).Value2 = "";

                }
                counter++;
            }
                       
        }


        /// <summary>
        /// Write the train pairs data to file. The train speed and timing at each point will be 
        /// displayed for future analysis and investigation.
        /// </summary>
        /// <param name="pair">A list of train pairs.</param>
        /// <param name="loopLocations">A list of loop or signal locations.</param>
        /// <param name="aggregatedDestination">The destination directory.</param>
        public static void writeTrainPairs(List<TrainPair> pair, List<LoopLocation> loopLocations, string aggregatedDestination)
        {

            /* Create the microsoft excel references. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            excel.Visible = false;
            Workbook workbook = excel.Workbooks.Add(Type.Missing);
            Worksheet worksheet;

            /* Create the header details. */
            string[,] headerString = {{ "km", "", "Trains:" },
                                     { "", "Train ID:", "" },
                                     { "", "Loco ID:", "" },
                                     { "", "Train Operator:", "" },
                                     { "", "Date:", "" },
                                     {"",  "Commodity:", ""},
                                     { "", "Direction:", "" },
                                     { "", "", "" },
                                     { "","Time of stopped train to converge with through train speed", "" },
                                     { "","Time of average actual through train to reach converged speed location", "" },
                                     { "","Time between rear of through train clearing, to stopped train starting", "" },
                                     { "","Total transaction time", "" }};
            /* Set display offset parameters. */
            int headerRows = headerString.GetLength(0);
            int headerColumns = headerString.GetLength(1);

            int headerOffset = headerRows + 2;
            int horizontalOffset = 3;

            int displayRow = headerOffset + pair[0].stoppedTrain.journey.Count() - 1;

            int timeOffset = 5;
            int timedataOffset = displayRow + timeOffset;

            /* Index offset for the train pairs. */
            int pairCount = 0;

            /* Loop through each loop location, displaying all train pairs associated with each loop on seperate worksheets. */
            for (int tabIdx = 0; tabIdx < loopLocations.Count(); tabIdx++)
            {
                /* Determine the number of train pairs stopping at each loop. */
                int numberOfPairsAtLoop = pair.Where(p => p.loopLocation == loopLocations[tabIdx]).Count();

                /* If a train stops at the loop. dispaly it. */
                if (numberOfPairsAtLoop > 0)
                {
                    int displayColumn = horizontalOffset + numberOfPairsAtLoop * 2 - 1;

                    /* Set the worksheet, and rename it according to the loop location. */
                    worksheet = workbook.Sheets.Add(Type.Missing, Type.Missing, 1, Type.Missing) as Worksheet;
                    worksheet.Name = string.Format("Loop {0:0.00} km - {1:0.00} km", loopLocations[tabIdx].loopStart, loopLocations[tabIdx].loopEnd);
                    /* Display the header on each worksheet. */
                    Range topLeft = worksheet.Cells[1, 1];
                    Range bottomRight = worksheet.Cells[headerRows, headerColumns];
                    worksheet.get_Range(topLeft, bottomRight).Value2 = headerString;

                    /* Deconstruct the train details into excel columns. */
                    string[,] TrainID = new string[1, numberOfPairsAtLoop * 2];
                    string[,] LocoID = new string[1, numberOfPairsAtLoop * 2];
                    string[,] trainOperator = new string[1, numberOfPairsAtLoop * 2];
                    double[,] powerToWeight = new double[1, numberOfPairsAtLoop * 2];
                    string[,] commodity = new string[1, numberOfPairsAtLoop * 2];
                    string[,] direction = new string[1, numberOfPairsAtLoop * 2];
                    DateTime[,] trainDate = new DateTime[1, numberOfPairsAtLoop * 2];
                    /* The transit time paramters. */
                    string[,] timeForStoppedTrainToReachTrackSpeed = new string[1, numberOfPairsAtLoop * 2];
                    string[,] simulatedTrainToReachTrackSpeedLocation = new string[1, numberOfPairsAtLoop * 2];
                    string[,] timeBetweenClearingLoopAndRestart = new string[1, numberOfPairsAtLoop * 2];
                    string[,] transactionTime = new string[1, numberOfPairsAtLoop * 2];

                    double[,] kilometerage = new double[pair[0].stoppedTrain.journey.Count(), 1];
                    string[,] loop = new string[pair[0].stoppedTrain.journey.Count(), 1];

                    double[,] speed = new double[pair[0].stoppedTrain.journey.Count(), numberOfPairsAtLoop * 2];
                    DateTime[,] dateTime = new DateTime[pair[0].stoppedTrain.journey.Count(), numberOfPairsAtLoop * 2];

                    /* Loop through the train pairs at each loop. */
                    for (int pairIdx = 0; pairIdx < numberOfPairsAtLoop; pairIdx++)
                    {
                        /* Account for the offset of each displaying loop location on seperate worksheets. */
                        int stoppedIdx = pairIdx * 2;
                        int throughIdx = pairIdx * 2 + 1;
                        int pairListIdx = pairCount + pairIdx;

                        /* Populate the stoppping train parameters. */
                        TrainID[0, stoppedIdx] = pair[pairListIdx].stoppedTrain.trainID;
                        LocoID[0, stoppedIdx] = pair[pairListIdx].stoppedTrain.locoID;
                        trainOperator[0, stoppedIdx] = pair[pairListIdx].stoppedTrain.trainOperator.ToString();
                        trainDate[0, stoppedIdx] = pair[pairListIdx].stoppedTrain.journey[0].dateTime;
                        powerToWeight[0, stoppedIdx] = pair[pairListIdx].stoppedTrain.powerToWeight;
                        commodity[0, stoppedIdx] = pair[pairListIdx].stoppedTrain.commodity.ToString();
                        direction[0, stoppedIdx] = pair[pairListIdx].stoppedTrain.trainDirection.ToString();

                        /* Populate the transaction time parameters. */
                        timeForStoppedTrainToReachTrackSpeed[0, stoppedIdx] = string.Format("{0:0.000}", pair[pairListIdx].timeForStoppedTrainToReachTrackSpeed);
                        simulatedTrainToReachTrackSpeedLocation[0, stoppedIdx] = string.Format("{0:0.000}", pair[pairListIdx].simulatedTrainToReachTrackSpeedLocation);
                        timeBetweenClearingLoopAndRestart[0, stoppedIdx] = string.Format("{0:0.000}", pair[pairListIdx].timeBetweenClearingLoopAndRestart);
                        transactionTime[0, stoppedIdx] = string.Format("{0:0.000}", pair[pairListIdx].transactionTime);

                        /* Populate the through train parameters. */
                        TrainID[0, throughIdx] = pair[pairListIdx].throughTrain.trainID;
                        LocoID[0, throughIdx] = pair[pairListIdx].throughTrain.locoID;
                        trainOperator[0, throughIdx] = pair[pairListIdx].throughTrain.trainOperator.ToString();
                        trainDate[0, throughIdx] = pair[pairListIdx].throughTrain.journey[0].dateTime;
                        powerToWeight[0, throughIdx] = pair[pairListIdx].throughTrain.powerToWeight;
                        commodity[0, throughIdx] = pair[pairListIdx].throughTrain.commodity.ToString();
                        direction[0, throughIdx] = pair[pairListIdx].throughTrain.trainDirection.ToString();

                        timeForStoppedTrainToReachTrackSpeed[0, throughIdx] = string.Format("{0:0.00}", pair[pairListIdx].distanceToTrackSpeed);
                        simulatedTrainToReachTrackSpeedLocation[0, throughIdx] = "";
                        timeBetweenClearingLoopAndRestart[0, throughIdx] = "";
                        transactionTime[0, throughIdx] = "";

                        /* Loop through the train journies. */
                        for (int journeyIdx = 0; journeyIdx < pair[0].stoppedTrain.journey.Count(); journeyIdx++)
                        {
                            /* Populate the kilometerage. */
                            kilometerage[journeyIdx, 0] = pair[0].stoppedTrain.journey[journeyIdx].kilometreage;
                            if (pair[0].stoppedTrain.journey[journeyIdx].isLoopHere)
                                loop[journeyIdx, 0] = "loop";
                            else
                                loop[journeyIdx, 0] = "";

                            /* Populate the speed and times for both the stopping train and the through train. */
                            speed[journeyIdx, stoppedIdx] = pair[pairListIdx].stoppedTrain.journey[journeyIdx].speed;
                            speed[journeyIdx, throughIdx] = pair[pairListIdx].throughTrain.journey[journeyIdx].speed;

                            dateTime[journeyIdx, stoppedIdx] = pair[pairListIdx].stoppedTrain.journey[journeyIdx].dateTime;
                            dateTime[journeyIdx, throughIdx] = pair[pairListIdx].throughTrain.journey[journeyIdx].dateTime;
                        }
                    }
                    /* Increment the train pair offset. */
                    pairCount += numberOfPairsAtLoop;

                    /* Write the data to the active excel worksheet. */
                    worksheet.Range[worksheet.Cells[2, 3], worksheet.Cells[2, numberOfPairsAtLoop * 2 + 2]].Value2 = TrainID;
                    worksheet.Range[worksheet.Cells[3, 3], worksheet.Cells[3, numberOfPairsAtLoop * 2 + 2]].Value2 = LocoID;
                    worksheet.Range[worksheet.Cells[4, 3], worksheet.Cells[4, numberOfPairsAtLoop * 2 + 2]].Value2 = trainOperator;
                    worksheet.Range[worksheet.Cells[5, 3], worksheet.Cells[5, numberOfPairsAtLoop * 2 + 2]].Value2 = trainDate;
                    worksheet.Range[worksheet.Cells[6, 3], worksheet.Cells[6, numberOfPairsAtLoop * 2 + 2]].Value2 = commodity;
                    worksheet.Range[worksheet.Cells[7, 3], worksheet.Cells[7, numberOfPairsAtLoop * 2 + 2]].Value2 = direction;
                    
                    worksheet.Range[worksheet.Cells[9, 3], worksheet.Cells[9, numberOfPairsAtLoop * 2 + 2]].Value2 = timeForStoppedTrainToReachTrackSpeed;
                    worksheet.Range[worksheet.Cells[10, 3], worksheet.Cells[10, numberOfPairsAtLoop * 2 + 2]].Value2 = simulatedTrainToReachTrackSpeedLocation;
                    worksheet.Range[worksheet.Cells[11, 3], worksheet.Cells[11, numberOfPairsAtLoop * 2 + 2]].Value2 = timeBetweenClearingLoopAndRestart;
                    worksheet.Range[worksheet.Cells[12, 3], worksheet.Cells[12, numberOfPairsAtLoop * 2 + 2]].Value2 = transactionTime;

                    /* Speed data */
                    worksheet.Range[worksheet.Cells[headerOffset, 1], worksheet.Cells[displayRow, 1]].Value2 = kilometerage;
                    worksheet.Range[worksheet.Cells[headerOffset, 2], worksheet.Cells[displayRow, 2]].Value2 = loop;
                    worksheet.Range[worksheet.Cells[headerOffset, horizontalOffset], worksheet.Cells[displayRow, displayColumn]].Value2 = speed;
                    /* Time data */
                    worksheet.Range[worksheet.Cells[timedataOffset, 1], worksheet.Cells[timedataOffset + pair[0].stoppedTrain.journey.Count() - 1, 1]].Value2 = kilometerage;
                    worksheet.Range[worksheet.Cells[timedataOffset, 2], worksheet.Cells[timedataOffset + pair[0].stoppedTrain.journey.Count() - 1, 2]].Value2 = loop;
                    worksheet.Range[worksheet.Cells[timedataOffset, horizontalOffset], worksheet.Cells[timedataOffset + pair[0].stoppedTrain.journey.Count() - 1, displayColumn]].Value2 = dateTime;
                }


            }

            string saveFilename = aggregatedDestination + @"\TransactionTime_TrainPairs" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";
            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }

            /* Save the workbook and close the excel application. */
            workbook.SaveAs(saveFilename, XlFileFormat.xlOpenXMLWorkbook, Type.Missing, Type.Missing, Type.Missing, Type.Missing,
                XlSaveAsAccessMode.xlNoChange, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close(Type.Missing, Type.Missing, Type.Missing);
            excel.UserControl = true;
            excel.Quit();

            return;
        }
        
        /// <summary>
        /// Write the aggregated data to a file for evaluation.
        /// </summary>
        /// <param name="averageTrains">List of aggregated train journies.</param>
        /// <param name="stats">The statstics generated for each average train</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void writeTrainPairStatistics(List<TrainPairStatistics> stats, string aggregatedDestination)
        {
            /* Create the microsoft excel references. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            excel.Visible = false;
            Workbook workbook = excel.Workbooks.Add(Type.Missing);
            Worksheet worksheet;

            /* Extract the statistics */
            /* Note: there is no check to confimr the order in which the statistics values are listed. */
            string[,] statisticsHeader = { { "Statistics:" }, 
                                         { "Number Of Trains Pairs" }, 
                                         { "Time To Reach Track Speed" }, 
                                         { "Time For Simulated Train To Reach Track Speed Location." },
                                         { "Time Between Through Train Clearing Loop And Stopped Train To Restart." }, 
                                         { "Transaction Time" },
                                         { "Standard Deviation in Transaction Time" } };

            int numberOfRows = statisticsHeader.GetLength(0);
            string[,] totalStatistics = new string[numberOfRows, 0];

            if (stats.Count() > 0)
            {
                totalStatistics = new string[numberOfRows, stats.Count()];

                /* Extract the statistics for each analysis Category */
                for (int index = 0; index < stats.Count(); index++)
                {
                    totalStatistics[0, index] = stats[index].Category;
                    totalStatistics[1, index] = stats[index].numberOfTrainsPairs.ToString();
                    totalStatistics[2, index] = stats[index].averageTimeForStoppedTrainToReachTrackSpeed.ToString();
                    totalStatistics[3, index] = stats[index].averageSimulatedTrainToReachTrackSpeedLocation.ToString();
                    totalStatistics[4, index] = stats[index].averageTimeBetweenClearingLoopAndRestart.ToString();
                    totalStatistics[5, index] = stats[index].averageTransactionTime.ToString();
                    totalStatistics[6, index] = stats[index].transactionTimeStandardDeviation.ToString();

                }
            }

            /* Display the header on each worksheet. */
            worksheet = workbook.Sheets[1];
            workbook.Sheets[1].Activate();

            Range topLeft = worksheet.Cells[1, 1];
            Range bottomRight = worksheet.Cells[numberOfRows, 1];
            worksheet.get_Range(topLeft, bottomRight).Value2 = statisticsHeader;

            /* Write the data to the active excel worksheet. */
            int dataOffset = 3;
            topLeft = worksheet.Cells[1, dataOffset];
            bottomRight = worksheet.Cells[numberOfRows, stats.Count() + dataOffset - 1];
            worksheet.get_Range(topLeft, bottomRight).Value2 = totalStatistics;


            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\TransactionTime_Statistics" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, Microsoft.Office.Interop.Excel.XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing,
                false, false, Microsoft.Office.Interop.Excel.XlSaveAsAccessMode.xlNoChange,
                Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;

        }

        /// <summary>
        /// Write the occupation blocks of each section to file.
        /// </summary>
        /// <param name="utilisation">List of occupation blocks for each section.</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void writeSectionUtilistion(List<List<OccupationBlock>> utilisation, string aggregatedDestination)
        {
            /* Create the microsoft excel references. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            excel.Visible = false;
            Workbook workbook = excel.Workbooks.Add(Type.Missing);
            Worksheet worksheet;

            /* Create the header details. */
            string[,] headerString = {{"Section:", "", "", "" },
                                      {"Time:", "Start",  "End", "Occupied" }};

            /* Pagenate the data for writing to excel. */
            int excelPageSize = 1000000;        /* Page size of the excel worksheet. */
            int excelPages = 1;                 /* Number of Excel pages to write. */

            int headerRows = headerString.GetLength(0);
            int headerColumns = headerString.GetLength(1);

            int columnOffset = 1;

            int maximumNumberOfOccupationBlocks = 0;

            /* Adjust the excel page size or the number of pages to write. */
            foreach (List<OccupationBlock> sections in utilisation) { if (sections.Count() > maximumNumberOfOccupationBlocks) maximumNumberOfOccupationBlocks = sections.Count(); }

            if (maximumNumberOfOccupationBlocks < excelPageSize)
                excelPageSize = maximumNumberOfOccupationBlocks;
            else
                excelPages = (int)Math.Round((double)maximumNumberOfOccupationBlocks / excelPageSize + 0.5);

            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Set the active worksheet. */
                worksheet = workbook.Sheets[excelPage + 1];
                workbook.Sheets[excelPage + 1].Activate();
                Range topLeft = worksheet.Cells[1, 1];
                Range bottomRight = worksheet.Cells[headerRows, headerColumns];
                worksheet.get_Range(topLeft, bottomRight).Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int sectionIndex = 0; sectionIndex < utilisation.Count(); sectionIndex++)
                {
                    /* Extract each section. */
                    List<OccupationBlock> section = utilisation[sectionIndex];

                    /* Define the arrays to write to the file. */
                    double[] sectionBoundaries = new double[] { section[0].section.sectionStart, section[0].section.sectionEnd };
                    DateTime[,] startTime = new DateTime[section.Count(), 1];
                    DateTime[,] endTime = new DateTime[section.Count(), 1];
                    double[,] timeOccupied = new double[section.Count(), 1];

                    /* Loop through each occupation block for the section. */
                    for (int blocKidx = 0; blocKidx < section.Count(); blocKidx++)
                    {
                        startTime[blocKidx, 0] = section[blocKidx].startTime;
                        endTime[blocKidx, 0] = section[blocKidx].endTime;
                        timeOccupied[blocKidx, 0] = section[blocKidx].minutesOccupied;
                    }

                    /* Adjust the desplay offset for each section. */
                    columnOffset = (sectionIndex * headerColumns) + 1;

                    /* Write the data to file. */
                    worksheet.Range[worksheet.Cells[1, columnOffset + 1], worksheet.Cells[1, columnOffset + 2]].Value2 = sectionBoundaries;
                    worksheet.Range[worksheet.Cells[headerRows + 1, columnOffset + 1], worksheet.Cells[headerRows + section.Count(), columnOffset + 1]].Value2 = startTime;
                    worksheet.Range[worksheet.Cells[headerRows + 1, columnOffset + 2], worksheet.Cells[headerRows + section.Count(), columnOffset + 2]].Value2 = endTime;
                    worksheet.Range[worksheet.Cells[headerRows + 1, columnOffset + 3], worksheet.Cells[headerRows + section.Count(), columnOffset + 3]].Value2 = timeOccupied;

                }

            }

            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\sectionUtilisation_" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing, false, false,
                XlSaveAsAccessMode.xlNoChange, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;
        }

        /// <summary>
        /// Write the aggregated utilisation data to file.
        /// </summary>
        /// <param name="rollingUtilisation">The sorted dictionary of rolling 24 hour section utilisation.</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void writeRollingUtilistion(SortedDictionary<string, List<double>> rollingUtilisation, string aggregatedDestination, DateTime[] dateRange)
        {
            /* Create the microsoft excel references. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            excel.Visible = false;
            Workbook workbook = excel.Workbooks.Add(Type.Missing);
            Worksheet worksheet;

            /* Create the header details. */
            string[,] headerString = {{"", "Section:"},
                                      {"Start Time", "End Time"}};

            /* Pagenate the data for writing to excel. */
            int excelPageSize = 1000000;        /* Page size of the excel worksheet. */
            int excelPages = 1;                 /* Number of Excel pages to write. */

            int headerRows = headerString.GetLength(0);
            int headerColumns = headerString.GetLength(1);

            int rowOffset = 2;
            int columnOffset = 2;

            int numberOfTimePeriods = (int)(dateRange[1] - dateRange[0]).TotalHours;

            /* Adjust the excel page size or the number of pages to write. */
            if (numberOfTimePeriods < excelPageSize)
                excelPageSize = numberOfTimePeriods;
            else
                excelPages = (int)Math.Round((double)numberOfTimePeriods / excelPageSize + 0.5);

            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Set the active worksheet. */
                worksheet = workbook.Sheets[excelPage + 1];
                workbook.Sheets[excelPage + 1].Activate();
                Range topLeft = worksheet.Cells[1, 1];
                Range bottomRight = worksheet.Cells[headerRows, headerColumns];
                worksheet.get_Range(topLeft, bottomRight).Value2 = headerString;

                /* Define the arrays to write to file. */
                string[,] startTime = new string[numberOfTimePeriods, 1];
                string[,] endTime = new string[numberOfTimePeriods, 1];
                string[] sectionBoundaries = new string[rollingUtilisation.Count()];
                /* The total number of minutes the section is occupied fr a 24 hour period from startTme. */
                double[,] totalMinutesOccupied = new double[numberOfTimePeriods, rollingUtilisation.Count()];

                /* Loop through the data for each excel page. */
                for (int sectionIndex = 0; sectionIndex < rollingUtilisation.Count(); sectionIndex++)
                {
                    /* Extract the key from the dictionary. */
                    string key = rollingUtilisation.Keys.ToList()[sectionIndex];
                    sectionBoundaries[sectionIndex] = key;

                    /* Loop through each hourly aggregation period. */
                    for (int timeIdx = 0; timeIdx < numberOfTimePeriods; timeIdx++)
                    {
                        if (sectionIndex == 0)
                        {
                            /* Set the start and end times. */
                            startTime[timeIdx, 0] = dateRange[0].AddHours(timeIdx).ToString();
                            endTime[timeIdx, 0] = dateRange[0].AddHours(timeIdx + 24).ToString();
                        }

                        /* Extract the total number of minutes the section is occupied. */
                        totalMinutesOccupied[timeIdx, sectionIndex] = rollingUtilisation[key][timeIdx];
                    }

                }

                /* Write the data to file. */
                worksheet.Range[worksheet.Cells[1, 3], worksheet.Cells[1, rollingUtilisation.Count() + columnOffset]].Value2 = sectionBoundaries;
                worksheet.Range[worksheet.Cells[1 + rowOffset, 1], worksheet.Cells[numberOfTimePeriods + rowOffset, 1]].Value2 = startTime;
                worksheet.Range[worksheet.Cells[1 + rowOffset, 2], worksheet.Cells[numberOfTimePeriods + rowOffset, 2]].Value2 = endTime;
                worksheet.Range[worksheet.Cells[1 + rowOffset, 3], 
                    worksheet.Cells[numberOfTimePeriods + rowOffset, rollingUtilisation.Count() + columnOffset]].Value2 = totalMinutesOccupied;
                
            }

            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\rollingUtilisation_" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
            {
                Tools.isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing, false, false,
                XlSaveAsAccessMode.xlNoChange, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;
        }
        
        /// <summary>
        /// Write the wagon details to an excel file for later analysis.
        /// </summary>
        /// <param name="wagon">The list of wagon objects containing the origin, destinaiton and net weight</param>
        /// <param name="destinationFolder">The destination directory for the resulting file.</param>
        public static void writeWagonData(List<wagonDetails> wagon, string destinationFolder)
        {
            /* Maximum number of rows in an excel worksheet is 1,048,576 (round down to a nice number) */
            int maxExcelRows = 1048500;

            /* Create the microsfot excel references. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            Workbook workbook;
            Worksheet worksheet;

            /* Get the reference to the new workbook. */
            workbook = (Workbook)(excel.Workbooks.Add(""));

            /* Create the header details. */
            string[] headerString = { "Train ID", "Operator", "Commodity", "Wagon ID", "Origin", "Planned Destination", "Destination", 
                                  "Attatchment Time", "Detatchment Time", "Origin-Destination", "Net Weight", "Gross Weight" };

            /* Get the page size of the excel worksheet. */
            int header = 2;
            int excelPageSize = wagon.Count() - 1;
            int excelPages = 1;

            if (wagon.Count() > maxExcelRows)
            {
                excelPageSize = 1000000;
                excelPages = (int)Math.Round((double)wagon.Count() / excelPageSize + 0.5);
            }

            /* Deconstruct the wagon details into excel columns. */
            string[,] trainID = new string[excelPageSize, 1];
            string[,] trainOperator = new string[excelPageSize, 1];
            string[,] commodity = new string[excelPageSize, 1];
            string[,] wagonID = new string[excelPageSize, 1];
            string[,] Orig = new string[excelPageSize, 1];
            string[,] Planned = new string[excelPageSize, 1];
            string[,] Dest = new string[excelPageSize, 1];
            string[,] OrigDest = new string[excelPageSize, 1];
            DateTime[,] attatch = new DateTime[excelPageSize, 1];
            DateTime[,] detatch = new DateTime[excelPageSize, 1];
            double[,] netWeight = new double[excelPageSize, 1];
            double[,] grossWeight = new double[excelPageSize, 1];


            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Set the active worksheet. */
                worksheet = workbook.Sheets.Add(Type.Missing, Type.Missing, 1, Type.Missing) as Worksheet;
                worksheet.get_Range("A1", "L1").Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int j = 0; j < excelPageSize; j++)
                {
                    string origin = "";
                    string destination = "";
                    List<string> Location = new List<string>();

                    
                    /* Check we dont try to read more data than there really is. */
                    int checkIdx = j + excelPage * excelPageSize;
                    if (checkIdx < wagon.Count())
                    {
                        trainID[j, 0] = wagon[checkIdx].TrainID;
                        trainOperator[j, 0] = wagon[checkIdx].trainOperator.ToString();
                        commodity[j, 0] = wagon[checkIdx].commodity.ToString();
                        wagonID[j, 0] = wagon[checkIdx].wagonID;
                        Orig[j, 0] = wagon[checkIdx].origin;
                        Planned[j, 0] = wagon[checkIdx].plannedDestination;
                        Dest[j, 0] = wagon[checkIdx].destination;
                        attatch[j, 0] = wagon[checkIdx].attachmentTime;
                        detatch[j, 0] = wagon[checkIdx].detachmentTime;

                        if (locationDictionary.TryGetValue(wagon[checkIdx].origin, out Location))
                            origin = Location[3];

                        if (locationDictionary.TryGetValue(wagon[checkIdx].destination, out Location))
                            destination = Location[3];

                        OrigDest[j, 0] = origin+" - "+destination;
                        
                        netWeight[j, 0] = wagon[checkIdx].netWeight;
                        grossWeight[j, 0] = wagon[checkIdx].grossWeight;
                    }
                    else
                    {
                        /* The end of the data has been reached. Replace the previous values with empty data. */
                        trainID[j, 0] = "";
                        trainOperator[j, 0] = "";
                        commodity[j, 0] = "";
                        wagonID[j, 0] = "";
                        Orig[j, 0] = "";
                        Planned[j, 0] = "";
                        Dest[j, 0] = "";
                        attatch[j, 0] = DateTime.MinValue;
                        detatch[j, 0] = DateTime.MinValue;
                        OrigDest[j, 0] = "";
                        netWeight[j, 0] = 0;
                        grossWeight[j, 0] = 0;
                    }
                }

                /* Write the data to the active excel workseet. */
                worksheet.get_Range("A" + header, "A" + (header + excelPageSize - 1)).Value2 = trainID;
                worksheet.get_Range("B" + header, "B" + (header + excelPageSize - 1)).Value2 = trainOperator;
                worksheet.get_Range("C" + header, "C" + (header + excelPageSize - 1)).Value2 = commodity;
                worksheet.get_Range("D" + header, "D" + (header + excelPageSize - 1)).Value2 = wagonID;
                worksheet.get_Range("E" + header, "E" + (header + excelPageSize - 1)).Value2 = Orig;
                worksheet.get_Range("F" + header, "F" + (header + excelPageSize - 1)).Value2 = Planned;
                worksheet.get_Range("G" + header, "G" + (header + excelPageSize - 1)).Value2 = Dest;
                worksheet.get_Range("H" + header, "H" + (header + excelPageSize - 1)).Value2 = attatch;
                worksheet.get_Range("I" + header, "I" + (header + excelPageSize - 1)).Value2 = detatch;
                worksheet.get_Range("J" + header, "J" + (header + excelPageSize - 1)).Value2 = OrigDest;
                worksheet.get_Range("K" + header, "K" + (header + excelPageSize - 1)).Value2 = netWeight;
                worksheet.get_Range("L" + header, "L" + (header + excelPageSize - 1)).Value2 = grossWeight;

            }

            string saveFilename = destinationFolder + @"\wagonDetails_" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
                File.Delete(saveFilename);

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing,
                false, false, XlSaveAsAccessMode.xlNoChange,
                Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;
        }

        /// <summary>
        /// Write the volume data to an excel file for analysis.
        /// </summary>
        /// <param name="volume">The list of volume objects containing the final origin destination details.</param>
        /// <param name="destinationFolder">The destination directory for the resulting file.</param>
        public static void writeVolumeData(List<volumeMovement> volume, string destinationFolder)
        {
            /* Maximum number of rows in an excel worksheet is 1,048,576 (round down to a nice number) */
            int maxExcelRows = 1048500;

            /* Create the microsfot excel references. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            Workbook workbook;
            Worksheet worksheet;

            /* Get the reference to the new workbook. */
            workbook = (Workbook)(excel.Workbooks.Add(""));

            /* Create the header details. */
            string[] headerString = { "Train ID", "Operator", "Commodity", "Wagon ID", 
                                        "Origin", "Origin SA4", "Origin State", "Origin Location",
                                        "Via", "Via SA4", "Via State", 
                                        "Destination", "Destination SA4", "Destination State", "Destination Location",
                                        "Attachment", "Detachment", "Origin-Destination", "Net Weight", "Gross Weight" };

            /* Get the page size of the excel worksheet. */
            int header = 2;
            int excelPageSize = volume.Count() - 1;
            int excelPages = 1;

            if (volume.Count() > maxExcelRows)
            {
                excelPageSize = 1000000;
                excelPages = (int)Math.Round((double)volume.Count() / excelPageSize + 0.5);
            }

            /* Deconstruct the volume details into excel columns. */
            string[,] trainID = new string[excelPageSize, 1];
            string[,] trainOperator = new string[excelPageSize, 1];
            string[,] commodity = new string[excelPageSize, 1];
            string[,] wagonID = new string[excelPageSize, 1];
            string[,] Orig = new string[excelPageSize, volume[0].Origin.Count()];
            string[,] Via = new string[excelPageSize, volume[0].Via.Count()];
            string[,] Dest = new string[excelPageSize, volume[0].Destination.Count()];
            double[,] netWeight = new double[excelPageSize, 1];
            double[,] grossWeight = new double[excelPageSize, 1];
            string[,] OriginDestination = new string[excelPageSize, 1];
                
            DateTime[,] attachment = new DateTime[excelPageSize, 1];
            DateTime[,] detachment = new DateTime[excelPageSize, 1];

            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Set the active worksheet. */
                worksheet = workbook.Sheets.Add(Type.Missing, Type.Missing, 1, Type.Missing) as Worksheet;
                worksheet.get_Range("A1", "T1").Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int j = 0; j < excelPageSize; j++)
                {
                    /* Check we dont try to read more data than there really is. */
                    int checkIdx = j + excelPage * excelPageSize;
                    if (checkIdx < volume.Count())
                    {
                        trainID[j, 0] = volume[checkIdx].trainID;
                        trainOperator[j, 0] = volume[checkIdx].trainOperator.ToString();
                        commodity[j, 0] = volume[checkIdx].commodity.ToString();
                        wagonID[j, 0] = volume[checkIdx].wagonID;
                        for (int locationIdx = 0; locationIdx < volume[checkIdx].Origin.Count(); locationIdx++)
                        {
                            Orig[j, locationIdx] = volume[checkIdx].Origin[locationIdx];
                            Via[j, locationIdx] = volume[checkIdx].Via[locationIdx];
                            Dest[j, locationIdx] = volume[checkIdx].Destination[locationIdx];
                        }
                        OriginDestination[j, 0] = volume[checkIdx].Origin[3] + " - " + volume[checkIdx].Destination[3] + " - " + volume[checkIdx].trainOperator;

                        netWeight[j, 0] = volume[checkIdx].netWeight;
                        grossWeight[j, 0] = volume[checkIdx].grossWeight;

                        attachment[j, 0] = volume[checkIdx].attachmentTime;
                        detachment[j, 0] = volume[checkIdx].detachmentTime;
                    }
                    else
                    {
                        /* The end of the data has been reached. Populate the remaining elements. */
                        trainID[j, 0] = "";
                        trainOperator[j, 0] = "";
                        commodity[j, 0] = "";
                        wagonID[j, 0] = "";
                        for (int locationIdx = 0; locationIdx < volume[0].Origin.Count(); locationIdx++)
                        {
                            Orig[j, locationIdx] = "";
                            Via[j, locationIdx] = "";
                            Dest[j, locationIdx] = "";
                        }
                        OriginDestination[j, 0] = "";

                        netWeight[j, 0] = 0;
                        grossWeight[j, 0] = 0;

                        attachment[j, 0] = DateTime.MinValue;
                        detachment[j, 0] = DateTime.MinValue;
                    }
                }

                /* Write the data to the active excel workseet. */
                worksheet.get_Range("A" + header, "A" + (header + excelPageSize - 1)).Value2 = trainID;
                worksheet.get_Range("B" + header, "B" + (header + excelPageSize - 1)).Value2 = trainOperator;
                worksheet.get_Range("C" + header, "C" + (header + excelPageSize - 1)).Value2 = commodity;
                worksheet.get_Range("D" + header, "D" + (header + excelPageSize - 1)).Value2 = wagonID;
                worksheet.get_Range("E" + header, "H" + (header + excelPageSize - 1)).Value2 = Orig;
                worksheet.get_Range("I" + header, "K" + (header + excelPageSize - 1)).Value2 = Via;
                worksheet.get_Range("L" + header, "O" + (header + excelPageSize - 1)).Value2 = Dest;
                worksheet.get_Range("P" + header, "P" + (header + excelPageSize - 1)).Value2 = attachment;
                worksheet.get_Range("Q" + header, "Q" + (header + excelPageSize - 1)).Value2 = detachment;
                worksheet.get_Range("R" + header, "R" + (header + excelPageSize - 1)).Value2 = OriginDestination;
                worksheet.get_Range("S" + header, "S" + (header + excelPageSize - 1)).Value2 = netWeight;
                worksheet.get_Range("T" + header, "T" + (header + excelPageSize - 1)).Value2 = grossWeight;
                

            }

            string saveFilename = destinationFolder + @"\volumeDetails_" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
                File.Delete(saveFilename);

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing,
                false, false, XlSaveAsAccessMode.xlNoChange,
                Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;
        }
        
        /// <summary>
        /// Write the volume data to an excel file categorised by the commodity for analysis.
        /// </summary>
        /// <param name="volume">The list of volume objects containing the final origin destination details.</param>
        /// <param name="destinationFolder">The destination directory for the resulting file.</param>
        public static void writeVolumeDataByCommodity(List<volumeMovement> volume, string destinationFolder)
        {
            /* Maximum number of rows in an excel worksheet is 1,048,576 (round down to a nice number) */
            int maxExcelRows = 1048500;

            /* Create the microsfot excel references. */
            Microsoft.Office.Interop.Excel.Application excel = new Microsoft.Office.Interop.Excel.Application();
            Workbook workbook;
            Worksheet worksheet;

            /* Get the reference to the new workbook. */
            workbook = (Workbook)(excel.Workbooks.Add(""));

            /* Create the header details. */
            string[] headerString = { "Train ID", "Operator", "Commodity", "Wagon ID", 
                                        "Origin", "Origin SA4", "Origin State", "Origin Location", 
                                        "Via", "Via SA4", "Via State", 
                                        "Destination", "Destination SA4", "Destination State", "Destination Location",
                                        "Attachment", "Detachment", "Origin-Destination", "Net Weight", "Gross Weight" };
                        
            /* Get the page size of the excel worksheet. */
            int header = 2;

            /* Create a list of commodities. */
            List<trainCommodity> commodities = volume.Select(v => v.commodity).Distinct().ToList();
            List<volumeMovement> currentVolumeList = new List<volumeMovement>();

            /* Cycle through each commodity */
            for (int i = 0; i < commodities.Count; i++)
            {
                /* Get the current list of volume movements with the identified commodity. */
                currentVolumeList = volume.Where(v => v.commodity == commodities[i]).ToList();

                int excelPageSize = currentVolumeList.Count() - 1;
                int excelPages = 1;

                /* Set the number of pages for each commodity. */
                if (currentVolumeList.Count() > maxExcelRows)
                {
                    excelPageSize = 1000000;
                    excelPages = (int)Math.Round((double)currentVolumeList.Count() / excelPageSize + 0.5);
                }

                /* Deconstruct the volume details into excel columns. */
                string[,] trainID = new string[excelPageSize, 1];
                string[,] trainOperator = new string[excelPageSize, 1];
                string[,] commodity = new string[excelPageSize, 1];
                string[,] wagonID = new string[excelPageSize, 1];
                string[,] Orig = new string[excelPageSize, volume[0].Origin.Count()];
                string[,] Via = new string[excelPageSize, volume[0].Via.Count()];
                string[,] Dest = new string[excelPageSize, volume[0].Destination.Count()];
                string[,] OriginDestination = new string[excelPageSize, 1];
                
                double[,] netWeight = new double[excelPageSize, 1];
                double[,] grossWeight = new double[excelPageSize, 1];
                DateTime[,] attachment = new DateTime[excelPageSize, 1];
                DateTime[,] detachment = new DateTime[excelPageSize, 1];
                
                /* Loop through the excel pages. */
                for (int excelPage = 0; excelPage < excelPages; excelPage++)
                {
                    /* Set the active worksheet. */
                    worksheet = workbook.Sheets.Add(Type.Missing, Type.Missing, 1, Type.Missing) as Worksheet;
                    if (excelPage == 0)
                        worksheet.Name = commodities[i].ToString();
                    else
                        worksheet.Name = string.Format("{0} {1}",commodities[i].ToString(),excelPage.ToString());

                    worksheet.get_Range("A1", "T1").Value2 = headerString;

                    /* Loop through the data for each excel page. */
                    for (int j = 0; j < excelPageSize; j++)
                    {
                        /* Check we dont try to read more data than there really is. */
                        int checkIdx = j + excelPage * excelPageSize;
                        if (checkIdx < currentVolumeList.Count())
                        {
                            trainID[j, 0] = currentVolumeList[checkIdx].trainID;
                            trainOperator[j, 0] = currentVolumeList[checkIdx].trainOperator.ToString();
                            commodity[j, 0] = currentVolumeList[checkIdx].commodity.ToString();
                            wagonID[j, 0] = currentVolumeList[checkIdx].wagonID;
                            for (int locationIdx = 0; locationIdx < currentVolumeList[checkIdx].Origin.Count(); locationIdx++)
                            {
                                Orig[j, locationIdx] = currentVolumeList[checkIdx].Origin[locationIdx];
                                Via[j, locationIdx] = currentVolumeList[checkIdx].Via[locationIdx];
                                Dest[j, locationIdx] = currentVolumeList[checkIdx].Destination[locationIdx];
                            }
                            //OriginDestination[j, 0] = currentVolumeList[checkIdx].Origin[3] + " - " + currentVolumeList[checkIdx].Destination[3] + " - " + currentVolumeList[checkIdx].trainOperator;
                            OriginDestination[j, 0] = currentVolumeList[checkIdx].OriginDestination + " - " + currentVolumeList[checkIdx].trainOperator;

                            netWeight[j, 0] = currentVolumeList[checkIdx].netWeight;
                            grossWeight[j, 0] = currentVolumeList[checkIdx].grossWeight;

                            attachment[j, 0] = currentVolumeList[checkIdx].attachmentTime;
                            detachment[j, 0] = currentVolumeList[checkIdx].detachmentTime;
                        }
                        else
                        {
                            /* The end of the data has been reached. Populate the remaining elements. */
                            trainID[j, 0] = "";
                            trainOperator[j, 0] = "";
                            commodity[j, 0] = "";
                            wagonID[j, 0] = "";
                            for (int locationIdx = 0; locationIdx < currentVolumeList[0].Origin.Count(); locationIdx++)
                            {
                                Orig[j, locationIdx] = "";
                                Via[j, locationIdx] = "";
                                Dest[j, locationIdx] = "";
                            }
                            OriginDestination[j, 0] = "";

                            netWeight[j, 0] = 0;
                            grossWeight[j, 0] = 0;

                            attachment[j, 0] = DateTime.MinValue;
                            detachment[j, 0] = DateTime.MinValue;
                        }
                    }

                    /* Write the data to the active excel workseet. */
                    worksheet.get_Range("A" + header, "A" + (header + excelPageSize - 1)).Value2 = trainID;
                    worksheet.get_Range("B" + header, "B" + (header + excelPageSize - 1)).Value2 = trainOperator;
                    worksheet.get_Range("C" + header, "C" + (header + excelPageSize - 1)).Value2 = commodity;
                    worksheet.get_Range("D" + header, "D" + (header + excelPageSize - 1)).Value2 = wagonID;
                    worksheet.get_Range("E" + header, "H" + (header + excelPageSize - 1)).Value2 = Orig;
                    worksheet.get_Range("I" + header, "K" + (header + excelPageSize - 1)).Value2 = Via;
                    worksheet.get_Range("L" + header, "O" + (header + excelPageSize - 1)).Value2 = Dest;
                    worksheet.get_Range("P" + header, "P" + (header + excelPageSize - 1)).Value2 = attachment;
                    worksheet.get_Range("Q" + header, "Q" + (header + excelPageSize - 1)).Value2 = detachment;
                    worksheet.get_Range("R" + header, "R" + (header + excelPageSize - 1)).Value2 = OriginDestination;
                    worksheet.get_Range("S" + header, "S" + (header + excelPageSize - 1)).Value2 = netWeight;
                    worksheet.get_Range("T" + header, "T" + (header + excelPageSize - 1)).Value2 = grossWeight;
                    
                }   // end of excel page loop
                
            }   // End of commodity loop


            string saveFilename = destinationFolder + @"\volumeDetails_" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

            /* Check the file does not exist yet. */
            if (File.Exists(saveFilename))
                File.Delete(saveFilename);

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing,
                false, false, XlSaveAsAccessMode.xlNoChange,
                Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();
            excel.Quit();

            return;
        }

        /// <summary>
        /// Determine the direction of travel based on the direction string. 
        /// The string must match the enumerated string values.
        /// </summary>
        /// <param name="subDirection">The string representing the train direction.</param>
        /// <returns></returns>
        private static direction getDirection(string subDirection)
        {
            if (direction.IncreasingKm.ToString().Equals(subDirection, StringComparison.OrdinalIgnoreCase))
                return direction.IncreasingKm;
            else if (direction.DecreasingKm.ToString().Equals(subDirection, StringComparison.OrdinalIgnoreCase))
                return direction.DecreasingKm;
            else
                return direction.Unknown;

        }

        /// <summary>
        /// Identify the train operator from the first few letters of the field.
        /// </summary>
        /// <param name="shortOperator">The first few letters of the operator field.</param>
        /// <returns>A train operator object identifying the train operator.</returns>
        private static trainOperator getOperator(string shortOperator)
        {
            int operatorStringLength = 6;

            if (shortOperator.Count() >= operatorStringLength)
                shortOperator = shortOperator.Substring(0, operatorStringLength);
            else
                shortOperator = shortOperator.PadRight(operatorStringLength);

            /* Compare each train operator to the supplied string to identify the correct operator. */
            if (shortOperator.Equals("Austra", StringComparison.OrdinalIgnoreCase))
                return trainOperator.ARTC;
            else if (shortOperator.Equals("ARTC  ", StringComparison.OrdinalIgnoreCase))
                return trainOperator.ARTC;
            else if (shortOperator.Equals("Aust R", StringComparison.OrdinalIgnoreCase))
                return trainOperator.AustralianRailwaysHistoricalSociety;
            else if (shortOperator.Equals("Aurizo", StringComparison.OrdinalIgnoreCase))
                return trainOperator.Aurizon;
            else if (shortOperator.Equals("QR Nat", StringComparison.OrdinalIgnoreCase))
                return trainOperator.Aurizon;
            else if (shortOperator.Equals("City R", StringComparison.OrdinalIgnoreCase))
                return trainOperator.CityRail;
            else if (shortOperator.Equals("Countr", StringComparison.OrdinalIgnoreCase))
                return trainOperator.Countrylink;
            else if (shortOperator.Equals("Freigh", StringComparison.OrdinalIgnoreCase))
                return trainOperator.Freightliner;
            else if (shortOperator.Equals("Geness", StringComparison.OrdinalIgnoreCase))
                return trainOperator.GenesseeWyoming;
            else if (shortOperator.Equals("Great ", StringComparison.OrdinalIgnoreCase))
                return trainOperator.GreatSouthernRail;
            else if (shortOperator.Equals("intera", StringComparison.OrdinalIgnoreCase))
                return trainOperator.Interail;
            else if (shortOperator.Equals("John H", StringComparison.OrdinalIgnoreCase))
                return trainOperator.JohnHollandRail;
            else if (shortOperator.Equals("Lauchl", StringComparison.OrdinalIgnoreCase))
                return trainOperator.LauchlanValleyRailSociety;
            else if (shortOperator.Equals("Pac Na", StringComparison.OrdinalIgnoreCase))
                return trainOperator.PacificNational;
            else if (shortOperator.Equals("Pacifi", StringComparison.OrdinalIgnoreCase))
                return trainOperator.PacificNational;
            else if (shortOperator.Equals("QUBE L", StringComparison.OrdinalIgnoreCase))
                return trainOperator.QUBE;
            else if (shortOperator.Equals("RailCo", StringComparison.OrdinalIgnoreCase))
                return trainOperator.RailCorp;
            else if (shortOperator.Equals("Rail T", StringComparison.OrdinalIgnoreCase))
                return trainOperator.RailTransportMuseum;
            else if (shortOperator.Equals("SCT   ", StringComparison.OrdinalIgnoreCase))
                return trainOperator.SCT;
            else if (shortOperator.Equals("Southe", StringComparison.OrdinalIgnoreCase))
                return trainOperator.SouthernShorthaulRail;
            else if (shortOperator.Equals("Sydney", StringComparison.OrdinalIgnoreCase))
                return trainOperator.SydneyRailService;
            else if (shortOperator.Equals("TheRai", StringComparison.OrdinalIgnoreCase))
                return trainOperator.TheRailMotorService;
            else if (shortOperator.Equals("V Line", StringComparison.OrdinalIgnoreCase))
                return trainOperator.VLinePassenger;
            else if (shortOperator.Equals("Combin", StringComparison.OrdinalIgnoreCase))
                return trainOperator.Combined;
            else if (shortOperator.Equals("Groupe", StringComparison.OrdinalIgnoreCase))
                return trainOperator.GroupRemaining;
            else if (shortOperator.Equals("GroupR", StringComparison.OrdinalIgnoreCase))
                return trainOperator.GroupRemaining;
            else if (shortOperator.Equals("Simula", StringComparison.OrdinalIgnoreCase))
                return trainOperator.Simulated;
            else
                return trainOperator.Unknown;

        }

        /// <summary>
        /// Identify the commodity from the first few letters of the field.
        /// </summary>
        /// <param name="commodity">The first few letters of the operator field.</param>
        /// <returns>A train commodity object identifying the commodity.</returns>
        private static trainCommodity getCommodity(string commodity)
        {
            /* List of individual commodities available seperated into commodity types. */
            string[] Clinker = { "Clinker" };
            string[] Coal = { "Coal", "Coal Export", "Containersied Coal", "Coal Domestic" };
            string[] Freight = { "General Freight" };
            string[] Grain = { "Grain" };
            string[] Intermodal = { "Intermodal" };
            string[] Minerals = { "Minerals" };
            string[] Passenger = { "Passenger" };
            string[] Steel = { "Steel" };
            string[] Work = { "Unspecified Commodity" };

            /* Compare each commodity to the supplied string to identify the correct commodity. */
            if (Clinker.Contains(commodity))
                return trainCommodity.Clinker;
            else if (Coal.Contains(commodity))
                return trainCommodity.Coal;
            else if (Freight.Contains(commodity))
                return trainCommodity.GeneralFreight;
            else if (Grain.Contains(commodity))
                return trainCommodity.Grain;
            else if (Intermodal.Contains(commodity))
                return trainCommodity.Intermodal;
            else if (Minerals.Contains(commodity))
                return trainCommodity.Mineral;
            else if (Passenger.Contains(commodity))
                return trainCommodity.Passenger;
            else if (Steel.Contains(commodity))
                return trainCommodity.Steel;
            else if (Work.Contains(commodity))
                return trainCommodity.Work;
            else
                return trainCommodity.Unknown;
        }

        
    }
}
