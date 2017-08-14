using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows.Forms;
using Microsoft.Office.Interop.Excel;

using TrainLibrary;
using Statistics;

namespace IOLibrary
{
    public class FileOperations
    {
        /* ARTC location code file */
        public static string geoLocationFile = @"H:\ARTC GEO Location Details - under construction.csv";
        
        /* Create a dictionary of locations:
         * Key:     Location Code [3 letter code]
         * Values:  Location Name, Location SA4 region, Location State.
         */
        public static Dictionary<string, List<string>> locationDictioanry = new Dictionary<string, List<string>>();

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
            Category Category = Category.Unknown;

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

                    if (loop.Equals("loop", StringComparison.OrdinalIgnoreCase) || loop.Equals("true", StringComparison.OrdinalIgnoreCase))
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
        /// The file is assumes the data has been extracted from Tableau 
        /// and hence has a specific file format.
        /// Column  Field
        ///  0      Origin
        ///  1      Planned Destination
        ///  2      Attachment Time
        ///  3      Class
        ///  4      Destination
        ///  5      Detatchment Time
        ///  6      Class Number
        ///  7      Train Operator
        ///  8      Train Date
        ///  9      Train ID
        ///  10     Train Type (Commodity)
        ///  11     Distance
        ///  12     Gross Mass
        ///  13     Move Count
        ///  14     Power Ratio
        ///  15     Tare Mass
        /// </summary>
        /// <param name="filename">The wagon data file.</param>
        /// <returns>The list of wagon objects.</returns>
        public static List<wagonDetails> readWagonDataFile(string filename)
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
            double netWeight = 0;

            /* Create the list of wagon objects. */
            List<wagonDetails> wagon = new List<wagonDetails>();

            /* Validate the format of the first line of the file, ignoring the header information */
            bool validFormat = false;
            bool validRecord = false;
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
                    string trainID = fields[9];
                    DateTime.TryParse(fields[8], out trainDate);
                    trainOperator trainOperator = Processing.getWagonOperator(fields[7]);
                    trainCommodity commodity = Processing.getWagonCommodity(fields[10]);

                    /* Extract the wagon location information and validate the codes. */
                    wagonID = fields[3] + " " + Regex.Replace(fields[6], ",", "");
                    double wagonTest;
                    if (double.TryParse(fields[3], out wagonTest))
                        validRecord = false;
                    else
                        validRecord = true;

                    /* Wagon Origin. */
                    origin = fields[0].ToUpper();
                    if (origin.Count() != 3)
                        Tools.messageBox("Origin location code is unknown: {0} Unknown location code.", origin);

                    /* Wagon planned destination. */
                    plannedDestination = fields[1].ToUpper();
                    if (plannedDestination.Count() != 3)
                        Tools.messageBox("Consigned Destination location code in unknown: {0} Unknown location code.", plannedDestination);

                    /* Wagon destination. */
                    destination = fields[4].ToUpper();
                    if (destination.Count() != 3)
                    {   /* If the destination field is empty, assume the wagon reaches the planned destination. */
                        if (destination.Equals(""))
                            destination = plannedDestination;
                        else
                            Tools.messageBox("Destination location code is unknown: {0} Unknown location code.", destination);
                    }

                    /* Extract remaining wagon details. */
                    DateTime.TryParse(fields[2], out attachmentTime);
                    DateTime.TryParse(fields[5], out detachmentTime);
                    double.TryParse(fields[15], out tareWeight);
                    double.TryParse(fields[12], out grossWeight);
                    netWeight = grossWeight - tareWeight;

                    /* Construct the wagon object and add to the list. */
                    if (validRecord)
                    {
                        wagonDetails data = new wagonDetails(trainID, trainDate, trainOperator, commodity, wagonID, origin, plannedDestination, destination, attachmentTime, detachmentTime, netWeight);
                        wagon.Add(data);
                    }
                }
            }
            /* Return the completed wagon List. */
            return wagon;
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
                                     { "", "Direction:", "" }};


            /* Pagenate the data for writing to excel. */
            int excelPageSize = 1000000;        /* Page size of the excel worksheet. */
            int excelPages = 1;                 /* Number of Excel pages to write. */
            int headerOffset = 9;

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
                worksheet.Range[worksheet.Cells[7, 3], worksheet.Cells[7, trainRecords.Count() + 2]].Value2 = direction;

                worksheet.Range[worksheet.Cells[headerOffset, 1], worksheet.Cells[headerOffset + trainRecords[0].journey.Count() - 1, 1]].Value2 = kilometerage;
                worksheet.Range[worksheet.Cells[headerOffset, 3], worksheet.Cells[headerOffset + trainRecords[0].journey.Count() - 1, 3 + trainRecords.Count() - 1]].Value2 = speed;

            }

            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\ICEData_InterpolatedTrains" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

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
            int excelPageSize = 1000000;        /* Page size of the excel worksheet. */
            int excelPages = 1;                 /* Number of Excel pages to write. */

            int headerOffset = 9;
            int horizontalOffset = 3;

            int headerRows = headerString.GetLength(0);
            int headerColumns = headerString.GetLength(1);

            int displayColumn = horizontalOffset; 

            int columnOffset = 4;
            
            /* Adjust the excel page size or the number of pages to write. */
            if (trainRecords.Count() < excelPageSize)
                excelPageSize = trainRecords.Count();
            else
                excelPages = (int)Math.Round((double)trainRecords.Count() / excelPageSize + 0.5);

            /* Deconstruct the train details into excel columns. */
            string[,] TrainID = new string[1, trainRecords.Count() * columnOffset];
            string[,] LocoID = new string[1, trainRecords.Count() * columnOffset];
            double[,] powerToWeight = new double[1, trainRecords.Count() * columnOffset];
            string[,] commodity = new string[1, trainRecords.Count() * columnOffset];
            string[,] direction = new string[1, trainRecords.Count() * columnOffset];
            DateTime[,] trainDate = new DateTime[1, trainRecords.Count() * columnOffset];


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
                for (int trainIdx = 0; trainIdx < trainRecords.Count(); trainIdx++)
                {

                    int displayRow = headerOffset + trainRecords[trainIdx].journey.Count() - 1;
                    
                    double[,] kilometerage = new double[trainRecords[trainIdx].journey.Count(), 1];

                    double[,] speed = new double[trainRecords[trainIdx].journey.Count(), 1];
                    DateTime[,] dateTime = new DateTime[trainRecords[trainIdx].journey.Count(), 1];


                    TrainID[0, trainIdx * columnOffset] = trainRecords[trainIdx].trainID;
                    LocoID[0, trainIdx * columnOffset] = trainRecords[trainIdx].locoID;
                    trainDate[0, trainIdx * columnOffset] = trainRecords[trainIdx].journey.Where(t => t.dateTime > DateTime.MinValue).ToList().Min(t => t.dateTime);

                    powerToWeight[0, trainIdx * columnOffset] = trainRecords[trainIdx].powerToWeight;

                    commodity[0, trainIdx * columnOffset] = trainRecords[trainIdx].commodity.ToString();

                    direction[0, trainIdx * columnOffset] = trainRecords[trainIdx].trainDirection.ToString();

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


                    for (int journeyIdx = 0; journeyIdx < trainRecords[trainIdx].journey.Count(); journeyIdx++)
                    {
                        kilometerage[journeyIdx, 0] = trainRecords[trainIdx].journey[journeyIdx].kilometreage;

                        speed[journeyIdx, 0] = trainRecords[trainIdx].journey[journeyIdx].speed;
                        dateTime[journeyIdx, 0] = trainRecords[trainIdx].journey[journeyIdx].dateTime;

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
                worksheet.Range[worksheet.Cells[2, 3], worksheet.Cells[2, trainRecords.Count() * columnOffset + 2]].Value2 = TrainID;
                worksheet.Range[worksheet.Cells[3, 3], worksheet.Cells[3, trainRecords.Count() * columnOffset + 2]].Value2 = LocoID;
                worksheet.Range[worksheet.Cells[4, 3], worksheet.Cells[4, trainRecords.Count() * columnOffset + 2]].Value2 = trainDate;
                worksheet.Range[worksheet.Cells[5, 3], worksheet.Cells[5, trainRecords.Count() * columnOffset + 2]].Value2 = powerToWeight;
                worksheet.Range[worksheet.Cells[6, 3], worksheet.Cells[6, trainRecords.Count() * columnOffset + 2]].Value2 = commodity;
                worksheet.Range[worksheet.Cells[7, 3], worksheet.Cells[7, trainRecords.Count() * columnOffset + 2]].Value2 = direction;

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
            int excelPageSize = 1000000;        /* Page size of the excel worksheet. */
            int excelPages = 1;                 /* Number of Excel pages to write. */

            int headerOffset = 9;
            int horizontalOffset = 3;

            int headerRows = headerString.GetLength(0);
            int headerColumns = headerString.GetLength(1);

            int displayRow = headerOffset + trainRecords[0].journey.Count() - 1;
            int displayColumn = horizontalOffset; // +trainRecords.Count() - 1;

            int timeOffset = 5;
            int timedataOffset = displayRow + timeOffset;

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
            string[,] direction = new string[1, trainRecords.Count()];
            DateTime[,] trainDate = new DateTime[1, trainRecords.Count()];


            double[,] kilometerage = new double[trainRecords[0].journey.Count(), 1];

            double[,] speed = new double[trainRecords[0].journey.Count(), 1]; //trainRecords.Count()];
            DateTime[,] dateTime = new DateTime[trainRecords[0].journey.Count(), 1]; // trainRecords.Count()];

            //double[,] speed = new double[6000, 2000];
            //DateTime[,] dateTime = new DateTime[6000,2000];


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
                for (int trainIdx = 0; trainIdx < trainRecords.Count(); trainIdx++)
                {
                    TrainID[0, trainIdx] = trainRecords[trainIdx].trainID;
                    LocoID[0, trainIdx] = trainRecords[trainIdx].locoID;
                    trainDate[0, trainIdx] = trainRecords[trainIdx].journey.Where(t => t.dateTime > DateTime.MinValue).ToList().Min(t => t.dateTime);

                    powerToWeight[0, trainIdx] = trainRecords[trainIdx].powerToWeight;

                    commodity[0, trainIdx] = trainRecords[trainIdx].commodity.ToString();

                    direction[0, trainIdx] = trainRecords[trainIdx].trainDirection.ToString();

                    for (int journeyIdx = 0; journeyIdx < trainRecords[trainIdx].journey.Count(); journeyIdx++)
                    {
                        //Console.WriteLine("{0}:: {1}",trainIdx,journeyIdx);

                        kilometerage[journeyIdx, 0] = startKm + interpoaltionInterval * Processing.metresToKilometers * journeyIdx;

                        speed[journeyIdx, 0] = trainRecords[trainIdx].journey[journeyIdx].speed;
                        dateTime[journeyIdx, 0] = trainRecords[trainIdx].journey[journeyIdx].dateTime;

                    }
                    /* Reduce memory needs
                     * Reduce the speed and dataTiem decleration.
                     * Write each trains journey details individually here.
                     */
                    worksheet.Range[worksheet.Cells[headerOffset, horizontalOffset], worksheet.Cells[displayRow, displayColumn]].Value2 = speed;
                    worksheet.Range[worksheet.Cells[timedataOffset, horizontalOffset], worksheet.Cells[timedataOffset + trainRecords[0].journey.Count() - 1, displayColumn]].Value2 = dateTime;
                    displayColumn++;
                    horizontalOffset++;
                    
                }

                /* Write the data to the active excel workseet. */
                worksheet.Range[worksheet.Cells[2, 3], worksheet.Cells[2, trainRecords.Count() + 2]].Value2 = TrainID;
                worksheet.Range[worksheet.Cells[3, 3], worksheet.Cells[3, trainRecords.Count() + 2]].Value2 = LocoID;
                worksheet.Range[worksheet.Cells[4, 3], worksheet.Cells[4, trainRecords.Count() + 2]].Value2 = trainDate;
                worksheet.Range[worksheet.Cells[5, 3], worksheet.Cells[5, trainRecords.Count() + 2]].Value2 = powerToWeight;
                worksheet.Range[worksheet.Cells[6, 3], worksheet.Cells[6, trainRecords.Count() + 2]].Value2 = commodity;
                worksheet.Range[worksheet.Cells[7, 3], worksheet.Cells[7, trainRecords.Count() + 2]].Value2 = direction;

                worksheet.Range[worksheet.Cells[headerOffset, 1], worksheet.Cells[displayRow, 1]].Value2 = kilometerage;
                //worksheet.Range[worksheet.Cells[headerOffset, horizontalOffset], worksheet.Cells[displayRow, displayColumn]].Value2 = speed;

                worksheet.Range[worksheet.Cells[timedataOffset, 1], worksheet.Cells[timedataOffset + trainRecords[0].journey.Count() - 1, 1]].Value2 = kilometerage;
                //worksheet.Range[worksheet.Cells[timedataOffset, horizontalOffset], worksheet.Cells[timedataOffset + trainRecords[0].journey.Count() - 1, displayColumn]].Value2 = dateTime;


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

            return;
        }

        /// <summary>
        /// Write the aggregated data to a file for evaluation.
        /// </summary>
        /// <param name="averageTrains">List of aggregated train journies.</param>
        /// <param name="stats">The statstics generated for each average train</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void wrtieAverageData(List<AverageTrain> averageTrains, List<TrainStatistics> stats, string aggregatedDestination)
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
            headerString.Add("Loops");
            headerString.Add("TSRs");

            /* Pagenate the data for writing to excel. */
            int numberOfPoints = averageTrains[0].kilometreage.Count();
            int headerOffset = statisticsHeader.GetLength(0) + 3;

            /* Deconstruct the train details into excel columns. */
            double[,] kilometerage = new double[numberOfPoints, 1];
            double[,] elevation = new double[numberOfPoints, 1];
            string[,] isLoophere = new string[numberOfPoints, 1];
            string[,] isTSRhere = new string[numberOfPoints, 1];

            double[,] averageSpeedArray = new double[numberOfPoints, averageTrains.Count()];


            /* Set the active worksheet. */
            worksheet = (_Worksheet)workbook.Sheets[1];
            workbook.Sheets[1].Activate();

            /* Loop through the data for each excel page. */
            for (int i = 0; i < numberOfPoints; i++)
            {
                /* Populate the kilometerage and evlevation. */
                kilometerage[i, 0] = averageTrains[0].kilometreage[i];
                elevation[i, 0] = averageTrains[0].elevation[i];

                /* Identify where the loops and TSR are. */
                if (averageTrains[0].isInLoopBoundary[i])
                    isLoophere[i, 0] = "Loop Boundary";

                if (averageTrains[0].isInTSRboundary[i])
                    isTSRhere[i, 0] = "TSR Boundary";

                /* Extract the average speed for each analysis Category */
                for (int j = 0; j < averageTrains.Count(); j++)
                {
                    averageSpeedArray[i, j] = averageTrains[j].averageSpeed[i];
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
            bottomRight = worksheet.Cells[headerOffset, averageTrains.Count() + 2];
            worksheet.get_Range(topLeft, bottomRight).Value2 = directionHeader.ToArray();

            topLeft = worksheet.Cells[headerOffset + 1, 1];
            bottomRight = worksheet.Cells[headerOffset + 1, averageTrains.Count() + 4];
            worksheet.get_Range(topLeft, bottomRight).Value2 = headerString.ToArray();

            int dataOffset = headerOffset + 2;
            /* Write the data to the active excel workseet. */
            worksheet.get_Range("A" + dataOffset, "A" + (dataOffset + numberOfPoints - 1)).Value2 = kilometerage;
            worksheet.get_Range("B" + dataOffset, "B" + (dataOffset + numberOfPoints - 1)).Value2 = elevation;

            topLeft = worksheet.Cells[dataOffset, column];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count() - 1];
            worksheet.get_Range(topLeft, bottomRight).Value2 = averageSpeedArray;

            /* Increment the column for loop data. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count()];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count()];
            worksheet.get_Range(topLeft, bottomRight).Value2 = isLoophere;

            /* Increment the column for TSR data. */
            topLeft = worksheet.Cells[dataOffset, column + averageTrains.Count() + 1];
            bottomRight = worksheet.Cells[dataOffset + numberOfPoints - 1, column + averageTrains.Count() + 1];
            worksheet.get_Range(topLeft, bottomRight).Value2 = isTSRhere;

            /* Generate the resulting file name and location to save to. */
            string saveFilename = aggregatedDestination + @"\AverageSpeed_" + DateTime.Now.ToString("yyyyMMdd") + ".xlsx";

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

            return;

        }

        /// <summary>
        /// Write the train pairs data to file. The train speed and timing at each point will be 
        /// displayed for future analysis and investigation.
        /// </summary>
        /// <param name="pair">A list of train pairs.</param>
        /// <param name="loopLocations">A list of loop locations.</param>
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
                                     { "", "Direction:", "" },
                                     { "", "", "" },
                                     { "","Time for stopped train to reach track speed", "" },
                                     { "","Simulated train to reach track speed location", "" },
                                     { "","Time between clearing loop and restarting", "" },
                                     { "","Transaction time", "" }};
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

            /* Loop through each loop location, diaplaying all train pairs associated with each loop on seperate worksheets. */
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
                    worksheet.Range[worksheet.Cells[6, 3], worksheet.Cells[6, numberOfPairsAtLoop * 2 + 2]].Value2 = direction;
                    //worksheet.Range[worksheet.Cells[7, 3], worksheet.Cells[7, numberOfPairsAtLoop * 2 + 2]].Value2 = commodity;

                    worksheet.Range[worksheet.Cells[8, 3], worksheet.Cells[8, numberOfPairsAtLoop * 2 + 2]].Value2 = timeForStoppedTrainToReachTrackSpeed;
                    worksheet.Range[worksheet.Cells[9, 3], worksheet.Cells[9, numberOfPairsAtLoop * 2 + 2]].Value2 = simulatedTrainToReachTrackSpeedLocation;
                    worksheet.Range[worksheet.Cells[10, 3], worksheet.Cells[10, numberOfPairsAtLoop * 2 + 2]].Value2 = timeBetweenClearingLoopAndRestart;
                    worksheet.Range[worksheet.Cells[11, 3], worksheet.Cells[11, numberOfPairsAtLoop * 2 + 2]].Value2 = transactionTime;

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
                //isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }

            /* Save the workbook and close the excel application. */
            workbook.SaveAs(saveFilename, XlFileFormat.xlOpenXMLWorkbook, Type.Missing, Type.Missing, Type.Missing, Type.Missing,
                XlSaveAsAccessMode.xlNoChange, Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close(Type.Missing, Type.Missing, Type.Missing);
            excel.UserControl = true;
            excel.Quit();

        }
        
        /// <summary>
        /// Write the aggregated data to a file for evaluation.
        /// </summary>
        /// <param name="averageTrains">List of aggregated train journies.</param>
        /// <param name="stats">The statstics generated for each average train</param>
        /// <param name="aggregatedDestination">The destination directory for the resulting file.</param>
        public static void wrtieTrainPairStatistics(List<TrainPairStatistics> stats, string aggregatedDestination)
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
                //isFileOpen(saveFilename);
                File.Delete(saveFilename);
            }

            /* Save the excel file. */
            excel.UserControl = false;
            workbook.SaveAs(saveFilename, Microsoft.Office.Interop.Excel.XlFileFormat.xlWorkbookDefault, Type.Missing, Type.Missing,
                false, false, Microsoft.Office.Interop.Excel.XlSaveAsAccessMode.xlNoChange,
                Type.Missing, Type.Missing, Type.Missing, Type.Missing, Type.Missing);

            workbook.Close();

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
            string[] headerString = { "Wagon ID", "Origin", "Planned Destiantion", "Destination", 
                                  "Attatchment Time", "Detatchment Time", "Net Weight" };

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
            string[,] ID = new string[excelPageSize, 1];
            string[,] Orig = new string[excelPageSize, 1];
            string[,] Planned = new string[excelPageSize, 1];
            string[,] Dest = new string[excelPageSize, 1];
            DateTime[,] attatch = new DateTime[excelPageSize, 1];
            DateTime[,] detatch = new DateTime[excelPageSize, 1];
            double[,] weight = new double[excelPageSize, 1];


            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Set the active worksheet. */
                worksheet = (Worksheet)workbook.Sheets[excelPage + 1];
                workbook.Sheets[excelPage + 1].Activate();
                worksheet.get_Range("A1", "G1").Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int j = 0; j < excelPageSize; j++)
                {
                    /* Check we dont try to read more data than there really is. */
                    int checkIdx = j + excelPage * excelPageSize;
                    if (checkIdx < wagon.Count())
                    {
                        ID[j, 0] = wagon[checkIdx].wagonID;
                        Orig[j, 0] = wagon[checkIdx].origin;
                        Planned[j, 0] = wagon[checkIdx].plannedDestination;
                        Dest[j, 0] = wagon[checkIdx].destination;
                        attatch[j, 0] = wagon[checkIdx].attachmentTime;
                        detatch[j, 0] = wagon[checkIdx].detachmentTime;
                        weight[j, 0] = wagon[checkIdx].netWeight;
                    }
                    else
                    {
                        /* The end of the data has been reached. Replace teh previous values with empty data. */
                        ID[j, 0] = "";
                        Orig[j, 0] = "";
                        Planned[j, 0] = "";
                        Dest[j, 0] = "";
                        attatch[j, 0] = DateTime.MinValue;
                        detatch[j, 0] = DateTime.MinValue;
                        weight[j, 0] = 0;
                    }
                }

                /* Write the data to the active excel workseet. */
                worksheet.get_Range("A" + header, "A" + (header + excelPageSize-1)).Value2 = ID;
                worksheet.get_Range("B" + header, "B" + (header + excelPageSize - 1)).Value2 = Orig;
                worksheet.get_Range("C" + header, "C" + (header + excelPageSize - 1)).Value2 = Planned;
                worksheet.get_Range("D" + header, "D" + (header + excelPageSize - 1)).Value2 = Dest;
                worksheet.get_Range("E" + header, "E" + (header + excelPageSize - 1)).Value2 = attatch;
                worksheet.get_Range("F" + header, "F" + (header + excelPageSize - 1)).Value2 = detatch;
                worksheet.get_Range("G" + header, "G" + (header + excelPageSize - 1)).Value2 = weight;

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
            string[] headerString = { "Wagon ID", 
                                        "Origin", "Origin SA4", "Origin State", 
                                        "Via", "Via SA4", "Via State", 
                                        "Destination", "Destination SA4", "Destination State", 
                                        "Weight" };

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
            string[,] ID = new string[excelPageSize, 1];
            string[,] Orig = new string[excelPageSize, volume[0].Origin.Count()];
            string[,] Via = new string[excelPageSize, volume[0].Via.Count()];
            string[,] Dest = new string[excelPageSize, volume[0].Destination.Count()];
            double[,] weight = new double[excelPageSize, 1];

            /* Loop through the excel pages. */
            for (int excelPage = 0; excelPage < excelPages; excelPage++)
            {
                /* Set the active worksheet. */
                worksheet = (Worksheet)workbook.Sheets[excelPage + 1];
                workbook.Sheets[excelPage + 1].Activate();
                worksheet.get_Range("A1", "K1").Value2 = headerString;

                /* Loop through the data for each excel page. */
                for (int j = 0; j < excelPageSize; j++)
                {
                    /* Check we dont try to read more data than there really is. */
                    int checkIdx = j + excelPage * excelPageSize;
                    if (checkIdx < volume.Count())
                    {
                        ID[j, 0] = volume[checkIdx].wagonID;
                        for (int locationIdx = 0; locationIdx < volume[checkIdx].Origin.Count(); locationIdx++)
                        {
                            Orig[j, locationIdx] = volume[checkIdx].Origin[locationIdx];
                            Via[j, locationIdx] = volume[checkIdx].Via[locationIdx];
                            Dest[j, locationIdx] = volume[checkIdx].Destination[locationIdx];
                        }
                        weight[j, 0] = volume[checkIdx].weight;
                    }
                    else
                    {
                        /* The end of the data has been reached. Populate the remaining elements. */
                        ID[j, 0] = "";
                        for (int locationIdx = 0; locationIdx < volume[checkIdx].Origin.Count(); locationIdx++)
                        {
                            Orig[j, locationIdx] = "";
                            Via[j, locationIdx] = "";
                            Dest[j, locationIdx] = "";
                        }
                        weight[j, 0] = 0;
                    }
                }

                /* Write the data to the active excel workseet. */
                worksheet.get_Range("A" + header, "A" + (header + excelPageSize - 1)).Value2 = ID;
                worksheet.get_Range("B" + header, "D" + (header + excelPageSize - 1)).Value2 = Orig;
                worksheet.get_Range("E" + header, "G" + (header + excelPageSize - 1)).Value2 = Via;
                worksheet.get_Range("H" + header, "J" + (header + excelPageSize - 1)).Value2 = Dest;
                worksheet.get_Range("K" + header, "K" + (header + excelPageSize - 1)).Value2 = weight;

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

            return;
        }

        /// <summary>
        /// Identify the train operator from the first few letters of the field.
        /// </summary>
        /// <param name="shortOperator">The first few letters of the operator field.</param>
        /// <returns>A train operator object identifying the train operator.</returns>
        private static trainOperator getOperator(string shortOperator)
        {
            /* Compare each train operator to the supplied string to identify the correct operator. */
            if (shortOperator.Equals("Austra", StringComparison.OrdinalIgnoreCase))
                return trainOperator.ARTC;
            else if (shortOperator.Equals("ARTC  ", StringComparison.OrdinalIgnoreCase))
                return trainOperator.ARTC;
            else if (shortOperator.Equals("Aust R", StringComparison.OrdinalIgnoreCase))
                return trainOperator.AustralianRailwaysHistoricalSociety;
            else if (shortOperator.Equals("Aurizo", StringComparison.OrdinalIgnoreCase))
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
