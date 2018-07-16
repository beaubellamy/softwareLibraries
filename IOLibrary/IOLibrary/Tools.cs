using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows.Forms;



namespace IOLibrary
{
    public class Tools
    {

        public Tools()
        { }

        /// <summary>
        /// Function allows for the selection of the destination folder where the 
        /// aggregate file will be saved to.
        /// </summary>
        /// <returns>The destination path.</returns>
        public static string selectFolder(string path = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations\Train Performance Analysis")
        {
            /* Create the folder browser and set the intial starting location. */
            FolderBrowserDialog folder = new FolderBrowserDialog();
            folder.SelectedPath = path;

            DialogResult result = folder.ShowDialog();

            if (result == DialogResult.OK && !string.IsNullOrWhiteSpace(folder.SelectedPath))
                return folder.SelectedPath;
            else
                return "";
        }

        /// <summary>
        /// A wrapper function to contain the try catch block for selecting a file using the browser.
        /// </summary>
        /// <param name="browseTitle">The title of the browser window.</param>
        /// <returns>The full path of the file selected.</returns>
        public static string browseFile(string browseTitle)
        {
            string filename = null;
            try
            {
                /* Open the browser and retrieve the file. */
                filename = selectDataFile(browseTitle);
                if (filename == null)
                    return "";
            }
            catch (Exception e)
            {
                return e.Message;
            }
            return filename;
        }

        /// <summary>
        /// Function opens a dialog box to browse and select the data file.
        /// </summary>
        /// <param name="initialDirectory">The default directory to start the browser from.</param>
        /// <returns>The full filename of the data file.</returns>
        public static string selectDataFile(string initialDirectory = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations")
        {

            /* Declare the filename to return. */
            string filename = null;

            /* Create a fileDialog browser to select the data file. */
            OpenFileDialog fileSelectBrowser = new OpenFileDialog();
            /* Set the browser properties. */
            fileSelectBrowser.Title = "Select Volume data file";
            fileSelectBrowser.InitialDirectory = initialDirectory;
            fileSelectBrowser.Filter = "All EXCEL FILES (*.xlsx*)|*.xlsx*|All files (*.*)|*.*";
            fileSelectBrowser.FilterIndex = 2;
            fileSelectBrowser.RestoreDirectory = true;
            try
            {
                /* Open the broser and select a file. */
                if (fileSelectBrowser.ShowDialog() == DialogResult.OK)
                {
                    filename = fileSelectBrowser.FileName;
                }
                else
                    return filename;

            }
            catch
            {
                /* If there was a problem with the file, show an error  */
                messageBox("Could not Open data file: ", "Failed to open data file.");
                throw;
            }
            return filename;
        }
        
        /// <summary>
        /// Function opens a dialog box to browse and select the data file.
        /// </summary>
        /// <param name="caption">A caption for the browser.</param>
        /// <param name="initialDirectory">The default directory to start the browser from.</param>
        /// <returns>The full filename of the data file.</returns>
        public static string selectDataFile(string caption, string initialDirectory = @"S:\Corporate Strategy\Infrastructure Strategies\Simulations")
        {

            /* Declare the filename to return. */
            string filename = null;

            /* Create a fileDialog browser to select the data file. */
            OpenFileDialog fileSelectBrowser = new OpenFileDialog();
            /* Set the browser properties. */
            fileSelectBrowser.Title = caption;
            fileSelectBrowser.InitialDirectory = initialDirectory;
            fileSelectBrowser.Filter = "All EXCEL FILES (*.xlsx*)|*.xlsx*|All files (*.*)|*.*";
            fileSelectBrowser.FilterIndex = 2;
            fileSelectBrowser.RestoreDirectory = true;
            try
            {
                /* Open the broser and select a file. */
                if (fileSelectBrowser.ShowDialog() == DialogResult.OK)
                {
                    filename = fileSelectBrowser.FileName;
                }
                else
                    return filename;

            }
            catch
            {
                /* If there was a problem with the file, show an error  */
                messageBox("Could not Open data file: ", "Failed to open data file.");
                throw;
            }
            return filename;
        }

        /// <summary>
        /// Determine if a file is already open before trying to read the file.
        /// </summary>
        /// <param name="filename">Filename of the file to be opened</param>
        /// <returns>True if the file is already open.</returns>
        public static void isFileOpen(string filename)
        {
            //Tools tool = new Tools();
            FileStream stream = null;

            /* Can the file be opened and read. */
            try
            {
                stream = System.IO.File.Open(filename, FileMode.Open, FileAccess.Read);
            }
            catch (IOException e)
            {
                /* File is already opended and locked for reading. */
                messageBox(e.Message + ":\n\nClose the file and Press OK.");
                //Environment.Exit(0);
            }
            finally
            {
                if (stream != null)
                    stream.Close();
            }

        }

        /// <summary>
        /// Validate the format of the data file.
        /// </summary>
        /// <param name="fields">All data fields for validation.</param>
        /// <returns>True if all fields are valid.</returns>
        public static bool validateFileFormat(string[] fields)
        {
            /* Initialise the validation parameters. */
            bool validWagonNumber = false;
            bool validTrainDate = false;
            bool validComodity = false;
            bool validOriginCode = false;
            bool validPlannedDestinationCode = false;
            bool validActualDestinationCode = false;
            bool validAttachmentTime = false;
            bool validDetachmentTime = false;
            bool validTareWeight = false;
            bool validGrossWeight = false;
            bool validDistanceTravelled = false;
            bool validWagonMovementCount = false;

            /* Check the number of fields */
            if (fields.Count() != 16)
            {
                messageBox("Incorrect number of fields detected in text file.", "Invalid number of fields.");
                return false;
            }
            else
            {
                /* Clean the Fields from escape characters. */
                char[] newDelimeters = { '\'', '"' };

                /* Check each field and validate. */
                double result = 0;

                /* Wagon class can be a 4 character string or upto 4 numbers. */
                /* This is covered during data extraction. */

                /* Validate Wagon Number */
                result = 0;
                if (double.TryParse(fields[7], out result))
                    validWagonNumber = true;
                
                /* Validate Train Date */
                DateTime dateResult = DateTime.MinValue;
                if (DateTime.TryParse(fields[9], out dateResult))
                {
                    if (dateResult == DateTime.MinValue)
                        validTrainDate = false;
                    else
                        validTrainDate = true;
                }
                

                /* Validate Commodity */
                if (fields[4].Count() == 4)
                    validComodity = true;
                
                /* Validate Origin code */
                if (fields[0].Count() == 3)
                    validOriginCode = true;
                
                /* Validate Planned Destination code */
                if (fields[1].Count() == 3)
                    validPlannedDestinationCode = true;
                
                /* Validate Destination code */
                if (fields[5].Count() == 3)
                    validActualDestinationCode = true;
                
                /* Validate Attachment Time. */
                dateResult = DateTime.MinValue;
                if (DateTime.TryParse(fields[2], out dateResult))
                {
                    if (dateResult == DateTime.MinValue)
                        validAttachmentTime = false;
                    else
                        validAttachmentTime = true;
                }
                
                /* Validate Dettachment Time */
                dateResult = DateTime.MinValue;
                if (DateTime.TryParse(fields[6], out dateResult))
                {
                    if (dateResult == DateTime.MinValue)
                        validDetachmentTime = false;
                    else
                        validDetachmentTime = true;
                }
                

                /* Validate Tare Weight */
                result = 0;
                if (double.TryParse(fields[15], out result))
                    validTareWeight = true;
                
                /* Validate Gross Weight */
                result = 0;
                if (double.TryParse(fields[12], out result))
                    validGrossWeight = true;
                
                /* Validate Distance Travelled. */
                result = 0;
                if (double.TryParse(fields[11], out result))
                    validDistanceTravelled = true;
                
                /* Validate Wagon Movement Count */
                result = 0;
                if (double.TryParse(fields[13], out result))
                    validWagonMovementCount = true;
                
            }

            /* If all the fields are validated, the file format is valid.  */
            if (validWagonNumber && validTrainDate && validComodity && validOriginCode &&
                validPlannedDestinationCode && validActualDestinationCode && validAttachmentTime &&
                validDetachmentTime && validTareWeight && validGrossWeight && validDistanceTravelled &&
                validWagonMovementCount)
                return true;
            else
                return false;

        }

        public static bool validateAzureFileFormat(string[] fields)
        {
            /* Initialise the validation parameters. */
            bool validWagonNumber = false;
            bool validTrainDate = false;
            bool validComodity = false;
            bool validOriginCode = false;
            bool validPlannedDestinationCode = false;
            bool validActualDestinationCode = false;
            bool validAttachmentTime = false;
            bool validDetachmentTime = false;
            bool validTareWeight = false;
            bool validGrossWeight = false;
            
            /* Check the number of fields */
            if (fields.Count() != 17)
            {
                messageBox("Incorrect number of fields detected in text file.", "Invalid number of fields.");
                return false;
            }
            else
            {                
                /* Clean the Fields from escape characters. */
                char[] newDelimeters = { '\'', '"' };

                /* Check each field and validate. */
                double result = 0;

                /* Wagon class can be a 4 character string or upto 4 numbers. */
                /* This is covered during data extraction. */

                /* Validate Wagon Number */
                result = 0;
                if (double.TryParse(fields[11], out result))
                    validWagonNumber = true;

                /* Validate Train Date */
                DateTime dateResult = DateTime.MinValue;
                if (DateTime.TryParse(fields[8], out dateResult))
                {
                    if (dateResult == DateTime.MinValue)
                        validTrainDate = false;
                    else
                        validTrainDate = true;
                }

                /* Validate Commodity */
                if (fields[6].Count() > 3)
                    validComodity = true;

                /* Validate Origin code */
                if (fields[4].Count() == 3)
                    validOriginCode = true;

                /* Validate Planned Destination code */
                if (fields[5].Count() == 3)
                    validPlannedDestinationCode = true;

                /* Validate Destination code */
                if (fields[1].Equals("-1") || fields[1].Count() == 3)
                    validActualDestinationCode = true;

                /* Validate Attachment Time. */
                dateResult = DateTime.MinValue;
                if (DateTime.TryParse(fields[0], out dateResult))
                {
                    if (dateResult == DateTime.MinValue)
                        validAttachmentTime = false;
                    else
                        validAttachmentTime = true;
                }

               

                /* Validate Dettachment Time */
                dateResult = DateTime.MinValue;
                if (DateTime.TryParse(fields[2], out dateResult))
                {
                    if (dateResult == DateTime.MinValue)
                        validDetachmentTime = false;
                    else
                        validDetachmentTime = true;
                }
                else if (fields[2].Equals(""))
                {
                    dateResult = DateTime.Now;
                    validDetachmentTime = true;
                }

                /* Validate Tare Weight */
                result = 0;
                if (double.TryParse(fields[16], out result))
                    validTareWeight = true;

                /* Validate Gross Weight */
                result = 0;
                if (double.TryParse(fields[13], out result))
                    validGrossWeight = true;

            }

            /* If all the fields are validated, the file format is valid.  */
            if (validWagonNumber && validTrainDate && validComodity && validOriginCode &&
                validPlannedDestinationCode && validActualDestinationCode && validAttachmentTime &&
                validDetachmentTime && validTareWeight && validGrossWeight)
                return true;
            else
                return false;

        }

        /// <summary>
        /// Display a message box with details about an error or information about some properties.
        /// </summary>
        /// <param name="message">Message to display.</param>
        public static void messageBox(string message)
        {
            MessageBox.Show(message, "Information", MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
        }

        /// <summary>
        /// Display a message box with details about an error or information about some properties.
        /// </summary>
        /// <param name="message">Message to display.</param>
        /// <param name="caption">Caption for the message box.</param>
        public static void messageBox(string message, string caption)
        {
            MessageBox.Show(message, caption, MessageBoxButtons.OK, MessageBoxIcon.Exclamation);
        }


    }
}
