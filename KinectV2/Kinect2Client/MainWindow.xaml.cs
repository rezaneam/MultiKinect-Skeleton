using Microsoft.Kinect;
using Microsoft.Win32;
using Microsoft.WindowsAPICodePack.Dialogs;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace Kinect2Client
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        #region socket programming variables

        public static Socket client;
        public static IPHostEntry HostInfo;
        private const int BUFFER_SIZE = 2048;
        private static readonly byte[] buffer = new byte[BUFFER_SIZE];
        public static string ReceivedText;
        public string NetworkMsg = "null";
        public SynchronizationContext uiContext;
        #endregion

        int ColorRefreshRate = 0;
        int SkeletonRefreshRate = 0;
        //StreamWriter stream_writer;
        FileStream SkeletonfileStream;
        FileStream EventfileStream;

        ushort[] DepthGrayScale = new ushort[512* 424];
        WriteableBitmap GrayScaleImage;
        //DateTime time = new DateTime();
        TimeSpan TimePeriod;
        UInt32 TimeValue = 0;
        UInt16 DateValue = 0;
        bool Write_File = false;

        byte[] TimeArray;
        byte[] DateArray;
        byte[] PosX, PosY, PosZ, BLx, BLy, OrienX, OrienY, OrienZ, OrienW;

        bool is_tracked = false;
      


        string VideoFolder;
        
        string PngFileStart;

        DoubleAnimation anim = new DoubleAnimation(50, 400, TimeSpan.FromSeconds(10), FillBehavior.HoldEnd);

        DispatcherTimer FrameRateTimer = new DispatcherTimer();
        float[,] JointsPoints = new float[25, 4]; // [Joint No , (X,Y,Stat,z)]


        #region Depth Information

        //public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource DepthImageSource
        {
            get
            {
                return this.depthBitmap;
            }
        }

        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;


        #endregion

        #region Kinect Color & Skeleton 
        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }


        public ImageSource TrackingImage
        {
            get
            {
                return this.SkeletonimageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }



        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;
        //private WriteableBitmap HDcolorBitmap = null;

        TransformedBitmap tbBitmap;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;



        ////////// Skeletin Tracking Properties 
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage SkeletonimageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        //private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int SkeletondisplayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int SkeletondisplayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        ///// <summary>
        ///// Current status text to display
        ///// </summary>
        //private string statusText = null;

        //////// Skeleton Tracking Properties

        #endregion

        #region Constructor and Deconstructor
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            #region Color Image

            //kinectSensor.CoordinateMapper.GetDepthCameraIntrinsics;
            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();
   
            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Rgba);

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
            //this.HDcolorBitmap = new WriteableBitmap(1280, 720, 96.0, 96.0, PixelFormats.Bgr32, null);
            #endregion
            ////////// Skeleton Tracking
            #region Skeleton Data
            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper; // Map Joints Poistion to coresponding position in Image

            // get the depth (display) extents
            FrameDescription SkeletonframeDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            //SkeletonframeDescription = this.kinectSensor.ColorFrameSource.FrameDescription;

            // get size of joint space
            this.SkeletondisplayWidth = SkeletonframeDescription.Width;
            this.SkeletondisplayHeight = SkeletonframeDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            ///////// Skeleton Tracking

            #endregion

            #region Depth Data
            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += Reader_DepthFrameFrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);


            #endregion


            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;


            ////////// Skeleton Tracking

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.SkeletonimageSource = new DrawingImage(this.drawingGroup);



            ////////// Skeleton Tracking



            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

            //Loaded += Window_Loaded;

            

        }

        

        /// <summary>
        /// Deconstructor
        /// </summary>
        ~MainWindow()
        {

        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            
            uiContext = SynchronizationContext.Current;

            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += Reader_BodyFrameArrived;
            }

            FrameRateTimer.Interval = TimeSpan.FromSeconds(1);

            FrameRateTimer.Tick += FrameRateTimer_Tick;

            FrameRateTimer.Start();
            
            Debug.WriteLine(DateTime.Now.ToString("'['yyyy'-'mm'-'dd']-{'HH'-'mm'-'ss'}'", CultureInfo.CurrentUICulture.DateTimeFormat));

            String title = "Kinect ID " + kinectSensor.UniqueKinectId;
            KinectIDTextBlock.Text = title;

        }

        
        private void Window_Closing(object sender, CancelEventArgs e)
        {
            //if (client.Connected)
            //{

            //    client.Shutdown(SocketShutdown.Both);
            //    client.Close();
            //    client.Dispose();
            //}
            FrameRateTimer.Stop();

            if (this.colorFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        #endregion

        #region Frame Arrived (Color - Skeleton - Depth )

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {

            ColorRefreshRate += 1;
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null )
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;


                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        
                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                            //this.HDcolorBitmap.AddDirtyRect(new Int32Rect(0, 0, 1280, 720));

                            if (EnalePointsCheckBox.IsChecked == true)
                            {

                                if (GuideLinesCheckBox.IsChecked == true)
                                {
                                    colorBitmap.FillRectangle(958, 0, 963, 1080, Colors.Red);
                                    colorBitmap.FillRectangle(0, 538, 1920, 543, Colors.Red);
                                }

                                for (int i = 0; i < 25; i++)
                                {
                                    int spot_size = 5;
                                    
                                    switch (Convert.ToInt16(JointsPoints[i, 2]))
                                    {
                                        case 1:
                                            if (colorBitmap.Width > JointsPoints[i, 0] && colorBitmap.Height > JointsPoints[i, 1] && JointsPoints[i, 0] > 0 && JointsPoints[i, 1] > 0)
                                             
                                                colorBitmap.FillEllipseCentered(Convert.ToInt32(JointsPoints[i, 0]), Convert.ToInt32(JointsPoints[i, 1]), spot_size + 2, spot_size + 2, Colors.Red);
                                            break;
                                        case 2:
                                            if (colorBitmap.Width > JointsPoints[i, 0] && colorBitmap.Height > JointsPoints[i, 1] && JointsPoints[i, 0] > 0 && JointsPoints[i, 1] > 0)
                                              
                                                colorBitmap.FillEllipseCentered(Convert.ToInt32(JointsPoints[i, 0]), Convert.ToInt32(JointsPoints[i, 1]), spot_size, spot_size, Colors.Yellow);
                                            break;

                                        default:
                                            break;
                                    }


                                }
                            }


                        }

                        
                        
                        this.colorBitmap.Unlock();

                      

                        


                    }
                }
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        //private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            SkeletonRefreshRate += 1;
            for (int i = 0; i < 25; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    JointsPoints[i, j] = 0;
                }
            }

            //BodyFrame newBoady = bodyFrameReader.BodyFrameSource.;
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }


            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.SkeletondisplayWidth, this.SkeletondisplayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {

                        //body.IsTracked

                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            is_tracked = true;
                          
                            
                                this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            if (body.LeanTrackingState != TrackingState.Tracked)
                            {
                                is_tracked = false;
                            }

                            /*
                            ----------  < Packet >
                            -------------
                            | $ | $ | $ | Flags 3 bytes
                            -------------

                            ----------------------------------------------
                            | Time | Date | # | RH | LH | LB | LBx | LBy | 18 Bytes
                            ----------------------------------------------
                            Time        > 4 Bytes (Unit 32)
                            Date        > 2 Bytes (Uint 16)
                            #Sign       > 1 Byte
                            Right Hand  > 1 Byte                    Low Confidence          Hi Confidence
                                                  Unknown               0                       8
                                                  Not Tracked           1                       9
                                                  Open                  2                       10
                                                  Closed                3                       11
                                                  Lasso                 4                       12

                            Left Hand   > 1 Byte                    Low Confidence          Hi Confidence
                                                  Unknown               0                       8
                                                  Not Tracked           1                       9
                                                  Open                  2                       10
                                                  Closed                3                       11
                                                  Lasso                 4                       12
                            
                            Lean Boady  > 1 Byte
                                                Not Tracked     0
                                                Inferred        1
                                                Tracked         2
                            
                            Lean X      > 4 Bytes (Float) 
                            Lean Y      > 4 Bytes (Float)

                                   -----------------------------------------------------------
                            25  x  | % | ST | JT | Px | Py | Pz | Y | JP | Ow | Ox | Oy | Oz | 31 Bytes
                                   -----------------------------------------------------------

                            % Sign          >   1 Byte
                            Y Sign          >   1 Byte
                            Status          >   1 Byte
                                                      Not Tracked     0
                                                      Inferred        1
                                                      Tracked         2 
                            Joint Type      >   1 Byte
                                                      1.Spine Base
                                                      2.Spin Mid
                                                      3.Neck
                                                      4.Head
                                                      5.Shoulder Left
                                                      6.Elbow Left
                                                      7.Wrist Left
                                                      8.Hand Left
                                                      9.Shoulder Right
                                                      10.Elbow Right
                                                      11.Wrist Right
                                                      12.Hand Right
                                                      13.Hip Left
                                                      14.Knee Left
                                                      15.Ankel Left
                                                      16.Foot Left
                                                      17.Hip Right
                                                      18.Knee Right
                                                      19.Ankel Right
                                                      20.Foot Right
                                                      21.Spin Shoulder
                                                      22.Hand Tip Left
                                                      23.Thumb Left
                                                      24. Hand Tip Right
                                                      25. Thumb Right
                            Position X      >   4 Bytes (Float)
                            Position Y      >   4 Bytes (Float)
                            Position Z      >   4 Bytes (Float)
                            Orientation W   >   4 Bytes (Float)
                            Orientation X   >   4 Bytes (Float)
                            Orientation Y   >   4 Bytes (Float)
                            Orientation Z   >   4 Bytes (Float)



                            -----
                            | E | Flag 1 bytes
                            -----

                            ----------  </Packet >
                            */

                            if (Write_File  && RecordSkeletonCheckBox.IsChecked == true)
                            {
                                TimeValue = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                                // Time Value = MiliSec  + { 1000 x [ Second + 60 x ( Minute + 60 x hour ) ] } > Number of Miliseconds form 00:00 am
                                TimePeriod = DateTime.Today.Subtract(new System.DateTime(2000, 1, 1, 0, 0, 0));
                                DateValue = Convert.ToUInt16(TimePeriod.TotalDays);
                                // Date Value = Days Number from 1 Jan 2000

                                TimeArray = BitConverter.GetBytes(TimeValue);
                                DateArray = BitConverter.GetBytes(DateValue);


                                SkeletonfileStream.WriteByte(36); // Flag $ Sign
                                SkeletonfileStream.WriteByte(36);   // Flag $ Sign
                                SkeletonfileStream.WriteByte(36);   // Flag $ Sign
                                SkeletonfileStream.Write(TimeArray, 0, TimeArray.Length);     // 32 bit Time : Number of Miliseconds form 00:00 am
                                SkeletonfileStream.Write(DateArray, 0, DateArray.Length);   // 16 bit Date : Day Number from 1 Jan 2000
                                                                                    //fileStream.WriteByte(35); // // Flag # Sign
                                                                                    //.Write('$');
                                                                                    //stream_writer.Write(36);
                                                                                    //stream_writer.Write(flag);
                                                                                    //stream_writer.Write(TimeValue);
                                                                                    //stream_writer.Write(DateValue);
                                                                                    //stream_writer.Write('#');

                                byte RH, LH, BL = 0;

                                // Determine Right Hand Status
                                switch (body.HandRightState)
                                {
                                    case HandState.Unknown:
                                        RH = 0;
                                        break;
                                    case HandState.NotTracked:
                                        RH = 1;
                                        break;
                                    case HandState.Open:
                                        RH = 2;
                                        break;
                                    case HandState.Closed:
                                        RH = 3;
                                        break;
                                    case HandState.Lasso:
                                        RH = 4;
                                        break;
                                    default:
                                        RH = 0;
                                        break;
                                }

                                switch (body.HandRightConfidence)
                                {
                                    case TrackingConfidence.Low:
                                        break;
                                    case TrackingConfidence.High:
                                        RH += 8;
                                        break;
                                    default:
                                        break;
                                }

                                // Determine Left Hand Status
                                switch (body.HandLeftState)
                                {
                                    case HandState.Unknown:
                                        LH = 0;
                                        break;
                                    case HandState.NotTracked:
                                        LH = 1;
                                        break;
                                    case HandState.Open:
                                        LH = 2;
                                        break;
                                    case HandState.Closed:
                                        LH = 3;
                                        break;
                                    case HandState.Lasso:
                                        LH = 4;
                                        break;
                                    default:
                                        LH = 0;
                                        break;
                                }

                                switch (body.HandLeftConfidence)
                                {
                                    case TrackingConfidence.Low:
                                        break;
                                    case TrackingConfidence.High:
                                        LH += 8;
                                        break;
                                    default:
                                        break;
                                }


                                switch (body.LeanTrackingState)
                                {
                                    case TrackingState.NotTracked:
                                        BL = 0;
                                        break;
                                    case TrackingState.Inferred:
                                        BL = 1;
                                        break;
                                    case TrackingState.Tracked:
                                        BL = 2;
                                        break;
                                    default:
                                        break;
                                }

                                BLx = BitConverter.GetBytes(body.Lean.X);
                                BLy = BitConverter.GetBytes(body.Lean.Y);

                                SkeletonfileStream.WriteByte(RH); // Right Hand Status
                                SkeletonfileStream.WriteByte(LH); // Left Hand Status
                                SkeletonfileStream.WriteByte(BL); // Body Lean Status
                                SkeletonfileStream.Write(BLx, 0, BLx.Length); // Lean Position X
                                SkeletonfileStream.Write(BLy, 0, BLy.Length); // Lean Position Y

                            }


                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            byte i = 0;
                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                ColorSpacePoint colorSpacePoint = coordinateMapper.MapCameraPointToColorSpace(position);

                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                               

                                JointsPoints[i, 0] = colorSpacePoint.X;
                                JointsPoints[i, 1] = colorSpacePoint.Y;
                                JointsPoints[i, 3] = position.Z;
                                switch (joints[jointType].TrackingState)
                                {
                                    case TrackingState.NotTracked:
                                        JointsPoints[i, 2] = 0;
                                        break;
                                    case TrackingState.Inferred:
                                        JointsPoints[i, 2] = 1;
                                        break;
                                    case TrackingState.Tracked:
                                        JointsPoints[i, 2] = 2;
                                        break;
                                    default:
                                        break;
                                }
                                i++;

                                if (joints[jointType].TrackingState != TrackingState.Tracked)
                                {
                                    is_tracked = false;
                                }

                                if (Write_File && RecordSkeletonCheckBox.IsChecked == true)
                                {

                                    SkeletonfileStream.WriteByte(37); // flag % Sign
                                    SkeletonfileStream.WriteByte(Joint_No(jointType));  // Joint Type
                                    //Convert.ToByte(jointType)
                                    //stream_writer.Write(Convert.ToChar(i));
                                    PosX = BitConverter.GetBytes(position.X);
                                    PosY = BitConverter.GetBytes(position.Y);
                                    PosZ = BitConverter.GetBytes(position.Z);

                                    SkeletonfileStream.Write(PosX, 0, PosX.Length);
                                    SkeletonfileStream.Write(PosY, 0, PosY.Length);
                                    SkeletonfileStream.Write(PosZ, 0, PosZ.Length);

                                    switch (joints[jointType].TrackingState)
                                    {
                                        case TrackingState.NotTracked:
                                            SkeletonfileStream.WriteByte(0);
                                            break;
                                        case TrackingState.Inferred:
                                            SkeletonfileStream.WriteByte(1);
                                            break;
                                        case TrackingState.Tracked:
                                            SkeletonfileStream.WriteByte(2);
                                            break;
                                        default:
                                            break;
                                    }
                                  
                                }

                                
                            }

                            if (is_tracked & EnalePointsCheckBox.IsEnabled)
                            {
                                float[] Joint1 = new float[3];
                                float[] Joint2 = new float[3];
                                float[] Joint3 = new float[3];

                                Joint1[0] = joints[JointType.AnkleLeft].Position.X;
                                Joint1[1] = joints[JointType.AnkleLeft].Position.Y;
                                Joint1[2] = joints[JointType.AnkleLeft].Position.Z;

                                Joint2[0] = joints[JointType.KneeLeft].Position.X;
                                Joint2[1] = joints[JointType.KneeLeft].Position.Y;
                                Joint2[2] = joints[JointType.KneeLeft].Position.Z;

                                Joint3[0] = joints[JointType.HipLeft].Position.X;
                                Joint3[1] = joints[JointType.HipLeft].Position.Y;
                                Joint3[2] = joints[JointType.HipLeft].Position.Z;

                               

                                Joint1[0] = joints[JointType.AnkleRight].Position.X;
                                Joint1[1] = joints[JointType.AnkleRight].Position.Y;
                                Joint1[2] = joints[JointType.AnkleRight].Position.Z;

                                Joint2[0] = joints[JointType.KneeRight].Position.X;
                                Joint2[1] = joints[JointType.KneeRight].Position.Y;
                                Joint2[2] = joints[JointType.KneeRight].Position.Z;

                                Joint3[0] = joints[JointType.HipRight].Position.X;
                                Joint3[1] = joints[JointType.HipRight].Position.Y;
                                Joint3[2] = joints[JointType.HipRight].Position.Z;

                                
                            }

                            //i = 0;
                            foreach (var orientation in body.JointOrientations)
                            {



                                if (Write_File && RecordSkeletonCheckBox.IsChecked == true)
                                {
                                    OrienW = BitConverter.GetBytes(orientation.Value.Orientation.W);
                                    OrienX = BitConverter.GetBytes(orientation.Value.Orientation.X);
                                    OrienY = BitConverter.GetBytes(orientation.Value.Orientation.Y);
                                    OrienZ = BitConverter.GetBytes(orientation.Value.Orientation.Z);

                                    SkeletonfileStream.WriteByte(165); // flag Special Sign
                                    SkeletonfileStream.WriteByte(Joint_No(orientation.Key));  // Joint Type
                                    SkeletonfileStream.Write(OrienW, 0, OrienW.Length);
                                    SkeletonfileStream.Write(OrienX, 0, OrienX.Length);
                                    SkeletonfileStream.Write(OrienY, 0, OrienY.Length);
                                    SkeletonfileStream.Write(OrienZ, 0, OrienZ.Length);
                                }
                            }

                            if (Write_File && RecordSkeletonCheckBox.IsChecked == true)
                            {
                                //flag = 'E';
                                //stream_writer.Write(flag);
                                //stream_writer.Write(flag);
                                SkeletonfileStream.WriteByte(69);
                                SkeletonfileStream.WriteByte(69);
                            }

                            
                                this.DrawBody(joints, jointPoints, dc, drawPen);

                                this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                                this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                            

                        }


                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.SkeletondisplayWidth, this.SkeletondisplayHeight));
                }

               

            }


        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_DepthFrameFrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //// maxDepth = depthFrame.DepthMaxReliableDistance

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);

                            depthFrameProcessed = true;
                        }
                    }
                }
            }



            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();

               
            }
        }

        #endregion

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void FrameRateTimer_Tick(object sender, EventArgs e)
        {
            //throw new NotImplementedException();

            ColorFrameNo.Text = "Color Image Frame Rate : " + ColorRefreshRate.ToString() + " fps";
            TrackingFrameNo.Text = "Skeleton Image Frame Rate : " + SkeletonRefreshRate.ToString() + " fps";

            //Debug.WriteLine(ColorFrameNo.Text);
            ColorRefreshRate = 0;
            SkeletonRefreshRate = 0;
        }


        #region Button  and Controls

        

        private void EnableRGBCheckBox_DataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
          
        }

        
        private void EnableRemoteCheckBox_DataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            if (EnableRemoteCheckBox.IsChecked == true)
            {
                NetworkStackPanel.Visibility = Visibility.Visible;
                MannualRecordStackPanel.Visibility = Visibility.Collapsed;
            }
            else
            {
                NetworkStackPanel.Visibility = Visibility.Collapsed;
                MannualRecordStackPanel.Visibility = Visibility.Visible;
            }

        }


        #endregion

        #region socket programming functions

        void NetworkStatUpdate(string text, int erorrCode)
        {
            NetworkStatusTextBlock.Text = text;

            if (erorrCode > 0)
            {
                ConnectButton.IsEnabled = true;
                PortNoTextBox.IsEnabled = true;
                IPAddTextBox.IsEnabled = true;
            }
            else
            {
                ConnectButton.IsEnabled = false;
                PortNoTextBox.IsEnabled = false;
                IPAddTextBox.IsEnabled = false;

                if (text == "Connected.")
                {
                    EnableRemoteCheckBox.IsEnabled = false;
                  


                }
            }
        }

        private void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
           
            RecordSkeletonCheckBox.IsEnabled = false;
            HostInfo = Dns.GetHostEntry(Dns.GetHostName());

            IPAddress newAdd = IPAddress.Parse(IPAddTextBox.Text);
            IPEndPoint ipEndPoint = new IPEndPoint(newAdd, Convert.ToInt16(PortNoTextBox.Text));
            client = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            client.NoDelay = true;
            NetworkStatUpdate("Please wait ...", 0);
            client.BeginConnect(ipEndPoint, ConnectCallBack, client);
        }

        private void EnableRemoteCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            NetworkStackPanel.Visibility = Visibility.Visible;
            MannualRecordStackPanel.Visibility = Visibility.Collapsed;
        }

        private void EnableRemoteCheckBox_Unchecked(object sender, RoutedEventArgs e)
        {
            NetworkStackPanel.Visibility = Visibility.Collapsed;
            MannualRecordStackPanel.Visibility = Visibility.Visible;
        }

        private void ConnectCallBack(IAsyncResult ar)
        {
            try
            {
                client.EndConnect(ar);
                client.BeginReceive(buffer, 0, BUFFER_SIZE, SocketFlags.None, ReceiveCallback, client);
            }
            catch (SocketException ecp)
            {

                uiContext.Send(x => NetworkStatUpdate(ecp.Message, ecp.ErrorCode), null);
                Debug.WriteLine(ecp.Data);
                Debug.WriteLine(ecp.ErrorCode);
                Debug.WriteLine(ecp.HResult);
                Debug.WriteLine(ecp.SocketErrorCode);
                Debug.WriteLine(ecp.NativeErrorCode);
                Debug.WriteLine(ecp.StackTrace);
            }



        }

        private void ReceiveCallback(IAsyncResult AR)
        {
            Socket current = (Socket)AR.AsyncState;
            int received;
            try
            {
                received = current.EndReceive(AR);
            }
            catch (SocketException ecp)
            {

                Debug.WriteLine("Server forcefully disconnected");
                Debug.WriteLine(ecp.Message);

                // Don't shutdown because the socket may be disposed and its disconnected anyway.
                current.Close();
                uiContext.Send(x => NetworkStatUpdate(ecp.Message, ecp.ErrorCode), null);


                return;
            }





            byte[] data;
            byte[] recBuf = new byte[received];
            Array.Copy(buffer, recBuf, received);
            ReceivedText = Encoding.ASCII.GetString(recBuf);
            Debug.WriteLine("Received Text: " + ReceivedText);
            //StatusTextBlock.Text = ReceivedText;
            if (ReceivedText.ToLower() == "name")
            {
                data = Encoding.ASCII.GetBytes(HostInfo.HostName);
                current.Send(data);
            }
            else if (ReceivedText.ToLower() == "type")
            {
                data = Encoding.ASCII.GetBytes("Kinect 2");
                current.Send(data);
            }
            else if (ReceivedText.ToLower() == "connected")
            {
                uiContext.Send(x => NetworkStatUpdate("Connected.", 0), null);

            }
            else if (ReceivedText.ToLower() == "init")
            {
                uiContext.Send(x => RemoteInit(), null);
            }
            else if (ReceivedText.ToLower() == "info")
            {
                current.Send(Encoding.ASCII.GetBytes(this.kinectSensor.UniqueKinectId));
            }
            else if (ReceivedText.ToLower() == "start" && SkeletonfileStream != null)
            {

                uiContext.Send(x => RemoteStart(), null);
            }
            else if (ReceivedText.ToLower() == "pause")
            {
                uiContext.Send(x => RemotePause(), null);
            }
            else if (ReceivedText.ToLower() == "resume")
            {
                uiContext.Send(x => RemoteResume(), null);
            }
            else if (ReceivedText.Contains("SubjectID"))
            {
                uiContext.Send(x => CreateFile(ReceivedText), null);

            }
            else if (ReceivedText.ToLower() == "stop")
            {
                uiContext.Send(x => RemoteStop(), null);
            }
            else if (ReceivedText.ToLower() == "time")
            {


                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                // Time Value = MiliSec  + { 1000 x [ Second + 60 x ( Minute + 60 x hour ) ] } > Number of Miliseconds form 00:00 am
                TimeSpan SysDatePeriod = DateTime.Today.Subtract(new System.DateTime(2000, 1, 1, 0, 0, 0));
                UInt16 SysDate = Convert.ToUInt16(SysDatePeriod.TotalDays);
                // Date Value = Days Number from 1 Jan 2000

                byte[] SysTimeArray = BitConverter.GetBytes(SysTime);
                byte[] SysDateArray = BitConverter.GetBytes(SysDate);
                byte[] SendTimeArray = new byte[6];
                for (int i = 0; i < 4; i++)
                    SendTimeArray[i] = SysTimeArray[i];
                for (int i = 0; i < 2; i++)
                    SendTimeArray[i + 4] = SysDateArray[i];
                current.Send(SendTimeArray);


            }


            current.BeginReceive(buffer, 0, BUFFER_SIZE, SocketFlags.None, ReceiveCallback, current);
        }

        void CreateFile(string filename)
        {
            try
            {


                filename = filename.Remove(0, 9);
                string path = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
                EventfileStream  = File.Create(path + "\\" + filename + "-Kinect2-" + HostInfo.HostName + ".csv");

                WriteTextFile(EventfileStream, string.Format("Sensor ID , {0}\r\n", this.kinectSensor.UniqueKinectId));


                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "File Create"
                    ,DateTime.Now, SysTime));

                if (RecordSkeletonCheckBox.IsChecked == true)
                    SkeletonfileStream = File.Create(path + "\\" + filename + "-" + HostInfo.HostName + ".ks2");
                //if (RecordDepthCheckBox.IsChecked == true)
                //    DepthfileStream = File.Create(path + "\\" + filename + "-" + HostInfo.HostName + ".kp2");
                //statusBar.Text = filename + " File Created.";
                VideoFolder = path + "\\" + filename + "-Kinect2-" + HostInfo.HostName;
                

                
                
                PngFileStart = "Kinect2-" + DateTime.Now.ToString("'['yyyy'-'mm'-'dd']-{'HH'-'mm'-'ss'}-'", CultureInfo.CurrentUICulture.DateTimeFormat);
               
                NetworkStatusTextBlock.Text = "Remote System Initialized";

                SendTextSocket("OK");
                NetworkStatusTextBlock.Text = "System Initialized";
            }
            catch
            {

                SendTextSocket("Failed");
                return;
            }
        }

        void RemoteStart()
        {
            if (SkeletonfileStream != null)
                SendTextSocket("OK");
            else
                SendTextSocket("Failed");
            //SaveFileButton_Click(null, null);

            Write_File = true;
          
            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Start"
                , DateTime.Now, SysTime));

            NetworkStatUpdate("Recording.", 0);

        }

        void RemoteStop()
        {
            if (SkeletonfileStream != null && Write_File == true)
                SendTextSocket("OK");
            else
                SendTextSocket("Failed");
            Write_File = false;
          

            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Stop"
                , DateTime.Now, SysTime));

            uiContext.Send(x => NetworkStatUpdate("Record Finished", 0), null);

            SkeletonfileStream.Close();
            EventfileStream.Close();
        }

        void RemotePause()
        {
            if (SkeletonfileStream != null && Write_File == true)
                SendTextSocket("OK");
            else
                SendTextSocket("Failed");
            Write_File = false;
       

            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Pause"
                , DateTime.Now, SysTime));

            uiContext.Send(x => NetworkStatUpdate("Record Paused", 0), null);
        }

        void RemoteResume()
        {
            if (SkeletonfileStream != null && Write_File == false)
                SendTextSocket("OK");
            else
                SendTextSocket("Failed");

            Write_File = true;


            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Resume"
                , DateTime.Now, SysTime));

            NetworkStatUpdate("Recording.", 0);
        }

        private void RemoteInit()
        {



            SaveFileDialog dlg = new Microsoft.Win32.SaveFileDialog();
            dlg.FileName = "Kinect 2 Joint File"; // Default file name
            dlg.DefaultExt = ".bin"; // Default file extension
            dlg.Filter = "Skeleton Binary File (.bin)|*.bin"; // Filter files by extension


            // Show save file dialog box
            Nullable<bool> result = dlg.ShowDialog();

            // Process save file dialog box results
            if (result == true)
            {
                // Save document

                SkeletonfileStream = File.Create(dlg.FileName);

                


                CommonOpenFileDialog dialog = new CommonOpenFileDialog();
                dialog.IsFolderPicker = true;
                CommonFileDialogResult dlgresult = dialog.ShowDialog();

                if (dlgresult == CommonFileDialogResult.Ok)
                {
                    VideoFolder = dialog.FileName;
                    
                    PngFileStart = "Kinect-" + DateTime.Now.ToString("'['yyyy'-'mm'-'dd']-{'HH'-'mm'-'ss'}-'", CultureInfo.CurrentUICulture.DateTimeFormat);
                    
                    NetworkStatusTextBlock.Text = "Remote System Initialized";




                }
                else
                    SendTextSocket("Failed"); // Folder doesn't created successfully
            }
            else
                SendTextSocket("Failed"); // Bin file creation failed

            SendTextSocket("OK");
        }

        void SendTextSocket(string text)
        {
            byte[] data = Encoding.ASCII.GetBytes(text);

            client.Send(data);
        }

        public void WriteTextFile(FileStream file, string text)
        {
            byte[] data = Encoding.ASCII.GetBytes(text);
            //await file.WriteAsync(data, 0, data.Length);
            file.WriteAsync(data, 0, data.Length);
        }

        private void GenerateFileButton_Click(object sender, RoutedEventArgs e)
        {
            string[] illegalChars = new string[9] { "<", ">", ":", "/", @"\", "|", "?", "\"", "*" };
            bool wrongfilename = false;
            if (FilenameTextBox.Text != string.Empty)
            {
                for (int i = 0; i < illegalChars.Length; i++)
                {
                    if (FilenameTextBox.Text.Contains(illegalChars[i]))
                    {
                        wrongfilename = true;

                    }
                }
                if (!wrongfilename)
                {
                    CreateFileMannual(FilenameTextBox.Text);
                    MannualRecordButtonsStackPanel.Visibility = Visibility.Visible;
                    FileInitiationStackPanel.Visibility = Visibility.Collapsed;
                    EnableRemoteCheckBox.IsEnabled = false;
                    GenerateFileButton.IsEnabled = false;
                    StartRecordButton.IsEnabled = true;
                }
                else
                {
                    string messageBoxText = "Reserved character(s) is in the file name! \r You are not allowed to use  < , > ,  / , \\ , | , ?, * , \" characters in the filename";
                    string caption = "Error!";
                    MessageBoxButton button = MessageBoxButton.OK;
                    MessageBoxImage icon = MessageBoxImage.Error;
                    MessageBox.Show(messageBoxText, caption, button, icon);
                }

            }
        }



        #endregion

        #region Mannual Record

        bool CreateFileMannual(string filename)
        {
            try
            {



                string path = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
                SkeletonfileStream = File.Create(path + "\\" + filename + ".ks2");
                EventfileStream = File.Create(path + "\\" + filename + "-Kinect2" + ".csv");
                MannualStatusTextBlock.Text = filename + ".ks2" + " File Created.";



                WriteTextFile(EventfileStream, string.Format("Sensor ID , {0}\r\n", this.kinectSensor.UniqueKinectId));

                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "File Create"
                    , DateTime.Now, SysTime));

                return true;

            }
            catch
            {


                return false;
            }
        }

        bool StartFile()
        {
            try
            {
                if (SkeletonfileStream != null)
                {

                    Write_File = true;
                    MannualStatusTextBlock.Text = "Saving Data ...";

                    UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                    WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Started"
                        , DateTime.Now, SysTime));

                    PauseRecordButton.IsEnabled = true;
                    StopRecordButton.IsEnabled = true;
                    ResumeRecordButton.IsEnabled = false;
                    StartRecordButton.IsEnabled = false;

                    return true;

                }
                else
                    return false;
            }
            catch (Exception)
            {

                return false;
            }


        }

        bool ResumeFile()
        {
            if (SkeletonfileStream != null && Write_File == false)
            {
                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Resume"
                    , DateTime.Now, SysTime));
                Write_File = true;
                MannualStatusTextBlock.Text = "Saving Data ...";

                PauseRecordButton.IsEnabled = true;
                StopRecordButton.IsEnabled = true;
                ResumeRecordButton.IsEnabled = false;
                StartRecordButton.IsEnabled = false;
                return true;
            }

            return false;
        }

        bool PauseFile()
        {
            if (SkeletonfileStream != null && Write_File == true)
            {
                Write_File = false;

                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Pause"
                    , DateTime.Now, SysTime));

                MannualStatusTextBlock.Text = "Record Paused.";

                PauseRecordButton.IsEnabled = false;
                StopRecordButton.IsEnabled = true;
                ResumeRecordButton.IsEnabled = true;
                StartRecordButton.IsEnabled = false;

                return true;

            }
            return false;
        }

        bool StopFile()
        {
            if (SkeletonfileStream != null)
            {
                MannualStatusTextBlock.Text = "Record Finished";
                Write_File = false;


                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Stop"
                    , DateTime.Now, SysTime));

                SkeletonfileStream.Close();
                EventfileStream.Close();
                PauseRecordButton.IsEnabled = false;
                StopRecordButton.IsEnabled = false;
                ResumeRecordButton.IsEnabled = false;
                StartRecordButton.IsEnabled = true;
                GenerateFileButton.IsEnabled = true;
                FileInitiationStackPanel.Visibility = Visibility.Visible;
                MannualRecordButtonsStackPanel.Visibility = Visibility.Collapsed;

                return true;
            }


            return false;

        }

        private void StartRecordButton_Click(object sender, RoutedEventArgs e)
        {
            StartFile();
        }

        private void PauseRecordButton_Click(object sender, RoutedEventArgs e)
        {
            PauseFile();
        }

        private void ResumeRecordButton_Click(object sender, RoutedEventArgs e)
        {
            ResumeFile();
        }

        private void StopRecordButton_Click(object sender, RoutedEventArgs e)
        {
            StopFile();
        }

        #endregion

        #region Draw Skeleton and Joint Information
        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.SkeletondisplayHeight - ClipBoundsThickness, this.SkeletondisplayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.SkeletondisplayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.SkeletondisplayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.SkeletondisplayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.SkeletondisplayHeight));
            }
        }

        

        

        byte Joint_No(JointType JointName)
        {
            byte return_val;

            switch (JointName)
            {
                case JointType.SpineBase:
                    return_val = 1;
                    break;
                case JointType.SpineMid:
                    return_val = 2;
                    break;
                case JointType.Neck:
                    return_val = 3;
                    break;
                case JointType.Head:
                    return_val = 4;
                    break;
                case JointType.ShoulderLeft:
                    return_val = 5;
                    break;
                case JointType.ElbowLeft:
                    return_val = 6;
                    break;
                case JointType.WristLeft:
                    return_val = 7;
                    break;
                case JointType.HandLeft:
                    return_val = 8;
                    break;
                case JointType.ShoulderRight:
                    return_val = 9;
                    break;
                case JointType.ElbowRight:
                    return_val = 10;
                    break;
                case JointType.WristRight:
                    return_val = 11;
                    break;
                case JointType.HandRight:
                    return_val = 12;
                    break;
                case JointType.HipLeft:
                    return_val = 13;
                    break;
                case JointType.KneeLeft:
                    return_val = 14;
                    break;
                case JointType.AnkleLeft:
                    return_val = 15;
                    break;
                case JointType.FootLeft:
                    return_val = 16;
                    break;
                case JointType.HipRight:
                    return_val = 17;
                    break;
                case JointType.KneeRight:
                    return_val = 18;
                    break;
                case JointType.AnkleRight:
                    return_val = 19;
                    break;
                case JointType.FootRight:
                    return_val = 20;
                    break;
                case JointType.SpineShoulder:
                    return_val = 21;
                    break;
                case JointType.HandTipLeft:
                    return_val = 22;
                    break;
                case JointType.ThumbLeft:
                    return_val = 23;
                    break;
                case JointType.HandTipRight:
                    return_val = 24;
                    break;
                case JointType.ThumbRight:
                    return_val = 25;
                    break;
                default:
                    return_val = 0;
                    break;
            }

            return return_val;
        }

        

        #endregion

        #region Depth Data Processing
        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];


                DepthGrayScale[i] = depth;
                
                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
                //this.DepthGrayScale[i] = (ushort)(depth >= minDepth && depth <= maxDepth ? (depth) : 0); ;
            }

            
            CreateGrayScaleBitmap(DepthGrayScale);
        }

        void CreateGrayScaleBitmap(ushort[] pixels)
        {
            var width = 512;
            var height = 424;

            GrayScaleImage = new WriteableBitmap(width, height, 96, 96, PixelFormats.Gray16, null);
            GrayScaleImage.WritePixels(
                new Int32Rect(0, 0, width, height),
                pixels, width * 2 
                , 0);



            
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }
        #endregion



    }

    
}
