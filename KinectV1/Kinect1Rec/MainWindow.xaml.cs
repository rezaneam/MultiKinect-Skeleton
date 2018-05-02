using Microsoft.Kinect;
using Microsoft.Win32;
using Microsoft.WindowsAPICodePack.Dialogs;
using System;
using System.Collections.Generic;
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
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace Kinect1Rec
{
    
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
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

        #region Recording Variables
        float[,] JointsPoints = new float[25, 4];
        DispatcherTimer FrameRateTimer = new DispatcherTimer();

        //StreamWriter stream_writer;
        FileStream fileStream;
        FileStream EventfileStream;

        int ColorRefreshRate = 0;
        int SkeletonRefreshRate = 0;
        //DateTime time = new DateTime();
        TimeSpan TimePeriod;
        UInt32 TimeValue = 0;
        UInt16 DateValue = 0;
        bool Write_File = false;

        byte[] TimeArray;
        byte[] DateArray;
        


        
        

        #endregion

        #region Kinect Variables
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap colorBitmap;

        /// <summary>
        /// Intermediate storage for the color data received from the camera
        /// </summary>
        private byte[] colorPixels;


        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap depthBitmap;

        /// <summary>
        /// Intermediate storage for the depth data received from the camera
        /// </summary>
        private DepthImagePixel[] depthPixels;
        //private byte[] depthPixels = null;

        WriteableBitmap GrayScaleImage;

        short[] DepthGrayScale = new short[640*480];

        private const int MapDepthToByte = 4000 / 256;

        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        
        Skeleton TempSkeleton = new Skeleton();

        #endregion

        #region Constructor and deconstructors

        public MainWindow()
        {
            InitializeComponent();
        }


        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }

        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            
            Debug.WriteLine("Now Time of Day " + DateTime.Now.TimeOfDay);
            Debug.WriteLine("Now Date  " + DateTime.Now.Date);
            Debug.WriteLine("Now " + DateTime.Now);
            uiContext = SynchronizationContext.Current;
            FrameRateTimer.Interval = TimeSpan.FromSeconds(1);
            FrameRateTimer.Tick += FrameRateTimer_Tick;
            FrameRateTimer.Start();
            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {

                SensorIDTextBlock.Text = "Sensor ID: " + sensor.UniqueKinectId;

                // Turn on the color stream to receive color frames
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);


                // Allocate space to put the pixels we'll receive
                this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, 
                    this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                
                // Set the image we display to point to the bitmap where we'll put the image data
                this.ColorImage.Source = this.colorBitmap;

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.ColorFrameReady += this.SensorColorFrameReady;


                // Turn on the depth stream to receive depth frames
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                // Allocate space to put the depth pixels we'll receive
                this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];
                //this.depthPixels = new byte[this.sensor.DepthStream.FramePixelDataLength];

                // This is the bitmap we'll display on-screen
                this.depthBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, 
                    this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Gray8, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                this.DepthImage.Source = this.depthBitmap;

                // Add an event handler to be called whenever there is new depth frame data
                this.sensor.DepthFrameReady += Sensor_DepthFrameReady;


                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusTextBlock.Text = Properties.Resources.NoKinectReady;
            }
        }

        
        void CreateGrayScaleBitmap(short[] pixels)
        {
            var width = 640;
            var height = 480;

            GrayScaleImage = new WriteableBitmap(width, height, 96, 96, PixelFormats.Gray16, null);
            GrayScaleImage.WritePixels(
                new Int32Rect(0, 0, width, height),
                pixels, width * 2
                , 0);
        }

        #endregion

        private void FrameRateTimer_Tick(object sender, EventArgs e)
        {
            
            FrameRateTextBlock.Text = "Skeleton Rate: " + SkeletonRefreshRate.ToString() + 
                " fps," + " Color Rate: " + ColorRefreshRate.ToString() + " fps";
            
            ColorRefreshRate = 0;
            SkeletonRefreshRate = 0;

        }

        #region Kinect Data Ready Events

        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {

                ColorRefreshRate += 1;

                if (colorFrame != null)
                {

                    // Copy the pixel data from the image to a temporary array
                    colorFrame.CopyPixelDataTo(this.colorPixels);

                    // Write the pixel data into our bitmap
                    this.colorBitmap.WritePixels(
                        new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                        this.colorPixels,
                        this.colorBitmap.PixelWidth * sizeof(int),
                        0);

                    if (GuideLinesCheckBox.IsChecked == true)
                    {
                        colorBitmap.FillRectangle(320, 0, 321, 480, Colors.Red);
                        colorBitmap.FillRectangle(0, 240, 640, 241, Colors.Red);
                    }

                    if (SkeletonCheckbox.IsChecked == true) // Show joints points
                    {

                        this.DrawBonesAndJoints(TempSkeleton, colorBitmap);
                        for (int i = 0; i < 20; i++)
                        {
                            byte point_size = 10;

                            
                            switch (Convert.ToByte(JointsPoints[i, 2]))
                            {
                                case 1:
                                    colorBitmap.FillEllipseCentered(Convert.ToInt32(JointsPoints[i, 0]), Convert.ToInt32(JointsPoints[i, 1]), 
                                        (int)(point_size / JointsPoints[i, 3]), (int)(point_size / JointsPoints[i, 3]), Colors.Yellow);
                                    break;
                                case 2:
                                    colorBitmap.FillEllipseCentered(Convert.ToInt32(JointsPoints[i, 0]), Convert.ToInt32(JointsPoints[i, 1]), 
                                        (int)(point_size / JointsPoints[i, 3]), (int)(point_size / JointsPoints[i, 3]), Colors.Green);
                                    break;
                            }
                            JointsPoints[i, 2] = 0;
                        }

                    }

                    
                }
            }

        }

        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            #region Save Received Joints into a Array
            //bool dataReceived = false;
            SkeletonRefreshRate += 1;
            for (int i = 0; i < 20; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    JointsPoints[i, j] = 0;
                }
            }

            #endregion 

            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            if (skeletons.Length != 0) // If there is a tracked skeleton
            {
                foreach (Skeleton skel in skeletons)
                {


                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {

                        TempSkeleton = skel;

                        //this.DrawBonesAndJoints(skel, dc);
                        byte i = 0;

                        #region Map Joints positions to Color coordinates

                        foreach (Joint joint in skel.Joints)
                        {
                            
                           
                            ColorImagePoint colorpoint =
                            this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint
                                (joint.Position, ColorImageFormat.RgbResolution640x480Fps30);

                            JointsPoints[i, 3] = joint.Position.Z;
                            JointsPoints[i, 0] = colorpoint.X;
                            JointsPoints[i, 1] = colorpoint.Y;

                            switch (joint.TrackingState)
                            {
                                case JointTrackingState.NotTracked:
                                    JointsPoints[i, 2] = 0;
                                    break;
                                case JointTrackingState.Inferred:
                                    JointsPoints[i, 2] = 1;
                                    break;
                                case JointTrackingState.Tracked:
                                    JointsPoints[i, 2] = 2;
                                    break;
                            }
                            i++;
                        }

                        #endregion


                        #region Write binary file

                        /*
                                ----------  < Packet > 322Bytes
                                ---------
                                | $ | $ |  Flags 2 bytes
                                ---------

                                ----------------------------------------------
                                | Time | Date | Body Pos | 18 Bytes
                                ----------------------------------------------
                                Time            > 4 Bytes (Unit 32)
                                Date            > 2 Bytes (Uint 16)
                                Body Position X >   4 Bytes (Float)
                                Body Position Y >   4 Bytes (Float)
                                Body Position Z >   4 Bytes (Float)

                                       -----------------------------------------------------------
                                20  x  | % | ST | JT | Px | Py | Pz | 15 Bytes
                                       -----------------------------------------------------------

                                % Sign          >   1 Byte
                                Status          >   1 Byte
                                                          Not Tracked     0
                                                          Inferred        1
                                                          Tracked         2 
                                Joint Type      >   1 Byte
                                                          1.HipCenter
                                                          2.Spine
                                                          3.ShoulderCenter
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

                                Position X      >   4 Bytes (Float)
                                Position Y      >   4 Bytes (Float)
                                Position Z      >   4 Bytes (Float)


                                ----------
                                | E | E | Flag 2 bytes
                                ----------

                                ----------  </Packet >
                                */

                        if (Write_File) // if (Write_File)
                        {
                            TimeValue = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                            // Time Value = MiliSec  + { 1000 x [ Second + 60 x ( Minute + 60 x hour ) ] } > Number of Miliseconds form 00:00 am
                            TimePeriod = DateTime.Today.Subtract(new System.DateTime(2000, 1, 1, 0, 0, 0));
                            DateValue = Convert.ToUInt16(TimePeriod.TotalDays);
                            // Date Value = Days Number from 1 Jan 2000

                            TimeArray = BitConverter.GetBytes(TimeValue);
                            DateArray = BitConverter.GetBytes(DateValue);


                            fileStream.WriteByte(36); // Flag $ Sign
                            fileStream.WriteByte(36);   // Flag $ Sign
                                                        //fileStream.WriteByte(36);   // Flag $ Sign
                            fileStream.Write(TimeArray, 0, TimeArray.Length);     // 32 bit Time : Number of Miliseconds form 00:00 am
                            fileStream.Write(DateArray, 0, DateArray.Length);   // 16 bit Date : Day Number from 1 Jan 2000


                            fileStream.Write(BitConverter.GetBytes(skel.Position.X), 0, 4); // Body Position X
                            fileStream.Write(BitConverter.GetBytes(skel.Position.Y), 0, 4); // Body Position Y
                            fileStream.Write(BitConverter.GetBytes(skel.Position.Z), 0, 4); // Body Position Z


                            foreach (Joint joint in skel.Joints)
                            {
                                byte joint_no = Joint_No(joint.JointType);

                                fileStream.WriteByte(37); // flag % Sign

                                // Write Joint Tracking Status
                                switch (joint.TrackingState)
                                {
                                    case JointTrackingState.NotTracked:
                                        fileStream.WriteByte(0);
                                        break;
                                    case JointTrackingState.Inferred:
                                        fileStream.WriteByte(1);
                                        break;
                                    case JointTrackingState.Tracked:
                                        fileStream.WriteByte(2);
                                        break;

                                }

                                // Write Joint No
                                fileStream.WriteByte(joint_no);

                                // Write Joint Position
                                fileStream.Write(BitConverter.GetBytes(joint.Position.X), 0, 4);
                                fileStream.Write(BitConverter.GetBytes(joint.Position.Y), 0, 4);
                                fileStream.Write(BitConverter.GetBytes(joint.Position.Z), 0, 4);
                            }

                            //flag = 'E'; End Flag
                            fileStream.WriteByte(69);
                            fileStream.WriteByte(69);

                        }
                        #endregion

                    }
                }
            }



        }

        private void Sensor_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);

                    // Get the min and max reliable depth for the current frame
                    int minDepth = depthFrame.MinDepth;
                    int maxDepth = depthFrame.MaxDepth;
                    byte[] depthArray = new byte[640 * 480];
                    // Convert the depth to RGB
                    for (int i = 0; i < this.depthPixels.Length; ++i)
                    {
                        // Get the depth for this pixel
                        short depth = depthPixels[i].Depth;

                        // To convert to a byte, we're discarding the most-significant
                        // rather than least-significant bits.
                        // We're preserving detail, although the intensity will "wrap."
                        // Values outside the reliable depth range are mapped to 0 (black).

                        // Note: Using conditionals in this loop could degrade performance.
                        // Consider using a lookup table instead when writing production code.
                        // See the KinectDepthViewer class used by the KinectExplorer sample
                        // for a lookup table example.
                        //byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                        DepthGrayScale[i] = depth;

                        depthArray[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
                       
                    }

                    // Write the pixel data into our bitmap
                    this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                depthArray,
                this.depthBitmap.PixelWidth,
                0);

                    CreateGrayScaleBitmap(DepthGrayScale);
                    

                }
            }
        }

        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        #endregion

        #region Data Packet and Saving

        byte Joint_No(JointType JointName)
        {
            byte return_val;

            switch (JointName)
            {
                case JointType.HipCenter:
                    return_val = 1;
                    break;
                case JointType.Spine:
                    return_val = 2;
                    break;
                case JointType.ShoulderCenter:
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
                default:
                    return_val = 0;
                    break;
            }

            return return_val;
        }


        #endregion

        #region Draw Skeleton Functions 

        /// <summary>
        /// Maps Skeleton positions to Depth Image coordinates
        /// </summary>
        /// <param name="skelpoint"> Skeleton Joints Points </param>
        /// <returns></returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            //DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            //return new Point(depthPoint.X, depthPoint.Y);

            ColorImagePoint colorPoint = this.sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skelpoint, ColorImageFormat.RgbResolution640x480Fps30);
            return new Point(colorPoint.X, colorPoint.Y);
        }
        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, WriteableBitmap drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                drawingContext.DrawLine((int)this.SkeletonPointToScreen(joint0.Position).X, (int)this.SkeletonPointToScreen(joint0.Position).Y
                , (int)this.SkeletonPointToScreen(joint1.Position).X, (int)this.SkeletonPointToScreen(joint1.Position).Y
                , Colors.Red);
                return;
            }

            if (joint0.TrackingState == JointTrackingState.Inferred ||
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                drawingContext.DrawLine((int)this.SkeletonPointToScreen(joint0.Position).X, (int)this.SkeletonPointToScreen(joint0.Position).Y
                , (int)this.SkeletonPointToScreen(joint1.Position).X, (int)this.SkeletonPointToScreen(joint1.Position).Y
                , Colors.Yellow);
                return;

            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            //Pen drawPen = this.inferredBonePen;
            //if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            //{
            //    drawPen = this.trackedBonePen;
            //}

            drawingContext.DrawLine((int)this.SkeletonPointToScreen(joint0.Position).X , (int)this.SkeletonPointToScreen(joint0.Position).Y
                , (int)this.SkeletonPointToScreen(joint1.Position).X , (int)this.SkeletonPointToScreen(joint1.Position).Y
                , Colors.GreenYellow);
            // drawingContext.DrawLine
        }


        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, WriteableBitmap drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            //// Render Joints
            //foreach (Joint joint in skeleton.Joints)
            //{
            //    Brush drawBrush = null;

            //    if (joint.TrackingState == JointTrackingState.Tracked)
            //    {
            //        drawBrush = this.trackedJointBrush;
            //    }
            //    else if (joint.TrackingState == JointTrackingState.Inferred)
            //    {
            //        drawBrush = this.inferredJointBrush;
            //    }

            //    if (drawBrush != null)
            //    {
            //        drawingContext.FillEllipseCentered( this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
            //    }
            //}
        }


        #endregion

        #region controllers and buttons
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

        private void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            HostInfo = Dns.GetHostEntry(Dns.GetHostName());

            IPAddress newAdd = IPAddress.Parse(IPAddTextBox.Text);
            IPEndPoint ipEndPoint = new IPEndPoint(newAdd, Convert.ToInt16(PortNoTextBox.Text));
            client = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            client.NoDelay = true;
            NetworkStatUpdate("Please wait ...", 0);
            client.BeginConnect(ipEndPoint, ConnectCallBack, client);
        }


        #endregion

        #region Socket Programming 
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
                    //RecordButton.IsEnabled = false;

                }
            }
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
                Debug.WriteLine(ecp.Data);
                Debug.WriteLine(ecp.ErrorCode);
                Debug.WriteLine(ecp.HResult);
                Debug.WriteLine(ecp.SocketErrorCode);
                Debug.WriteLine(ecp.NativeErrorCode);
                Debug.WriteLine(ecp.StackTrace);
                //clientSockets.Remove(current);
                //clients.Remove(clients.Find(Socket => Socket.Equals(current)));
                return;
            }





            byte[] data;
            byte[] recBuf = new byte[received];
            Array.Copy(buffer, recBuf, received);
            ReceivedText = Encoding.ASCII.GetString(recBuf);
            //Debug.WriteLine("Received Text: " + ReceivedText);
            //StatusTextBlock.Text = ReceivedText;
            if (ReceivedText.ToLower() == "name")
            {
                data = Encoding.ASCII.GetBytes(HostInfo.HostName);
                current.Send(data);
            }
            else if (ReceivedText.ToLower() == "type")
            {
                data = Encoding.ASCII.GetBytes("Kinect 1");
                current.Send(data);
            }
            else if (ReceivedText.ToLower() == "connected")
            {
                uiContext.Send(x => NetworkStatUpdate("Connected.", 0), null);

            }
            else if (ReceivedText.ToLower() == "info")
            {
                current.Send(Encoding.ASCII.GetBytes(this.sensor.UniqueKinectId));
            }
            //else if (ReceivedText.ToLower() == "init")
            //{
            //    uiContext.Send(x => RemoteInit(), null);
            //}
            else if (ReceivedText.ToLower() == "start" && fileStream != null)
            {
                
                uiContext.Send(x => RemoteStart(), null);
            }
            else if (ReceivedText.ToLower() == "stop" )
            {
                uiContext.Send(x => RemoteStop(), null);
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
                    SendTimeArray[i+4] = SysDateArray[i];
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
                fileStream = File.Create(path + "\\" + filename + "-" + HostInfo.HostName + ".ks1");
                EventfileStream = File.Create(path + "\\" + filename + "-Kinect1-" + HostInfo.HostName + ".csv");
                statusTextBlock.Text = filename + " File Created.";
                SendTextSocket("OK");
                NetworkStatusTextBlock.Text = "System Initialized";

                WriteTextFile(EventfileStream, string.Format("Sensor ID , {0}\r\n", this.sensor.UniqueKinectId));

                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "File Create"
                    , DateTime.Now, SysTime));

                

            }
            catch
            {

                SendTextSocket("Failed");
                return;
            }
        }

        
        private void RemoteInit()
        {

            string path = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);

            SaveFileDialog dlg = new Microsoft.Win32.SaveFileDialog();
            dlg.FileName = "Kinect 1 Joint File"; // Default file name
            dlg.DefaultExt = ".bin"; // Default file extension
            dlg.Filter = "Skeleton Binary File (.bin)|*.bin"; // Filter files by extension


            // Show save file dialog box
            Nullable<bool> result = dlg.ShowDialog();

            // Process save file dialog box results
            if (result == true)
            {
                // Save document

                fileStream = File.Create(dlg.FileName);
                
                statusTextBlock.Text = "File Created.";

                
            }
            else
            {
                SendTextSocket("Failed"); // Bin file creation failed
                return;
            }
                


            SendTextSocket("OK");
            NetworkStatusTextBlock.Text = "System Initialized";
        }

        void RemoteStart()
        {
            if (fileStream != null)            
                SendTextSocket("OK");
            else
                SendTextSocket("Failed");
            Write_File = true;
            NetworkStatUpdate("Recording.", 0);

            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Start"
                , DateTime.Now, SysTime));

        }

        void RemoteStop()
        {
            if (fileStream != null )
                SendTextSocket("OK");
            else
                SendTextSocket("Failed");
            Write_File = false;
            uiContext.Send(x => NetworkStatUpdate("Record Finished", 0), null);

            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Stop"
                , DateTime.Now, SysTime));

            fileStream.Close();
            EventfileStream.Close();

        }


        void RemotePause()
        {
            if (fileStream != null && Write_File == true)
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
            if (fileStream != null && Write_File == false)
                SendTextSocket("OK");
            else
                SendTextSocket("Failed");
            Write_File = true;

            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Resume"
                , DateTime.Now, SysTime));

            uiContext.Send(x => NetworkStatUpdate("Recording", 0), null);

        }


        void SendTextSocket(string text)
        {
            byte[] data = Encoding.ASCII.GetBytes(text);

            client.Send(data);
        }

        public void WriteTextFile(FileStream file, string text)
        {
            byte[] data = Encoding.ASCII.GetBytes(text);
            file.WriteAsync(data, 0, data.Length);
        }

        #endregion

        #region Mannual Recording 
        bool CreateFileMannual(string filename)
        {
            try
            {



                string path = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
                fileStream = File.Create(path + "\\" + filename   + ".ks1");
                EventfileStream = File.Create(path + "\\" + filename + "-Kinect1"+".csv");
                statusTextBlock.Text = filename + ".ks1" + " File Created.";



                WriteTextFile(EventfileStream, string.Format("Sensor ID , {0}\r\n", this.sensor.UniqueKinectId));

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
                if (fileStream != null)
                {

                    Write_File = true;
                    NetworkStatusTextBlock.Text = "Saving Data ...";

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

        bool PauseFile()
        {
            if (fileStream != null && Write_File == true)
            {
                Write_File = false;

                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Pause"
                    , DateTime.Now, SysTime));

                NetworkStatusTextBlock.Text = "Record Paused.";

                PauseRecordButton.IsEnabled = false;
                StopRecordButton.IsEnabled = true;
                ResumeRecordButton.IsEnabled = true;
                StartRecordButton.IsEnabled = false;

                return true;

            }
            return false;
        }

        bool ResumeFile()
        {
            if (fileStream != null && Write_File == false)
            {
                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Resume"
                    , DateTime.Now, SysTime));
                Write_File = true;
                NetworkStatusTextBlock.Text = "Saving Data ...";

                PauseRecordButton.IsEnabled = true;
                StopRecordButton.IsEnabled = true;
                ResumeRecordButton.IsEnabled = false;
                StartRecordButton.IsEnabled = false;
                return true;
            }
             
            return false;
        }

        bool StopFile()
        {
            if (fileStream != null)
            {
                NetworkStatusTextBlock.Text = "Record Finished";
                Write_File = false;
                

                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(EventfileStream, string.Format("Event , {0} , Time , {1} ,{2}\r\n", "Record Stop"
                    , DateTime.Now, SysTime));

                fileStream.Close();
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

        private void GenerateFileButton_Click(object sender, RoutedEventArgs e)
        {
            
            string[] illegalChars = new string [9]{ "<", ">", ":","/",@"\","|","?","\"" ,"*"};
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
    }
}
