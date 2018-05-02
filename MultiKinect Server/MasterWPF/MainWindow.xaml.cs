using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
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

namespace MasterWPF
{

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public DateTime RecordStartTime;
        public DispatcherTimer StatusTextUpdate = new DispatcherTimer();
        public double RecordDuration;
        public DispatcherTimer mytimer = new DispatcherTimer();
        public FileStream filestream;
        public bool paused = false;
        public class ClientClass
        {
            
            public Socket ClientSocket { get; set; }
            public string ClientName { get; set; }
            public string ClientIP { get; set; }
            public string ClientType { get; set; }
            public string ClientStatus { get; set; }
            public string ClientTime { get; set; }
            public string ClientLatency { get; set; }
            public string ClientSensorID { get; set; }
            public string ClientInfo { get; set; }
            


            public ClientClass(Socket socket, string name, string ip, string type, string status, string time, string latency , string info)
            {
                ClientSocket = socket;
                ClientType = type;
                ClientName = name;
                ClientIP = ip;
                ClientStatus = status;
                ClientTime = time;
                ClientLatency = latency;
                ClientInfo = info;
            }

        }

        public enum Status
        {
            Load, // App Loaded
            Initialize, // Subject Info Saved
            Listen, // Server Start to Listen
            SetClient, // Clients info set
            Recording, // Clients are in recording mode
            Pause, // Clients are paused
            Resume, // Recording is resumed
            Finish // Recording Session terminated
             
        }

        public Status serverStatus;

        private static readonly List<Socket> clientSockets = new List<Socket>();
        private const int BUFFER_SIZE = 2048;
        //private const int PORT = 100;
        private static readonly byte[] buffer = new byte[BUFFER_SIZE];
        int received;

        public static ObservableCollection<ClientClass> clients = new ObservableCollection<ClientClass>();
        public static IPHostEntry ipHostInfo;
        public static IPEndPoint ipEndPoint;
        public static Socket server_socket;
        public static string ReceivedText;
        public static bool NewDataReceived = false;
        public SynchronizationContext uiContext;
        public UInt32 Latency;
        public const int record_delay = 0000;

        #region Constructors and deconstructors
        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
            StatusTextBlock.Text = "Please save Subject Info";
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            System.Media.SystemSounds.Hand.Play();
            Thread.Sleep(1000);
            System.Media.SystemSounds.Beep.Play();
            uiContext = SynchronizationContext.Current;
            StatusTextUpdate.Interval = TimeSpan.FromSeconds(1);
            StatusTextUpdate.Tick += StatusTextUpdate_Tick;
            mytimer.Interval = TimeSpan.FromSeconds(10);
            mytimer.Tick += Mytimer_Tick;
            UInt32 SysTim = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            //ServerTimeTextBlock.Text = "Server " + SysTim.ToString() ;
            serverStatus = Status.Load;
            ClientListView.ItemsSource = clients;
            ipHostInfo = Dns.GetHostEntry(Dns.GetHostName());
            foreach (var item in ipHostInfo.AddressList)
            {
                IPListCombo.Items.Add(item);
            }
        }

        private void StatusTextUpdate_Tick(object sender, EventArgs e)
        {
            StatusTextBlock.Text = String.Format("Recording duration {0} sec", (int)(DateTime.Now - RecordStartTime).TotalSeconds);
        }

        private void Mytimer_Tick(object sender, EventArgs e)
        {
            mytimer.Stop();
            //throw new NotImplementedException();
            if (paused == true && serverStatus != Status.Finish)
            {
                StopButton.IsEnabled = true;
                PauseButton.IsEnabled = true;
            }

            else if (serverStatus == Status.Finish)
            {
                this.Close();
            }

            else
            {
                PauseButton.IsEnabled = true;
            }

            

        }

        ~MainWindow()
        {
            
        }

        #endregion

        #region Buttons and controls

        private void SelectIPButton_Click(object sender, RoutedEventArgs e)
        {

            StatusTextBlock.Text = "Sever is waiting for clients";
            //IPHostEntry newHost = Dns.GetHostEntry("127.0.0.1");
            //Debug.WriteLine("System Host Name : " + newHost.HostName);
            
            SelectIPButton.IsEnabled = false;
            IPListCombo.IsEnabled = false;
            PortNoTextBox.IsEnabled = false;
            Start_Server();

            serverStatus = Status.Listen;
            //    Socket client= socket.Accept();
            //    IPEndPoint clientEndPoint = (IPEndPoint)client.RemoteEndPoint;
                
                   
                
            
        }

        private void IPListCombo_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
        if (IPListCombo.SelectedIndex > -1)
            {
                ipEndPoint = new IPEndPoint(ipHostInfo.AddressList[IPListCombo.SelectedIndex], Convert.ToInt16(PortNoTextBox.Text));
                //ipEndPoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 20212);
                SelectIPButton.IsEnabled = true;
                StatusTextBlock.Text = "Start Server";
            }
        }

        private void InitButton_Click(object sender, RoutedEventArgs e)
        {
            //foreach (ClientClass client in clients)
            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(filestream, string.Format("Clients Init  , Time , {0:t} , {0:d} ,{1}\r\n", DateTime.Now, SysTime.ToString()));

            for (int i = 0; i < clients.Count; i++)

            {
                if (clients[i].ClientStatus.ToLower() == "connected")
                {



                    string reply = SentWaitReceive(clients[i].ClientSocket, String.Format("SubjectID" + SubjectIDTextBox.Text));
                    //Debug.WriteLine("Init Result " + reply);


                    if (reply.ToLower() == "ok")
                    {
                        //ClientClass temp = clients[i];
                        //clients.Remove(temp);
                        clients[i].ClientStatus = "Initialized";
                        //clients.Add( temp);



                    }
                    else
                    {
                        //ClientClass temp = client;
                        //clients.Remove(temp);
                        clients[i].ClientStatus = "Init Failed";
                        //clients.Add(temp);
                    }
                }

            }

            ObservableCollection<ClientClass> temp = new ObservableCollection<ClientClass>(clients);
            clients.Clear();
            foreach (ClientClass item in temp)
                clients.Add(item);
            InitButton.IsEnabled = false;
            TriggerButton.IsEnabled = true;
            serverStatus = Status.SetClient;

        }

        void UpdateButtons(Button button, bool stat)
        {
            button.IsEnabled = stat;
        }

        private void TriggerButton_Click(object sender, RoutedEventArgs e)
        {
            System.Media.SystemSounds.Hand.Play();
            Thread.Sleep(record_delay);
            System.Media.SystemSounds.Beep.Play();
            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(filestream, string.Format("Clients Starts Recording  , Time , {0:t} , {0:d} ,{1}\r\n", DateTime.Now, SysTime.ToString()));
            
            if (!paused)
                for (int i = 0; i < clients.Count; i++)
                {
                    if (clients[i].ClientStatus.ToLower() == "initialized")
                    {



                        string reply = SentWaitReceive(clients[i].ClientSocket, "start");
                        Debug.WriteLine("Start Result " + reply);

                        if (reply.ToLower() == "ok")
                        {
                            //ClientClass temp = clients[i];
                            //clients.Remove(temp);
                            clients[i].ClientStatus = "Recording";
                            //clients.Add(temp);



                        }
                        else
                        {
                            //ClientClass temp = clients[i];
                            //clients.Remove(temp);
                            clients[i].ClientStatus = "Record Failed";
                            //clients.Add(temp);
                        }
                    }

                }

            ObservableCollection<ClientClass> temp = new ObservableCollection<ClientClass>(clients);
            clients.Clear();
            foreach (ClientClass item in temp)
                clients.Add(item);
            TriggerButton.IsEnabled = false;
            //StopButton.IsEnabled = true;
            PauseButton.IsEnabled = true;
            serverStatus = Status.Recording;

            RecordStartTime = DateTime.Now;
            StatusTextUpdate.Start();
            StatusTextBlock.Text = "Recording ...";
        }

        private void StopButton_Click(object sender, RoutedEventArgs e)
        {
            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(filestream, string.Format("Clients Stoped  , Time , {0:t} , {0:d} ,{1}\r\n", DateTime.Now, SysTime.ToString()));

            for (int i = 0; i < clients.Count; i++)
            {
                if (clients[i].ClientStatus.ToLower() == "paused")
                {



                    string reply = SentWaitReceive(clients[i].ClientSocket, "stop");
                    Debug.WriteLine("Stop Result " + reply);

                    if (reply.ToLower() == "ok")
                    {
                        //ClientClass temp = clients[i];
                        //clients.Remove(temp);
                        clients[i].ClientStatus = "Connected";
                        //clients.Add(temp);



                    }
                    else
                    {
                        //ClientClass temp = clients[i];
                        //clients.Remove(temp);
                        clients[i].ClientStatus = "Stop Failed";
                        //clients.Add(temp);
                    }
                }

            }

            ObservableCollection<ClientClass> temp = new ObservableCollection<ClientClass>(clients);
            clients.Clear();
            foreach (ClientClass item in temp)
                clients.Add(item);

            TriggerButton.IsEnabled = false;
            StopButton.IsEnabled = false;
            //InitButton.IsEnabled = true;
            PauseButton.IsEnabled = false;

            WriteTextFile(filestream, string.Format("\r\nClient Host Name , Client IP , Client Type , Client Status , Client Time Difference , Client Connection Latency , Client Info \r\n"));
            foreach (var clientItem in clients)
            {
                WriteTextFile(filestream, string.Format(" {0} , {1} , {2} , {3} , {4} , {5} , {6} \r\n",
                    clientItem.ClientName, clientItem.ClientIP, clientItem.ClientType, clientItem.ClientStatus,
                    clientItem.ClientTime, clientItem.ClientLatency, clientItem.ClientInfo));
            }
            StatusTextUpdate.Stop();
            serverStatus = Status.Finish;
            mytimer.Interval = TimeSpan.FromSeconds(10);
            mytimer.Start();

            StatusTextBlock.Text = "Recording Terminated. Application will shut down automatically.";
        }

        private void StartNewSessionButton_Click(object sender, RoutedEventArgs e)
        {
            SaveFileDialog dlg = new SaveFileDialog();
            dlg.FileName = "SubjectID"; // Default file name
            dlg.DefaultExt = ".csv"; // Default file extension
            dlg.Filter = "CSV File (.csv)|*.csv"; // Filter files by extension
            dlg.InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);

            Nullable<bool> result = dlg.ShowDialog();
            if (result == true)
            {
                try
                {
                    filestream = File.Create(dlg.FileName);
                    MainPlaneStackPanel.Visibility = Visibility.Visible;
                    SaveSubjectButton.IsEnabled = true;
                    SubjectIDTextBox.Text = System.IO.Path.GetFileNameWithoutExtension(dlg.FileName);
                    StartNewSessionButton.IsEnabled = false;

                }
                catch (Exception)
                {

                    throw;
                }


            }
        }

        private void SaveSubjectButton_Click(object sender, RoutedEventArgs e)
        {
            SubjectInfoStackPanel.Visibility = Visibility.Collapsed;
            SaveSubjectButton.IsEnabled = false;
            StatusTextBlock.Text = "Select an IP Address and Port No";
            IPListCombo.IsEnabled = true;

            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
            WriteTextFile(filestream, string.Format("Server Name , {0} \r\n", ipHostInfo.HostName));
            WriteTextFile(filestream, string.Format("Server Save Time , {0:t} , {0:d} ,{1}\r\n", DateTime.Now, SysTime.ToString()));
            WriteTextFile(filestream, string.Format("Subject ID , {0} \r\n", SubjectIDTextBox.Text));
            WriteTextFile(filestream, string.Format("Subject Health , {0} , {1} \r\n", IsHealthyCheckBox.IsChecked, SubjectConditionTextBox.Text));
            WriteTextFile(filestream, string.Format("Subject Age , {0} \r\n", SubjectAgeTextBox.Text));
            WriteTextFile(filestream, string.Format("Subject Weight , {0} \r\n", WeightTextBox.Text));
            WriteTextFile(filestream, string.Format("Subject Height , {0} \r\n", HeightTextBox.Text));
            try { 
            double BMI = Convert.ToDouble(WeightTextBox.Text) / Math.Pow(Convert.ToDouble(HeightTextBox.Text) / 100, 2);
                WriteTextFile(filestream, string.Format("Subject BMI , {0} \r\n", BMI));
            }
            catch
            {
                WriteTextFile(filestream, string.Format("Subject BMI , {0} \r\n", "Not specified"));
            }
            

            if (SubjectIsMaleRadioButton.IsChecked == true)
            {
                WriteTextFile(filestream, string.Format("Subject Gender , Male \r\n"));
            }
            else if (SubjectIsFemaleRadioButton.IsChecked == true)
            {
                WriteTextFile(filestream, string.Format("Subject Gender , Female \r\n"));
            }
            else
                WriteTextFile(filestream, string.Format("Subject Gender , Unspecified \r\n"));

            // Hand Dominances
            WriteTextFile(filestream, string.Format("Subject Hand Dominance "));
            if (RightHandCheckBox.IsChecked == true)
            {
                WriteTextFile(filestream, string.Format(", Right  "));
            }
            if (LeftHandCheckBox.IsChecked == true)
            {
                WriteTextFile(filestream, string.Format(", Left  "));
            }
            WriteTextFile(filestream, string.Format("\r\n"));


            // Foot Dominances
            WriteTextFile(filestream, string.Format("Subject Foot Dominance "));
            if (RightFootCheckBox.IsChecked == true)
            {
                WriteTextFile(filestream, string.Format(", Right  "));
            }
            if (LeftFootCheckBox.IsChecked == true)
            {
                WriteTextFile(filestream, string.Format(", Left  "));
            }
            WriteTextFile(filestream, string.Format("\r\n"));


            // Eye Dominances
            WriteTextFile(filestream, string.Format("Subject Eye Dominance "));
            if (RightEyeCheckBox.IsChecked == true)
            {
                WriteTextFile(filestream, string.Format(", Right  "));
            }
            if (LeftEyeCheckBox.IsChecked == true)
            {
                WriteTextFile(filestream, string.Format(", Left  "));
            }
            WriteTextFile(filestream, string.Format("\r\n"));


            WriteTextFile(filestream, string.Format("Subject's Info , {0} \r\n", CommentsTextBox.Text));

            serverStatus = Status.Initialize;

        }

        private void PauseButton_Click(object sender, RoutedEventArgs e)
        {

            PauseButton.IsEnabled = false;

            if (!paused)
            {
                

                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(filestream, string.Format("Clients Paused  , Time , {0:t} , {0:d} ,{1}\r\n", DateTime.Now, SysTime.ToString()));

                for (int i = 0; i < clients.Count; i++)
                {
                    if (clients[i].ClientStatus.ToLower() == "recording")
                    {



                        string reply = SentWaitReceive(clients[i].ClientSocket, "pause");


                        if (reply.ToLower() == "ok")
                        {

                            clients[i].ClientStatus = "Paused";




                        }
                        else
                        {

                            clients[i].ClientStatus = "Pause Failed";

                        }
                    }

                }

                
                PauseButton.Content = "Resume Record";
                //StopButton.IsEnabled = true;
                RecordDuration = (DateTime.Now - RecordStartTime).TotalSeconds;
                WriteTextFile(filestream, string.Format("      , Record Duration , {0}  \r\n", RecordDuration));
                mytimer.Interval = TimeSpan.FromSeconds(Math.Max(10, (int)(RecordDuration / 9 ))+2);
                
                mytimer.Start();
                paused = true;
                StatusTextBlock.Text = "Recording Paused.";
                StatusTextUpdate.Stop();

            }

            else
            {
                System.Media.SystemSounds.Hand.Play();
                //Thread.Sleep(record_delay);
                System.Media.SystemSounds.Beep.Play();

                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                WriteTextFile(filestream, string.Format("Clients Resumed  , Time , {0:t} , {0:d} ,{1}\r\n", DateTime.Now, SysTime.ToString()));

                
                for (int i = 0; i < clients.Count; i++)
                {
                    if (clients[i].ClientStatus.ToLower() == "paused")
                    {



                        string reply = SentWaitReceive(clients[i].ClientSocket, "resume");


                        if (reply.ToLower() == "ok")
                        {

                            clients[i].ClientStatus = "Recording";




                        }
                        else
                        {

                            clients[i].ClientStatus = "Resume Failed";

                        }
                    }

                    StatusTextBlock.Text = "Recording ...";

                }
                StatusTextUpdate.Start();
                PauseButton.Content = "Pause Record";
                StopButton.IsEnabled = false;
                paused = false;
                RecordStartTime = DateTime.Now;
                mytimer.Interval = TimeSpan.FromSeconds(5);
                mytimer.Start();
            }

            ObservableCollection<ClientClass> temp = new ObservableCollection<ClientClass>(clients);
            clients.Clear();
            foreach (ClientClass item in temp)
                clients.Add(item);

            //TriggerButton.IsEnabled = false;
            //PauseButton.IsEnabled = false;
            //InitButton.IsEnabled = true;
        }

        #endregion

        #region CSV File
        public void WriteTextFile(FileStream file, string text)
        {
            byte[] data = Encoding.ASCII.GetBytes(text);
            //await file.WriteAsync(data, 0, data.Length);
            file.Write(data, 0, data.Length);
        }

        #endregion 

        #region Server Commands

        private void AddClient(Socket clientsocket)
        {

            try
            {

                string name = SentWaitReceive(clientsocket, "name");
                string type = SentWaitReceive(clientsocket, "type");
                //string sensor = SentWaitReceive(clientsocket, "sensor");
                string info = SentWaitReceive(clientsocket, "info");

                // Get Client Time and Calculate Differece
                byte[] MyDateTime = GetClientTime(clientsocket);
                byte[] RecTimeArray = new byte[4];
                byte[] RecDateArray = new byte[2];
                UInt32 ClientTime = 0;

                for (int j = 0; j < 4; j++)
                    ClientTime += MyDateTime[j] * Convert.ToUInt32(Math.Pow(256, Convert.ToDouble(j)));
                //for (int i = 0; i < 2; i++)
                //    SendTimeArray[i + 4] = RecTimeArray[i];

                UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second + 60 * (DateTime.Now.Minute + 60 * (DateTime.Now.Hour))));
                Int32 DiffTime = Convert.ToInt32(SysTime) - Convert.ToInt32(ClientTime);

                //clients.Add(new ClientClass(clientsocket, name, clientsocket.RemoteEndPoint.ToString(), type));
                uiContext.Send(x => clients.Add(new ClientClass(clientsocket, name, clientsocket.RemoteEndPoint.ToString(), type, "Connected", String.Format("Time Diff: " + DiffTime.ToString() + " ms"), String.Format("Latency: " + Latency.ToString() + " ms"), info)), null);

                if (clients.Count > 0)
                    uiContext.Send(x => UpdateButtons(InitButton, true), null);


                byte[] data = Encoding.ASCII.GetBytes("connected");
                NewDataReceived = false;

                clientsocket.Send(data);


            }
            catch (Exception ex)
            {

                Debug.WriteLine("client add exception throw : " + ex.Message);
            }


        }

        private byte[] GetClientTime(Socket socket)
        {
            //byte[] ReceivedByte = new byte [6] ;
            byte[] data = Encoding.ASCII.GetBytes("time");
            NewDataReceived = false;
            socket.Send(data);
            while (!NewDataReceived) ;
            byte[] recBuf = new byte[6];
            Array.Copy(buffer, recBuf, 6);
            return recBuf;
        }

        #endregion

        #region Server Socket
        void Start_Server()
        {
            server_socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            
            server_socket.NoDelay = true;
            server_socket.Bind(ipEndPoint);
            server_socket.Listen(5);
            server_socket.BeginAccept(AcceptCallBack, null);
            
        }

        private void AcceptCallBack(IAsyncResult ar)
        {
            Socket socket;
            

            try
            {
                socket = server_socket.EndAccept(ar);
            }
            catch (ObjectDisposedException) // I cannot seem to avoid this (on exit when properly closing sockets)
            {
                return;
            }

            clientSockets.Add(socket);
            Debug.WriteLine("Client " + socket.RemoteEndPoint + " connected");
            //ClientListView.Items.Add(socket.RemoteEndPoint.ToString());
            socket.BeginReceive(buffer, 0, BUFFER_SIZE, SocketFlags.None, ReceiveCallback, socket);
            AddClient(socket);
            
            server_socket.BeginAccept(AcceptCallBack, null);
        }
      
        private string SentWaitReceive(Socket socket , string text)
        {
            Latency = 0;
            UInt32 SysTime = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second ));
            //string Received_Text = string.Empty;
            byte[] data = Encoding.ASCII.GetBytes(text);
            NewDataReceived = false;
            socket.Send(data);
            while (!NewDataReceived) ;
            Debug.WriteLine("Sent Wait Receive : Receive Text > " + ReceivedText);
            Latency = Convert.ToUInt32(DateTime.Now.Millisecond + 1000 * (DateTime.Now.Second)) - SysTime;
            return ReceivedText;
            
        }

        private void ReceiveCallback(IAsyncResult AR)
        {
            Socket current = (Socket)AR.AsyncState;
            
            
            try
            {
                received = current.EndReceive(AR);
            }
            catch (SocketException ecp)
            {
                Debug.WriteLine("Client forcefully disconnected");
                Debug.WriteLine(ecp.Message);
                // Don't shutdown because the socket may be disposed and its disconnected anyway.
                current.Close();
                clientSockets.Remove(current);
                //clients.Remove(clients.Where(s => s.ClientSocket.Equals(current)).First());
                uiContext.Send(x => clients.Remove(clients.Where(s => s.ClientSocket.Equals(current)).First()), null);
                if (clients.Count == 0)
                {
                    uiContext.Send(x => UpdateButtons(InitButton, false), null);
                    uiContext.Send(x => UpdateButtons(TriggerButton, false), null);
                    uiContext.Send(x => UpdateButtons(StopButton, false), null);
                }
                    

                return;
            }

            byte[] recBuf = new byte[received];
            Array.Copy(buffer, recBuf, received);
            ReceivedText = Encoding.ASCII.GetString(recBuf);
            //for (int j = 0; j < recBuf.Length; j++)
            //{
            //    Debug.Write(recBuf[j].ToString() + "-");
            //}
            //Debug.WriteLine("Received Text: " + ReceivedText);
            NewDataReceived = true;
            //if (text.ToLower() == "get time") // Client requested time
            //{
            //    Console.WriteLine("Text is a get time request");
            //    byte[] data = Encoding.ASCII.GetBytes(DateTime.Now.ToLongTimeString());
            //    current.Send(data);
            //    Console.WriteLine("Time sent to client");
            //}
            //else if (text.ToLower() == "exit") // Client wants to exit gracefully
            //{
            //    // Always Shutdown before closing
            //    current.Shutdown(SocketShutdown.Both);
            //    current.Close();
            //    clientSockets.Remove(current);
            //    Console.WriteLine("Client disconnected");
            //    return;
            //}
            //else
            //{
            //    Console.WriteLine("Text is an invalid request");
            //    byte[] data = Encoding.ASCII.GetBytes("Invalid request");
            //    current.Send(data);
            //    Console.WriteLine("Warning Sent");
            //}

            current.BeginReceive(buffer, 0, BUFFER_SIZE, SocketFlags.None, ReceiveCallback, current);
        }      

        
        #endregion
    }
}
