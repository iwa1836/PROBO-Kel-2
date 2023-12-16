using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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
using System.Net.WebSockets;
using SharpDX.XInput;
using System.Threading;
using System.ComponentModel;
using System.Windows.Threading;
using System.Diagnostics;
using System.Text.RegularExpressions;

namespace GCS_PROBO
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private ClientWebSocket _webSocket;
        DispatcherTimer _controllerTimer = new DispatcherTimer();
        DispatcherTimer _wifiTimer = new DispatcherTimer();
        private int _leftAxisX, _leftAxisY, _rightAxisX, _rightAxisY, _buttons;
        private Controller _controller;

        public MainWindow()
        {
            DataContext = this;
            Loaded += MainWindow_Loaded;
            Closing += MainWindow_Closing;
            InitializeComponent();
            _controllerTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(20) };
            _controllerTimer.Tick += _controllerTimer_Tick;
            _controllerTimer.Start();
            _wifiTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
            _wifiTimer.Tick += _wifiTimer_Tick;
            _wifiTimer.Start();
        }

        private void _controllerTimer_Tick(object sender, EventArgs e)
        {
            AssignControllerVal();
            DisplayControllerVal();

            if (_controller.IsConnected)
            {
                GamepadStatus.Background = new SolidColorBrush(Colors.LimeGreen);
            }
            else
            {
                GamepadStatus.Background = new SolidColorBrush(Colors.Red);
            }
        }

        private void _wifiTimer_Tick(object sender, EventArgs e)
        {
            getWiFiStatus();

            if (_webSocket?.State == WebSocketState.Open)
            {
                WebSocketStatus.Background = new SolidColorBrush(Colors.LimeGreen);
            }
            else
            {
                WebSocketStatus.Background = new SolidColorBrush(Colors.Red);
            }
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            _controller = null;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            try
            {
                _controller = new Controller(UserIndex.One);
            }
            catch (Exception ex)
            {
                MsgBoxAppend("ERROR - " + ex.Message + "\n");
            }
        }

        #region Tombol-tombol
        private async void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                _webSocket = new ClientWebSocket();
                await _webSocket.ConnectAsync(new Uri(URITextBox.Text), CancellationToken.None);
                MsgBoxAppend("Connected to server\n");
                await ReceiveMessages();
            }
            catch (Exception ex)
            {
                MsgBoxAppend("Error connecting to server: " + ex.Message + "\n");
            }
        }

        private async void DisconnectButton_Click(Object sender, RoutedEventArgs e)
        {
            try
            {
                if (_webSocket?.State == WebSocketState.Open)
                {
                    await _webSocket.CloseAsync(WebSocketCloseStatus.NormalClosure, "Closing connection", CancellationToken.None);
                    MsgBoxAppend("Disconnected from server\n");
                }

            }
            catch (Exception ex)
            {
                MsgBoxAppend("Error disconnecting from server: " + ex.Message + "\n");
            }
        }

        private async void WriteButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_webSocket?.State == WebSocketState.Open)
                {
                    float Kp = Convert.ToSingle(KpTextBox.Text, System.Globalization.CultureInfo.InvariantCulture);
                    float Ki = Convert.ToSingle(KiTextBox.Text, System.Globalization.CultureInfo.InvariantCulture);
                    float Kd = Convert.ToSingle(KdTextBox.Text, System.Globalization.CultureInfo.InvariantCulture);
                    byte Speed = Convert.ToByte(SpeedTextBox.Text);
                    byte LTThreshold = Convert.ToByte(LTThresholdTextBox.Text);
                    byte BallThreshold = Convert.ToByte(BallThresholdTextBox.Text);
                    byte[] KpByte = BitConverter.GetBytes(Kp);
                    byte[] KiByte = BitConverter.GetBytes(Ki);
                    byte[] KdByte = BitConverter.GetBytes(Kd);
                    byte[] writeMessage = new byte[17];
                    writeMessage[0] = 0xA5;
                    writeMessage[1] = (byte)StartPointComboBox.SelectedIndex;
                    writeMessage[2] = KpByte[0];
                    writeMessage[3] = KpByte[1];
                    writeMessage[4] = KpByte[2];
                    writeMessage[5] = KpByte[3];
                    writeMessage[6] = KiByte[0];
                    writeMessage[7] = KiByte[1];
                    writeMessage[8] = KiByte[2];
                    writeMessage[9] = KiByte[3];
                    writeMessage[10] = KdByte[0];
                    writeMessage[11] = KdByte[1];
                    writeMessage[12] = KdByte[2];
                    writeMessage[13] = KdByte[3];
                    writeMessage[14] = Speed;
                    writeMessage[15] = LTThreshold;
                    writeMessage[16] = BallThreshold;
                    await SendMessages(writeMessage);
                }
            }
            catch (Exception ex)
            {
                MsgBoxAppend("Error sending read command: " + ex.Message + "\n");
            }
        }

        private async void CalLTHighButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_webSocket?.State == WebSocketState.Open)
                {
                    byte[] calMessage = new byte[1];
                    calMessage[0] = 0xFF;
                    await SendMessages(calMessage);
                }
            }
            catch (Exception ex)
            {
                MsgBoxAppend("Error sending read command: " + ex.Message + "\n");
            }
        }

        private async void CalLTLowButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_webSocket?.State == WebSocketState.Open)
                {
                    byte[] calMessage = new byte[1];
                    calMessage[0] = 0x00;
                    await SendMessages(calMessage);
                }
            }
            catch (Exception ex)
            {
                MsgBoxAppend("Error sending read command: " + ex.Message + "\n");
            }
        }

        private async void ReadyButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_webSocket?.State == WebSocketState.Open)
                {
                    byte[] readyMessage = new byte[1];
                    readyMessage[0] = 0xAA;
                    await SendMessages(readyMessage);
                }
            }
            catch (Exception ex)
            {
                MsgBoxAppend("Error sending read command: " + ex.Message + "\n");
            }
        }

        private async void ConfigButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_webSocket?.State == WebSocketState.Open)
                {
                    byte[] configMessage = new byte[1];
                    configMessage[0] = 0x55;
                    await SendMessages(configMessage);
                }
            }
            catch (Exception ex)
            {
                MsgBoxAppend("Error sending read command: " + ex.Message + "\n");
            }
        }
        #endregion

        #region Kirim Terima Pesan
        private async Task SendMessages(byte[] message)
        {
            if (_webSocket?.State == WebSocketState.Open)
            {
                var segment = new ArraySegment<byte>(message);
                await _webSocket.SendAsync(segment, WebSocketMessageType.Binary, true, CancellationToken.None);
                MsgBoxAppend("Sent message: " + message.ToString() + "\n");
            }
        }

        private async Task ReceiveMessages()
        {
            while (_webSocket?.State == WebSocketState.Open)
            {
                byte[] buffer = new byte[4096];
                WebSocketReceiveResult result = await _webSocket.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);
                if (result.MessageType == WebSocketMessageType.Binary)
                {
                    Application.Current.Dispatcher.Invoke(() =>
                    {
                        MsgBoxAppend("Received message: " + buffer.ToString() + "\n");
                        ParsingData(buffer);
                    });
                    if (buffer[0] == 0xAA)
                    {
                        byte[] data = new byte[7];
                        data[0] = 0xFF;
                        data[1] = (byte)_leftAxisX;
                        data[2] = (byte)_leftAxisY;
                        data[3] = (byte)_rightAxisX;
                        data[4] = (byte)_rightAxisY;
                        data[5] = (byte)(_buttons >> 8);
                        data[6] = (byte)_buttons;
                        await SendMessages(data);
                    } else if (buffer[0] == 0x55)
                    {
                        byte[] data = new byte[3];
                        data[0] = 0x5A;
                        data[1] = (byte)(_buttons >> 8);
                        data[2] = (byte)_buttons;
                        await SendMessages(data);
                    }

                    if (MessageTextBox.LineCount >= 64)
                    {
                        MessageTextBox.Clear();
                    }
                }
            }
        }
        #endregion

        #region Lain-lain
        void MsgBoxAppend(string text)
        {
            MessageTextBox.AppendText(text);
            MessageTextBox.ScrollToEnd();
        }

        void ParsingData(byte[] data)
        {
            try
            {
                if ((data[0] == 0xAA) || (data[0] == 0x55))
                {
                    UInt32 packetCount = (UInt32)data[1] << 24;
                    packetCount |= (UInt32)data[2] << 16;
                    packetCount |= (UInt32)data[3] << 8;
                    packetCount |= (UInt32)data[4];
                    PacketCount_Val.Text = packetCount.ToString();

                    switch(data[5])
                    {
                        case 0:
                            CurrState.Text = "CONFIG";
                            CurrStateBG.Background = new SolidColorBrush(Colors.Orange);
                            break;

                        case 1:
                            CurrState.Text = "READY";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LightBlue);
                            break;

                        case 2:
                            CurrState.Text = "A-SEARCH BALL";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LimeGreen);
                            break;

                        case 3:
                            CurrState.Text = "A-PICK BALL";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LimeGreen);
                            break;

                        case 4:
                            CurrState.Text = "A-DELIVER BALL";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LimeGreen);
                            break;

                        case 5:
                            CurrState.Text = "A-DROP BALL 1";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LimeGreen);
                            break;

                        case 6:
                            CurrState.Text = "A-DROP BALL 2";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LimeGreen);
                            break;

                        case 7:
                            CurrState.Text = "A-DROP BALL 3";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LimeGreen);
                            break;

                        case 8:
                            CurrState.Text = "A-DROP BALL 4";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LimeGreen);
                            break;

                        case 9:
                            CurrState.Text = "A-SEARCH MAN";
                            CurrStateBG.Background = new SolidColorBrush(Colors.LimeGreen);
                            break;

                        case 10:
                            CurrState.Text = "M-RUNNING";
                            CurrStateBG.Background = new SolidColorBrush(Colors.Yellow);
                            break;

                        case 11:
                            CurrState.Text = "M-PICK BALL";
                            CurrStateBG.Background = new SolidColorBrush(Colors.Yellow);
                            break;

                        case 12:
                            CurrState.Text = "M-SHOOT BALL";
                            CurrStateBG.Background = new SolidColorBrush(Colors.Yellow);
                            break;

                        default:
                            CurrState.Text = "INVALID";
                            CurrStateBG.Background = new SolidColorBrush(Colors.Red);
                            break;
                    }

                    Int16 RMotor = (Int16)((Int16)data[6] << 8);
                    RMotor |= (Int16)data[7];
                    RightMotor_Val.Text = RMotor.ToString();
                    RightMotor_ProgBar.Value = map(RMotor, -2000, 2000, 0, 100);

                    Int16 LMotor = (Int16)((Int16)data[8] << 8);
                    LMotor |= (Int16)data[9];
                    LeftMotor_Val.Text = LMotor.ToString();
                    LeftMotor_ProgBar.Value = map(LMotor, -2000, 2000, 0, 100);

                    LT_0.Background = new SolidColorBrush(Color.FromRgb(data[10], data[10], data[10]));
                    LT_1.Background = new SolidColorBrush(Color.FromRgb(data[11], data[11], data[11]));
                    LT_2.Background = new SolidColorBrush(Color.FromRgb(data[12], data[12], data[12]));
                    LT_3.Background = new SolidColorBrush(Color.FromRgb(data[13], data[13], data[13]));
                    LT_4.Background = new SolidColorBrush(Color.FromRgb(data[14], data[14], data[14]));
                    LT_5.Background = new SolidColorBrush(Color.FromRgb(data[15], data[15], data[15]));
                    LT_6.Background = new SolidColorBrush(Color.FromRgb(data[16], data[16], data[16]));
                    LT_7.Background = new SolidColorBrush(Color.FromRgb(data[17], data[17], data[17]));

                    if (data[18] == 1)
                    {
                        BallColor.Text = "ORANGE";
                        BallColorBG.Background = new SolidColorBrush(Colors.Orange);
                    } else if (data[18] == 2)
                    {
                        BallColor.Text = "WHITE";
                        BallColorBG.Background = new SolidColorBrush(Colors.White);
                    } else
                    {
                        BallColor.Text = "UNKNOWN";
                        BallColorBG.Background = new SolidColorBrush(Colors.Red);
                    }
                    BallColor.Text = BallColor.Text + " (" + data[19].ToString() + ")";
                }
                else if (data[0] == 0x5A)
                {
                    switch(data[1])
                    {
                        case 0:
                            StartPointValue.Text = "Red";
                            break;

                        case 1:
                            StartPointValue.Text = "Blue";
                            break;

                        default:
                            StartPointValue.Text = "Invalid";
                            break;
                    }

                    float Kp = BitConverter.ToSingle(data, 2);
                    float Ki = BitConverter.ToSingle(data, 6);
                    float Kd = BitConverter.ToSingle(data, 10);
                    KpValue.Text = Kp.ToString();
                    KiValue.Text = Ki.ToString();
                    KdValue.Text = Kd.ToString();
                    SpeedValue.Text = data[14].ToString();
                    LTThresholdValue.Text = data[15].ToString();
                    BallThresholdValue.Text = data[16].ToString();
                }
            }
            catch (Exception ex)
            {
                MsgBoxAppend("ERROR - " + ex.Message + "\n");
            }
        }

        void AssignControllerVal()
        {
            try
            {
                var state = _controller.GetState();
                _leftAxisX = map(state.Gamepad.LeftThumbX, -32768, 32767, 0, 255);
                _leftAxisY = map(state.Gamepad.LeftThumbY, -32768, 32767, 0, 255);
                _rightAxisX = map(state.Gamepad.RightThumbX, -32768, 32767, 0, 255);
                _rightAxisY = map(state.Gamepad.RightThumbY, -32768, 32767, 0, 255);
                _buttons = (int)state.Gamepad.Buttons;
            }
            catch (Exception ex)
            {
                MsgBoxAppend("ERROR - " + ex.Message + "\n");
            }
        }

        void DisplayControllerVal()
        {
            TranslateTransform LeftAxisTrans = new TranslateTransform(_leftAxisX - 127, -_leftAxisY + 127);
            LeftAxisDot.RenderTransform = LeftAxisTrans;

            TranslateTransform RightAxisTrans = new TranslateTransform(_rightAxisX - 127, -_rightAxisY + 127);
            RightAxisDot.RenderTransform = RightAxisTrans;
        }

        void getWiFiStatus()
        {
            var process = new Process
            {
                StartInfo =
                {
                    FileName = "netsh.exe",
                    Arguments = "wlan show interfaces",
                    UseShellExecute = false,
                    RedirectStandardOutput = true,
                    CreateNoWindow = true
                }
            };
            process.Start();
            
            var output = process.StandardOutput.ReadToEnd();
            var lineSSID = output.Split(new[] {Environment.NewLine}, StringSplitOptions.RemoveEmptyEntries).FirstOrDefault(l => l.Contains("SSID") && !l.Contains("BSSID"));
            if (lineSSID == null)
            {
                WiFiStatus.Background = new SolidColorBrush(Colors.Red);
                WiFi_Signal_Val.Text = "NOT CONNECTED";
                return;
            }
            var ssid = lineSSID.Split(new[] { ":" }, StringSplitOptions.RemoveEmptyEntries)[1].TrimStart();
            var lineSignal = output.Split(new[] { Environment.NewLine }, StringSplitOptions.RemoveEmptyEntries).FirstOrDefault(l => l.Contains("Signal") && !l.Contains("Profile"));
            var signal = lineSignal.Split(new[] { ":" }, StringSplitOptions.RemoveEmptyEntries)[1].TrimStart();
            
            if(ssid == "PROBO KEL 2")
            {
                WiFiStatus.Background = new SolidColorBrush(Colors.LimeGreen);
                WiFi_Signal_Val.Text = signal;
            }
            else
            {
                WiFiStatus.Background = new SolidColorBrush(Colors.Red);
                WiFi_Signal_Val.Text = "NOT CONNECTED";
            }
        }

        private static int map(int value, int fromLow, int fromHigh, int toLow, int toHigh)
        {
            return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
        }

        private void FloatValidationTextBox(object sender, TextCompositionEventArgs e)
        {
            Regex regex = new Regex("[^0-9.-]+");
            e.Handled = regex.IsMatch(e.Text);
        }

        private void Int8BitValidationTextBox(object sender, TextCompositionEventArgs e)
        {
            Regex regex = new Regex("[^0-9]+");
            e.Handled = regex.IsMatch(e.Text);
        }
        #endregion
    }
}
