﻿#pragma checksum "..\..\MainWindow.xaml" "{8829d00f-11b8-4213-878b-770e8597ac16}" "9539399C83310760BF0D93466CD2D17884D25565E90CBFB8860EF64CBC706633"
//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.42000
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using GCS_PROBO;
using System;
using System.Diagnostics;
using System.Net.WebSockets;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Shell;


namespace GCS_PROBO {
    
    
    /// <summary>
    /// MainWindow
    /// </summary>
    public partial class MainWindow : System.Windows.Window, System.Windows.Markup.IComponentConnector {
        
        
        #line 30 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox URITextBox;
        
        #line default
        #line hidden
        
        
        #line 35 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Button ConnectButton;
        
        #line default
        #line hidden
        
        
        #line 41 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Button DisconnectButton;
        
        #line default
        #line hidden
        
        
        #line 54 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox MessageTextBox;
        
        #line default
        #line hidden
        
        
        #line 61 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border WiFiStatus;
        
        #line default
        #line hidden
        
        
        #line 73 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border WebSocketStatus;
        
        #line default
        #line hidden
        
        
        #line 85 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border GamepadStatus;
        
        #line default
        #line hidden
        
        
        #line 118 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LeftAxisDot;
        
        #line default
        #line hidden
        
        
        #line 136 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border RightAxisDot;
        
        #line default
        #line hidden
        
        
        #line 170 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock WiFi_Signal_Val;
        
        #line default
        #line hidden
        
        
        #line 175 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock PacketCount_Val;
        
        #line default
        #line hidden
        
        
        #line 183 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border CurrStateBG;
        
        #line default
        #line hidden
        
        
        #line 184 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock CurrState;
        
        #line default
        #line hidden
        
        
        #line 192 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ProgressBar RightMotor_ProgBar;
        
        #line default
        #line hidden
        
        
        #line 195 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock RightMotor_Val;
        
        #line default
        #line hidden
        
        
        #line 203 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ProgressBar LeftMotor_ProgBar;
        
        #line default
        #line hidden
        
        
        #line 206 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock LeftMotor_Val;
        
        #line default
        #line hidden
        
        
        #line 216 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border BallColorBG;
        
        #line default
        #line hidden
        
        
        #line 217 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock BallColor;
        
        #line default
        #line hidden
        
        
        #line 232 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LT_0;
        
        #line default
        #line hidden
        
        
        #line 240 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LT_1;
        
        #line default
        #line hidden
        
        
        #line 248 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LT_2;
        
        #line default
        #line hidden
        
        
        #line 256 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LT_3;
        
        #line default
        #line hidden
        
        
        #line 264 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LT_4;
        
        #line default
        #line hidden
        
        
        #line 272 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LT_5;
        
        #line default
        #line hidden
        
        
        #line 280 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LT_6;
        
        #line default
        #line hidden
        
        
        #line 288 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Border LT_7;
        
        #line default
        #line hidden
        
        
        #line 313 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock StartPointValue;
        
        #line default
        #line hidden
        
        
        #line 323 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ComboBox StartPointComboBox;
        
        #line default
        #line hidden
        
        
        #line 330 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock KpValue;
        
        #line default
        #line hidden
        
        
        #line 337 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox KpTextBox;
        
        #line default
        #line hidden
        
        
        #line 346 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock KiValue;
        
        #line default
        #line hidden
        
        
        #line 353 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox KiTextBox;
        
        #line default
        #line hidden
        
        
        #line 362 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock KdValue;
        
        #line default
        #line hidden
        
        
        #line 369 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox KdTextBox;
        
        #line default
        #line hidden
        
        
        #line 378 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock SpeedValue;
        
        #line default
        #line hidden
        
        
        #line 385 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox SpeedTextBox;
        
        #line default
        #line hidden
        
        
        #line 395 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock LTThresholdValue;
        
        #line default
        #line hidden
        
        
        #line 402 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox LTThresholdTextBox;
        
        #line default
        #line hidden
        
        
        #line 412 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBlock BallThresholdValue;
        
        #line default
        #line hidden
        
        
        #line 419 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.TextBox BallThresholdTextBox;
        
        #line default
        #line hidden
        
        private bool _contentLoaded;
        
        /// <summary>
        /// InitializeComponent
        /// </summary>
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        public void InitializeComponent() {
            if (_contentLoaded) {
                return;
            }
            _contentLoaded = true;
            System.Uri resourceLocater = new System.Uri("/GCS_PROBO;component/mainwindow.xaml", System.UriKind.Relative);
            
            #line 1 "..\..\MainWindow.xaml"
            System.Windows.Application.LoadComponent(this, resourceLocater);
            
            #line default
            #line hidden
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        [System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Design", "CA1033:InterfaceMethodsShouldBeCallableByChildTypes")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        void System.Windows.Markup.IComponentConnector.Connect(int connectionId, object target) {
            switch (connectionId)
            {
            case 1:
            this.URITextBox = ((System.Windows.Controls.TextBox)(target));
            return;
            case 2:
            this.ConnectButton = ((System.Windows.Controls.Button)(target));
            
            #line 40 "..\..\MainWindow.xaml"
            this.ConnectButton.Click += new System.Windows.RoutedEventHandler(this.ConnectButton_Click);
            
            #line default
            #line hidden
            return;
            case 3:
            this.DisconnectButton = ((System.Windows.Controls.Button)(target));
            
            #line 46 "..\..\MainWindow.xaml"
            this.DisconnectButton.Click += new System.Windows.RoutedEventHandler(this.DisconnectButton_Click);
            
            #line default
            #line hidden
            return;
            case 4:
            this.MessageTextBox = ((System.Windows.Controls.TextBox)(target));
            return;
            case 5:
            this.WiFiStatus = ((System.Windows.Controls.Border)(target));
            return;
            case 6:
            this.WebSocketStatus = ((System.Windows.Controls.Border)(target));
            return;
            case 7:
            this.GamepadStatus = ((System.Windows.Controls.Border)(target));
            return;
            case 8:
            this.LeftAxisDot = ((System.Windows.Controls.Border)(target));
            return;
            case 9:
            this.RightAxisDot = ((System.Windows.Controls.Border)(target));
            return;
            case 10:
            this.WiFi_Signal_Val = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 11:
            this.PacketCount_Val = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 12:
            this.CurrStateBG = ((System.Windows.Controls.Border)(target));
            return;
            case 13:
            this.CurrState = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 14:
            this.RightMotor_ProgBar = ((System.Windows.Controls.ProgressBar)(target));
            return;
            case 15:
            this.RightMotor_Val = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 16:
            this.LeftMotor_ProgBar = ((System.Windows.Controls.ProgressBar)(target));
            return;
            case 17:
            this.LeftMotor_Val = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 18:
            this.BallColorBG = ((System.Windows.Controls.Border)(target));
            return;
            case 19:
            this.BallColor = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 20:
            this.LT_0 = ((System.Windows.Controls.Border)(target));
            return;
            case 21:
            this.LT_1 = ((System.Windows.Controls.Border)(target));
            return;
            case 22:
            this.LT_2 = ((System.Windows.Controls.Border)(target));
            return;
            case 23:
            this.LT_3 = ((System.Windows.Controls.Border)(target));
            return;
            case 24:
            this.LT_4 = ((System.Windows.Controls.Border)(target));
            return;
            case 25:
            this.LT_5 = ((System.Windows.Controls.Border)(target));
            return;
            case 26:
            this.LT_6 = ((System.Windows.Controls.Border)(target));
            return;
            case 27:
            this.LT_7 = ((System.Windows.Controls.Border)(target));
            return;
            case 28:
            this.StartPointValue = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 29:
            this.StartPointComboBox = ((System.Windows.Controls.ComboBox)(target));
            return;
            case 30:
            this.KpValue = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 31:
            this.KpTextBox = ((System.Windows.Controls.TextBox)(target));
            
            #line 339 "..\..\MainWindow.xaml"
            this.KpTextBox.PreviewTextInput += new System.Windows.Input.TextCompositionEventHandler(this.FloatValidationTextBox);
            
            #line default
            #line hidden
            return;
            case 32:
            this.KiValue = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 33:
            this.KiTextBox = ((System.Windows.Controls.TextBox)(target));
            
            #line 355 "..\..\MainWindow.xaml"
            this.KiTextBox.PreviewTextInput += new System.Windows.Input.TextCompositionEventHandler(this.FloatValidationTextBox);
            
            #line default
            #line hidden
            return;
            case 34:
            this.KdValue = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 35:
            this.KdTextBox = ((System.Windows.Controls.TextBox)(target));
            
            #line 371 "..\..\MainWindow.xaml"
            this.KdTextBox.PreviewTextInput += new System.Windows.Input.TextCompositionEventHandler(this.FloatValidationTextBox);
            
            #line default
            #line hidden
            return;
            case 36:
            this.SpeedValue = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 37:
            this.SpeedTextBox = ((System.Windows.Controls.TextBox)(target));
            
            #line 387 "..\..\MainWindow.xaml"
            this.SpeedTextBox.PreviewTextInput += new System.Windows.Input.TextCompositionEventHandler(this.Int8BitValidationTextBox);
            
            #line default
            #line hidden
            return;
            case 38:
            this.LTThresholdValue = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 39:
            this.LTThresholdTextBox = ((System.Windows.Controls.TextBox)(target));
            
            #line 404 "..\..\MainWindow.xaml"
            this.LTThresholdTextBox.PreviewTextInput += new System.Windows.Input.TextCompositionEventHandler(this.Int8BitValidationTextBox);
            
            #line default
            #line hidden
            return;
            case 40:
            this.BallThresholdValue = ((System.Windows.Controls.TextBlock)(target));
            return;
            case 41:
            this.BallThresholdTextBox = ((System.Windows.Controls.TextBox)(target));
            
            #line 421 "..\..\MainWindow.xaml"
            this.BallThresholdTextBox.PreviewTextInput += new System.Windows.Input.TextCompositionEventHandler(this.Int8BitValidationTextBox);
            
            #line default
            #line hidden
            return;
            case 42:
            
            #line 427 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Button)(target)).Click += new System.Windows.RoutedEventHandler(this.WriteButton_Click);
            
            #line default
            #line hidden
            return;
            case 43:
            
            #line 434 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Button)(target)).Click += new System.Windows.RoutedEventHandler(this.CalLTHighButton_Click);
            
            #line default
            #line hidden
            return;
            case 44:
            
            #line 439 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Button)(target)).Click += new System.Windows.RoutedEventHandler(this.CalLTLowButton_Click);
            
            #line default
            #line hidden
            return;
            case 45:
            
            #line 449 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Button)(target)).Click += new System.Windows.RoutedEventHandler(this.ReadyButton_Click);
            
            #line default
            #line hidden
            return;
            case 46:
            
            #line 455 "..\..\MainWindow.xaml"
            ((System.Windows.Controls.Button)(target)).Click += new System.Windows.RoutedEventHandler(this.ConfigButton_Click);
            
            #line default
            #line hidden
            return;
            }
            this._contentLoaded = true;
        }
    }
}

