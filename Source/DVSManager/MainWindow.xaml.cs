using Microsoft.Win32;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using Forms = System.Windows.Forms;
using Drawing = System.Drawing;
using Border = System.Windows.Controls.Border;
using Button = System.Windows.Controls.Button;
using ComboBox = System.Windows.Controls.ComboBox;
using ComboBoxItem = System.Windows.Controls.ComboBoxItem;
using ProgressBar = System.Windows.Controls.ProgressBar;
using TextBlock = System.Windows.Controls.TextBlock;
using Color = System.Windows.Media.Color;
using MessageBox = System.Windows.MessageBox;
using SaveFileDialog = Microsoft.Win32.SaveFileDialog;

namespace DVSManager;

public partial class MainWindow : Window
{
    private const string StartupRegistryPath = @"Software\Microsoft\Windows\CurrentVersion\Run";
    private const string StartupRegistryName = "DVS Manager";

    private readonly SerialReceiverService _receiver = new();
    private readonly ObservableCollection<ActivityEntry> _activity = new();
    private readonly ObservableCollection<ActivityEntry> _recentActivity = new();
    private readonly Forms.NotifyIcon _trayIcon;
    private readonly string[] _lastDeckStates = { "", "" };
    private readonly bool[] _lowBatteryNotified = { false, false };
    private readonly RollingLinkMetrics[] _rollingLink =
        { new(TimeSpan.FromSeconds(7)), new(TimeSpan.FromSeconds(7)) };
    private ReceiverConfig _config = new();
    private bool _reallyExit;
    private bool _shownTrayHint;
    private bool _updatingSettings;
    private CancellationTokenSource? _toastCancellation;

    public MainWindow(bool startMinimized)
    {
        InitializeComponent();

        ActivityList.ItemsSource = _activity;
        RecentActivityList.ItemsSource = _recentActivity;
        StartupCheck.IsChecked = IsStartupEnabled();

        _trayIcon = new Forms.NotifyIcon
        {
            Icon = Drawing.SystemIcons.Application,
            Text = "DVS Manager — looking for receiver",
            Visible = true,
            ContextMenuStrip = BuildTrayMenu()
        };
        _trayIcon.DoubleClick += (_, _) => Dispatcher.Invoke(ShowFromTray);
        _trayIcon.BalloonTipClicked += (_, _) => Dispatcher.Invoke(ShowFromTray);

        _receiver.ConnectionChanged += Receiver_ConnectionChanged;
        _receiver.StatusReceived += Receiver_StatusReceived;
        _receiver.ConfigReceived += Receiver_ConfigReceived;
        _receiver.EventReceived += Receiver_EventReceived;
        _receiver.ResponseReceived += Receiver_ResponseReceived;
        _receiver.Start();

        AddActivity("Manager started", startMinimized
            ? "Running minimized in the system tray"
            : "Searching Windows COM ports for the receiver");
    }

    private Forms.ContextMenuStrip BuildTrayMenu()
    {
        var menu = new Forms.ContextMenuStrip();
        var open = new Forms.ToolStripMenuItem("Open DVS Manager");
        open.Click += (_, _) => Dispatcher.Invoke(ShowFromTray);
        var reconnect = new Forms.ToolStripMenuItem("Reconnect receiver");
        reconnect.Click += (_, _) => _receiver.ForceReconnect();
        var exit = new Forms.ToolStripMenuItem("Exit");
        exit.Click += (_, _) => Dispatcher.Invoke(ExitApplication);
        menu.Items.Add(open);
        menu.Items.Add(reconnect);
        menu.Items.Add(new Forms.ToolStripSeparator());
        menu.Items.Add(exit);
        return menu;
    }

    private void Receiver_ConnectionChanged(bool connected, string portName, ReceiverHello? hello)
    {
        Dispatcher.InvokeAsync(() =>
        {
            if (connected && hello is not null)
            {
                ConnectionDot.Fill = GreenBrush;
                ConnectionTitle.Text = "Receiver connected";
                ConnectionDetail.Text = $"USB · {portName} · receiver {hello.Serial}";
                FirmwareSidebarText.Text = $"Firmware {hello.Firmware}";
                ReconnectButton.IsEnabled = true;
                SwapDecksButton.IsEnabled =
                    Version.TryParse(hello.Firmware, out Version? firmwareVersion) &&
                    firmwareVersion >= new Version(1, 2, 3);
                _trayIcon.Text = $"DVS Manager — connected on {portName}";
                AddActivity("Receiver connected", $"{portName} · firmware {hello.Firmware}");
                ShowToast($"Receiver connected on {portName}.");
            }
            else
            {
                ConnectionDot.Fill = AmberBrush;
                ConnectionTitle.Text = "Looking for receiver…";
                ConnectionDetail.Text = "Reconnect the USB cable if detection takes more than a few seconds";
                FirmwareSidebarText.Text = "Firmware —";
                SwapDecksButton.IsEnabled = false;
                _trayIcon.Text = "DVS Manager — receiver disconnected";
                AddActivity("Receiver disconnected", "Automatic reconnection is active");
                _rollingLink[0].Reset();
                _rollingLink[1].Reset();
                SetDeckDisconnected(1);
                SetDeckDisconnected(2);
            }
        });
    }

    private void Receiver_StatusReceived(ReceiverStatus status)
    {
        Dispatcher.InvokeAsync(() =>
        {
            FormatSummaryText.Text =
                $"{(status.Format == 0 ? "Serato CV02" : "Traktor carrier")} · gain {status.Gain:0.00}" +
                (status.PairOpen ? $" · pairing {status.PairRemaining}s" : "");
            foreach (DeckTelemetry deck in status.Decks)
                UpdateDeck(deck);
        });
    }

    private void Receiver_ConfigReceived(ReceiverConfig config)
    {
        Dispatcher.InvokeAsync(() =>
        {
            _config = config;
            _updatingSettings = true;
            try
            {
                SettingsFormatCombo.SelectedIndex = Math.Clamp(config.Format, 0, 1);
                SettingsGainSlider.Value = config.Gain;
                BaseRpmCombo.SelectedIndex = config.BaseRpm > 40 ? 1 : 0;
                QuickBaseRpmCombo.SelectedIndex = config.BaseRpm > 40 ? 1 : 0;
                SelectComboTag(BatteryLowCombo, config.BatteryLow, 0.06);
                SelectComboTag(PairWindowCombo, config.PairWindow, 1);
                BrightnessSlider.Value = config.Brightness;
                LowBatteryLedAlertCheck.IsChecked = config.LowBatteryLedAlert;
                ApFallbackCheck.IsChecked = config.ApFallback;
            }
            finally
            {
                _updatingSettings = false;
            }
        });
    }

    private void Receiver_EventReceived(ManagerEvent managerEvent)
    {
        Dispatcher.InvokeAsync(() =>
        {
            string deck = managerEvent.Deck > 0 ? $"Deck {managerEvent.Deck}" : "Receiver";
            string title = managerEvent.Name switch
            {
                "link_lost" => $"{deck} link lost — timecode holding",
                "link_restored" => $"{deck} link restored",
                "battery_low" => $"{deck} battery low",
                "battery_critical" => $"{deck} battery critical",
                "battery_recovered" => $"{deck} battery recovered",
                "calibration_complete" => $"{deck} calibration completed",
                "ap_disabled" => "Emergency AP is disabled",
                _ => managerEvent.Name.Replace('_', ' ')
            };
            AddActivity(title, managerEvent.Detail);

            // Link and battery notifications are driven by status transitions
            // below so an event followed by telemetry cannot create duplicates.
            if (managerEvent.Name is "calibration_complete")
                ShowTrayNotification(title, managerEvent.Detail, Forms.ToolTipIcon.Info);
            else if (managerEvent.Name is "ap_disabled")
                ShowTrayNotification(title, managerEvent.Detail, Forms.ToolTipIcon.Warning);
        });
    }

    private void Receiver_ResponseReceived(CommandResponse response)
    {
        Dispatcher.InvokeAsync(() =>
        {
            ShowToast(response.Message, !response.Ok);
            AddActivity(response.Ok ? "Receiver command completed" : "Receiver command failed", response.Message);
        });
    }

    private void UpdateDeck(DeckTelemetry deck)
    {
        int index = deck.Deck - 1;
        if (index is < 0 or > 1) return;

        TextBlock identity;
        TextBlock stateText;
        Ellipse stateDot;
        TextBlock batteryText;
        TextBlock batteryPercent;
        ProgressBar batteryBar;
        TextBlock rpmText;
        TextBlock rssiText;
        TextBlock lossText;
        TextBlock ageText;
        Border holdingBorder;
        TextBlock calibrationText;
        TextBlock puckMac;
        TextBlock puckBattery;
        TextBlock puckSignal;
        TextBlock puckLoss;

        if (deck.Deck == 1)
        {
            identity = Deck1IdentityText; stateText = Deck1StateText; stateDot = Deck1StateDot;
            batteryText = Deck1BatteryText; batteryPercent = Deck1BatteryPercentText; batteryBar = Deck1BatteryBar;
            rpmText = Deck1RpmText; rssiText = Deck1RssiText; lossText = Deck1LossText; ageText = Deck1AgeText;
            holdingBorder = Deck1HoldingBorder; calibrationText = Deck1CalibrationText;
            puckMac = Puck1MacText; puckBattery = Puck1BatteryText; puckSignal = Puck1SignalText; puckLoss = Puck1LossText;
        }
        else
        {
            identity = Deck2IdentityText; stateText = Deck2StateText; stateDot = Deck2StateDot;
            batteryText = Deck2BatteryText; batteryPercent = Deck2BatteryPercentText; batteryBar = Deck2BatteryBar;
            rpmText = Deck2RpmText; rssiText = Deck2RssiText; lossText = Deck2LossText; ageText = Deck2AgeText;
            holdingBorder = Deck2HoldingBorder; calibrationText = Deck2CalibrationText;
            puckMac = Puck2MacText; puckBattery = Puck2BatteryText; puckSignal = Puck2SignalText; puckLoss = Puck2LossText;
        }

        identity.Text = deck.Assigned ? $"Puck {ShortMac(deck.Mac)}" : "Waiting for puck";
        puckMac.Text = deck.Assigned ? $"{deck.Mac} · {StateLabel(deck.State)}" : "Not paired";
        (double? averageRssi, double? averageLoss) = _rollingLink[index].Update(deck);

        bool knownBattery = deck.Battery > 0.1;
        double percent = knownBattery ? Math.Clamp((deck.Battery - 3.0) / 1.2 * 100.0, 0, 100) : 0;
        batteryText.Text = knownBattery ? $"{deck.Battery:0.00} V" : "No reading";
        batteryPercent.Text = knownBattery ? $"{percent:0}%" : "—";
        batteryBar.Value = percent;
        batteryBar.Foreground = !knownBattery
            ? MutedBrush
            : deck.Battery < 3.30
                ? RedBrush
                : deck.LowBattery ? AmberBrush : GreenBrush;

        rpmText.Text = $"{deck.Rpm:0.00}";
        rssiText.Text = averageRssi.HasValue ? $"{averageRssi:0} dBm" : "—";
        lossText.Text = averageLoss.HasValue ? $"{averageLoss:0.0}%" : "—";
        ageText.Text = deck.Age > 0 ? $"{deck.Age} ms" : "—";
        calibrationText.Text = string.IsNullOrWhiteSpace(deck.Calibration)
            ? (deck.Assigned ? "No calibration message" : "Waiting for a paired puck")
            : deck.Calibration;

        puckBattery.Text = knownBattery ? $"{deck.Battery:0.00} V" : "—";
        puckSignal.Text = averageRssi.HasValue ? $"{averageRssi:0} dBm" : "—";
        puckLoss.Text = averageLoss.HasValue ? $"{averageLoss:0.0}%" : "—";

        bool holding = string.Equals(deck.State, "holding", StringComparison.OrdinalIgnoreCase);
        bool live = string.Equals(deck.State, "live", StringComparison.OrdinalIgnoreCase);
        holdingBorder.Visibility = holding ? Visibility.Visible : Visibility.Collapsed;
        if (holding)
        {
            stateText.Text = "HOLDING";
            stateDot.Fill = AmberBrush;
        }
        else if (live && deck.LowBattery)
        {
            stateText.Text = "LOW BATTERY";
            stateDot.Fill = AmberBrush;
        }
        else if (live)
        {
            stateText.Text = "LIVE";
            stateDot.Fill = GreenBrush;
        }
        else
        {
            stateText.Text = "OFFLINE";
            stateDot.Fill = RedBrush;
        }

        string previous = _lastDeckStates[index];
        if (!string.IsNullOrEmpty(previous) && previous != deck.State)
        {
            if (holding)
            {
                ShowTrayNotification(
                    $"Deck {deck.Deck} link lost",
                    "Timecode is holding. Switch the deck to Internal while reconnecting the puck.",
                    Forms.ToolTipIcon.Warning);
            }
            else if (live && previous == "holding")
            {
                ShowTrayNotification(
                    $"Deck {deck.Deck} link restored",
                    "Live RPM has resumed. Return the deck to Relative when ready.",
                    Forms.ToolTipIcon.Info);
            }
        }
        _lastDeckStates[index] = deck.State;

        if (deck.LowBattery && !_lowBatteryNotified[index])
        {
            _lowBatteryNotified[index] = true;
            bool critical = knownBattery && deck.Battery < 3.30;
            ShowTrayNotification(
                $"Deck {deck.Deck} battery {(critical ? "critical" : "low")}",
                knownBattery ? $"Puck battery is {deck.Battery:0.00} V." : "Charge the puck soon.",
                Forms.ToolTipIcon.Warning);
        }
        else if (!deck.LowBattery)
        {
            _lowBatteryNotified[index] = false;
        }
    }

    private void SetDeckDisconnected(int deck)
    {
        if (deck == 1)
        {
            Deck1StateText.Text = "OFFLINE";
            Deck1StateDot.Fill = MutedBrush;
            Deck1HoldingBorder.Visibility = Visibility.Collapsed;
        }
        else
        {
            Deck2StateText.Text = "OFFLINE";
            Deck2StateDot.Fill = MutedBrush;
            Deck2HoldingBorder.Visibility = Visibility.Collapsed;
        }
    }

    private static string StateLabel(string state) => state switch
    {
        "live" => "LIVE",
        "holding" => "HOLDING",
        _ => "OFFLINE"
    };

    private static string ShortMac(string mac)
    {
        if (string.IsNullOrWhiteSpace(mac) || mac == "--") return "—";
        string[] parts = mac.Split(':');
        return parts.Length >= 2 ? $"{parts[^2]}:{parts[^1]}" : mac;
    }

    private sealed class RollingLinkMetrics(TimeSpan window)
    {
        private readonly Queue<(DateTime At, double Value)> _rssi = new();
        private readonly Queue<(DateTime At, double Value)> _loss = new();
        private string _mac = "";

        public (double? Rssi, double? Loss) Update(DeckTelemetry deck)
        {
            if (!deck.Assigned)
            {
                Reset();
                return (null, null);
            }

            if (!string.Equals(_mac, deck.Mac, StringComparison.OrdinalIgnoreCase))
            {
                Reset();
                _mac = deck.Mac;
            }

            DateTime now = DateTime.UtcNow;
            if (deck.Rssi.HasValue)
                _rssi.Enqueue((now, deck.Rssi.Value));
            if (deck.Loss.HasValue)
                _loss.Enqueue((now, deck.Loss.Value));

            Prune(_rssi, now);
            Prune(_loss, now);
            return (
                _rssi.Count > 0 ? _rssi.Average(sample => sample.Value) : null,
                _loss.Count > 0 ? _loss.Average(sample => sample.Value) : null);
        }

        public void Reset()
        {
            _rssi.Clear();
            _loss.Clear();
            _mac = "";
        }

        private void Prune(Queue<(DateTime At, double Value)> samples, DateTime now)
        {
            while (samples.Count > 0 && now - samples.Peek().At > window)
                samples.Dequeue();
        }
    }

    private void AddActivity(string title, string detail)
    {
        var entry = new ActivityEntry(DateTime.Now, title, detail);
        _activity.Insert(0, entry);
        _recentActivity.Insert(0, entry);
        while (_activity.Count > 250) _activity.RemoveAt(_activity.Count - 1);
        while (_recentActivity.Count > 3) _recentActivity.RemoveAt(_recentActivity.Count - 1);
    }

    private async void ShowToast(string message, bool error = false)
    {
        _toastCancellation?.Cancel();
        _toastCancellation?.Dispose();
        _toastCancellation = new CancellationTokenSource();
        ToastText.Text = message;
        ToastBorder.Background = error
            ? new SolidColorBrush(Color.FromArgb(50, 255, 107, 125))
            : new SolidColorBrush(Color.FromArgb(45, 85, 216, 255));
        ToastBorder.BorderBrush = error ? RedBrush : CyanBrush;
        ToastBorder.Visibility = Visibility.Visible;
        try
        {
            await Task.Delay(3200, _toastCancellation.Token);
            ToastBorder.Visibility = Visibility.Collapsed;
        }
        catch (OperationCanceledException)
        {
            // Replaced by a newer toast.
        }
    }

    private void ShowTrayNotification(string title, string text, Forms.ToolTipIcon icon)
    {
        // NotifyIcon/Windows balloon notifications do not expose a dependable
        // per-notification "silent" option. Keep the tray icon and in-app
        // Activity/toasts, but suppress system balloons (and their sounds).
    }

    private SolidColorBrush GreenBrush => (SolidColorBrush)FindResource("GreenBrush");
    private SolidColorBrush AmberBrush => (SolidColorBrush)FindResource("AmberBrush");
    private SolidColorBrush RedBrush => (SolidColorBrush)FindResource("RedBrush");
    private SolidColorBrush CyanBrush => (SolidColorBrush)FindResource("CyanBrush");
    private SolidColorBrush MutedBrush => (SolidColorBrush)FindResource("MutedBrush");

    private void Navigation_Click(object sender, RoutedEventArgs e)
    {
        if (sender is not Button button || button.Tag is not string page) return;
        ShowPage(page);
    }

    private void ShowPage(string page)
    {
        OverviewPage.Visibility = page == "Overview" ? Visibility.Visible : Visibility.Collapsed;
        PucksPage.Visibility = page == "Pucks" ? Visibility.Visible : Visibility.Collapsed;
        SettingsPage.Visibility = page == "Settings" ? Visibility.Visible : Visibility.Collapsed;
        ActivityPage.Visibility = page == "Activity" ? Visibility.Visible : Visibility.Collapsed;

        foreach (Button button in new[] { OverviewNav, PucksNav, SettingsNav, ActivityNav })
            button.Style = (Style)FindResource((string?)button.Tag == page ? "NavButtonActive" : "NavButton");
    }

    private void SettingsGainSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (SettingsGainValueText is not null)
            SettingsGainValueText.Text = e.NewValue.ToString("0.00", CultureInfo.InvariantCulture);
    }

    private void BrightnessSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (BrightnessValueText is not null)
            BrightnessValueText.Text = $"{(int)Math.Round(e.NewValue)} / 255";
    }

    private void SettingsSaveButton_Click(object sender, RoutedEventArgs e)
    {
        if (!EnsureConnected()) return;
        var config = new ReceiverConfig
        {
            Gain = SettingsGainSlider.Value,
            Format = SettingsFormatCombo.SelectedIndex,
            Brightness = (int)Math.Round(BrightnessSlider.Value),
            PairWindow = (int)GetSelectedTag(PairWindowCombo, 60),
            BaseRpm = GetSelectedTag(BaseRpmCombo, 33.3333),
            BatteryLow = GetSelectedTag(BatteryLowCombo, 3.50),
            LowBatteryLedAlert = LowBatteryLedAlertCheck.IsChecked == true,
            ApFallback = ApFallbackCheck.IsChecked == true
        };
        _receiver.SaveConfig(config);
    }

    private void CalibrateButton_Click(object sender, RoutedEventArgs e)
    {
        if (!EnsureConnected()) return;
        double selectedRpm = GetSelectedTag(QuickBaseRpmCombo, 33.3333);
        string speed = selectedRpm > 40 ? "45 RPM" : "33⅓ RPM";
        MessageBoxResult answer = MessageBox.Show(
            $"Set both platters to {speed} with pitch at 0% and let them settle.\n\n" +
            "Do not touch either platter during the approximately 10-second sample.\n\nStart calibration now?",
            $"Calibrate pucks at {speed}",
            MessageBoxButton.YesNo,
            MessageBoxImage.Information);
        if (answer == MessageBoxResult.Yes)
        {
            // Commands are queued in order: save the selected reference first,
            // then arm calibration using that newly saved speed.
            _receiver.SaveConfig(new ReceiverConfig
            {
                Gain = _config.Gain,
                Format = _config.Format,
                Brightness = _config.Brightness,
                PairWindow = _config.PairWindow,
                BaseRpm = selectedRpm,
                BatteryLow = _config.BatteryLow,
                LowBatteryLedAlert = _config.LowBatteryLedAlert,
                ApFallback = _config.ApFallback
            });
            _receiver.Calibrate();
        }
    }

    private void OpenPairingButton_Click(object sender, RoutedEventArgs e)
    {
        if (!EnsureConnected()) return;
        _receiver.OpenPairing(_config.PairWindow);
    }

    private void SwapDecksButton_Click(object sender, RoutedEventArgs e)
    {
        if (!EnsureConnected()) return;
        MessageBoxResult answer = MessageBox.Show(
            "Swap the two paired puck assignments?\n\n" +
            "Deck 1 will become Deck 2 and Deck 2 will become Deck 1. " +
            "The audio outputs switch immediately.",
            "Swap DVS decks",
            MessageBoxButton.YesNo,
            MessageBoxImage.Warning);
        if (answer == MessageBoxResult.Yes)
            _receiver.SwapDecks();
    }

    private void ClearPairingsButton_Click(object sender, RoutedEventArgs e)
    {
        if (!EnsureConnected()) return;
        MessageBoxResult answer = MessageBox.Show(
            "Clear both deck assignments and reopen pairing?\n\nAudio links will briefly drop.",
            "Clear DVS pairings",
            MessageBoxButton.YesNo,
            MessageBoxImage.Warning);
        if (answer == MessageBoxResult.Yes)
            _receiver.ClearPairings();
    }

    private bool EnsureConnected()
    {
        if (_receiver.IsConnected) return true;
        ShowToast("Receiver is not connected.", true);
        return false;
    }

    private void ReconnectButton_Click(object sender, RoutedEventArgs e)
    {
        ConnectionTitle.Text = "Reconnecting…";
        ConnectionDot.Fill = AmberBrush;
        _receiver.ForceReconnect();
    }

    private static double GetSelectedTag(ComboBox combo, double fallback)
    {
        if (combo.SelectedItem is ComboBoxItem item &&
            double.TryParse(item.Tag?.ToString(), NumberStyles.Float, CultureInfo.InvariantCulture, out double value))
            return value;
        return fallback;
    }

    private static void SelectComboTag(ComboBox combo, double target, double tolerance)
    {
        for (int i = 0; i < combo.Items.Count; i++)
        {
            if (combo.Items[i] is ComboBoxItem item &&
                double.TryParse(item.Tag?.ToString(), NumberStyles.Float, CultureInfo.InvariantCulture, out double value) &&
                Math.Abs(value - target) <= tolerance)
            {
                combo.SelectedIndex = i;
                return;
            }
        }
    }

    private void StartupCheck_Click(object sender, RoutedEventArgs e)
    {
        if (_updatingSettings) return;
        try
        {
            using RegistryKey key = Registry.CurrentUser.CreateSubKey(StartupRegistryPath, true);
            if (StartupCheck.IsChecked == true)
            {
                string executable = Environment.ProcessPath
                    ?? throw new InvalidOperationException("Could not resolve the application path.");
                key.SetValue(StartupRegistryName, $"\"{executable}\" --minimized");
                ShowToast("DVS Manager will start with Windows.");
            }
            else
            {
                key.DeleteValue(StartupRegistryName, false);
                ShowToast("Windows startup disabled.");
            }
        }
        catch (Exception ex)
        {
            StartupCheck.IsChecked = IsStartupEnabled();
            ShowToast($"Could not update Windows startup: {ex.Message}", true);
        }
    }

    private static bool IsStartupEnabled()
    {
        try
        {
            using RegistryKey? key = Registry.CurrentUser.OpenSubKey(StartupRegistryPath, false);
            return key?.GetValue(StartupRegistryName) is string;
        }
        catch
        {
            return false;
        }
    }

    private void ExportActivityButton_Click(object sender, RoutedEventArgs e)
    {
        var dialog = new SaveFileDialog
        {
            Title = "Export DVS activity",
            Filter = "CSV file (*.csv)|*.csv",
            FileName = $"DVS-activity-{DateTime.Now:yyyy-MM-dd-HHmm}.csv"
        };
        if (dialog.ShowDialog(this) != true) return;

        var csv = new StringBuilder("Time,Title,Detail\r\n");
        foreach (ActivityEntry entry in _activity.Reverse())
        {
            csv.Append(Csv(entry.Time.ToString("O", CultureInfo.InvariantCulture))).Append(',')
               .Append(Csv(entry.Title)).Append(',')
               .Append(Csv(entry.Detail)).Append("\r\n");
        }
        File.WriteAllText(dialog.FileName, csv.ToString(), new UTF8Encoding(true));
        ShowToast("Activity exported.");
    }

    private static string Csv(string value) => $"\"{value.Replace("\"", "\"\"")}\"";

    private void TitleBar_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        if (e.ClickCount == 2)
            ToggleMaximize();
        else if (e.LeftButton == MouseButtonState.Pressed)
            DragMove();
    }

    private void MinimizeButton_Click(object sender, RoutedEventArgs e) => HideToTray();

    private void MaximizeButton_Click(object sender, RoutedEventArgs e) => ToggleMaximize();

    private void CloseButton_Click(object sender, RoutedEventArgs e) => Close();

    private void ToggleMaximize() =>
        WindowState = WindowState == WindowState.Maximized ? WindowState.Normal : WindowState.Maximized;

    private void ShowFromTray()
    {
        Show();
        if (WindowState == WindowState.Minimized)
            WindowState = WindowState.Normal;
        Activate();
        Topmost = true;
        Topmost = false;
        Focus();
    }

    private void HideToTray()
    {
        Hide();
        if (_shownTrayHint) return;
        _shownTrayHint = true;
        ShowTrayNotification(
            "DVS Manager is still running",
            "It will monitor the receiver and pucks from the system tray.",
            Forms.ToolTipIcon.Info);
    }

    private void Window_Closing(object? sender, CancelEventArgs e)
    {
        if (_reallyExit) return;
        e.Cancel = true;
        HideToTray();
    }

    private void ExitApplication()
    {
        _reallyExit = true;
        _toastCancellation?.Cancel();
        _receiver.Dispose();
        _trayIcon.Visible = false;
        _trayIcon.Dispose();
        Close();
        System.Windows.Application.Current.Shutdown();
    }
}
