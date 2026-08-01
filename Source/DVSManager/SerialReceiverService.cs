using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Text.Json;

namespace DVSManager;

public sealed class SerialReceiverService : IDisposable
{
    public const int SupportedProtocol = 1;
    private static readonly TimeSpan PingInterval = TimeSpan.FromSeconds(2);
    private static readonly TimeSpan TelemetryTimeout = TimeSpan.FromSeconds(12);

    private readonly CancellationTokenSource _cancellation = new();
    private readonly ConcurrentQueue<string> _outgoing = new();
    private readonly object _stateLock = new();
    private Task? _worker;
    private SerialPort? _port;
    private int _nextCommandId = 10;
    private string? _preferredPort;
    private bool _disposed;

    public event Action<bool, string, ReceiverHello?>? ConnectionChanged;
    public event Action<ReceiverStatus>? StatusReceived;
    public event Action<ReceiverConfig>? ConfigReceived;
    public event Action<ManagerEvent>? EventReceived;
    public event Action<CommandResponse>? ResponseReceived;

    public bool IsConnected
    {
        get
        {
            lock (_stateLock)
                return _port?.IsOpen == true;
        }
    }

    public string PortName
    {
        get
        {
            lock (_stateLock)
                return _port?.PortName ?? "";
        }
    }

    public void Start()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        _worker ??= Task.Run(() => WorkerLoopAsync(_cancellation.Token));
    }

    public void ForceReconnect()
    {
        SerialPort? port;
        lock (_stateLock)
            port = _port;
        try { port?.Close(); } catch { }
    }

    public int RequestConfig() => QueueCommand("GET_CONFIG");

    public int RequestStatus() => QueueCommand("GET_STATUS");

    public int Calibrate() => QueueCommand("CALIBRATE");

    public int OpenPairing(int seconds) =>
        QueueCommand($"PAIR seconds={Math.Clamp(seconds, 10, 300)}");

    public int ClearPairings() => QueueCommand("REPAIR");

    public int SwapDecks() => QueueCommand("SWAP_DECKS");

    public int SaveConfig(ReceiverConfig config)
    {
        string command = string.Create(
            CultureInfo.InvariantCulture,
            $"SET_CONFIG gain={config.Gain:0.000} fmt={config.Format} " +
            $"bri={config.Brightness} pwin={config.PairWindow} " +
            $"base={config.BaseRpm:0.0000} blow={config.BatteryLow:0.00} " +
            $"lbled={(config.LowBatteryLedAlert ? 1 : 0)} " +
            $"ap={(config.ApFallback ? 1 : 0)} " +
            $"btn1={config.Button1Action} btn2={config.Button2Action}");
        return QueueCommand(command);
    }

    private int QueueCommand(string command)
    {
        if (!IsConnected)
            return 0;
        int id = Interlocked.Increment(ref _nextCommandId);
        _outgoing.Enqueue($"@DVS {command} id={id}");
        return id;
    }

    private async Task WorkerLoopAsync(CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested)
        {
            SerialPort? connected = null;
            try
            {
                connected = TryFindReceiver(cancellationToken);
                if (connected is null)
                {
                    await Task.Delay(1600, cancellationToken);
                    continue;
                }

                lock (_stateLock)
                    _port = connected;

                DateTime lastMachineLine = DateTime.UtcNow;
                DateTime lastPing = DateTime.MinValue;
                while (!cancellationToken.IsCancellationRequested && connected.IsOpen)
                {
                    FlushOutgoing(connected);
                    DateTime now = DateTime.UtcNow;
                    if (now - lastPing >= PingInterval)
                    {
                        connected.WriteLine(
                            $"@DVS PING id={Interlocked.Increment(ref _nextCommandId)}");
                        lastPing = now;
                    }
                    try
                    {
                        string line = connected.ReadLine().Trim();
                        if (TryProcessLine(line))
                            lastMachineLine = DateTime.UtcNow;
                    }
                    catch (TimeoutException)
                    {
                        if (DateTime.UtcNow - lastMachineLine > TelemetryTimeout)
                            throw new IOException("Receiver telemetry timed out.");
                    }
                }
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                break;
            }
            catch
            {
                // USB removal, board reboot and COM-number changes are expected.
                // The outer loop closes the old port and scans again.
            }
            finally
            {
                lock (_stateLock)
                {
                    if (ReferenceEquals(_port, connected))
                        _port = null;
                }
                if (connected is not null)
                {
                    try { connected.Close(); } catch { }
                    connected.Dispose();
                    ConnectionChanged?.Invoke(false, "", null);
                }
            }

            try
            {
                await Task.Delay(900, cancellationToken);
            }
            catch (OperationCanceledException)
            {
                break;
            }
        }
    }

    private SerialPort? TryFindReceiver(CancellationToken cancellationToken)
    {
        string[] ports = SerialPort.GetPortNames()
            .OrderBy(name => string.Equals(name, _preferredPort, StringComparison.OrdinalIgnoreCase) ? 0 : 1)
            .ThenBy(PortSortKey)
            .ToArray();

        foreach (string name in ports)
        {
            cancellationToken.ThrowIfCancellationRequested();
            SerialPort? candidate = null;
            try
            {
                candidate = new SerialPort(name, 115200)
                {
                    // Do not toggle either modem-control line on native USB.
                    // ESP32-S3 CDC implementations and board reset circuits can
                    // interpret DTR/RTS transitions as reset/bootloader requests.
                    DtrEnable = false,
                    RtsEnable = false,
                    ReadTimeout = 260,
                    WriteTimeout = 260,
                    NewLine = "\n",
                    Encoding = System.Text.Encoding.UTF8
                };
                candidate.Open();
                candidate.DiscardInBuffer();
                candidate.DiscardOutBuffer();

                int helloId = Interlocked.Increment(ref _nextCommandId);
                candidate.WriteLine($"@DVS HELLO id={helloId}");
                DateTime deadline = DateTime.UtcNow.AddMilliseconds(1250);
                while (DateTime.UtcNow < deadline)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    try
                    {
                        string line = candidate.ReadLine().Trim();
                        if (!TryParseHello(line, out ReceiverHello? hello) || hello is null)
                            continue;
                        if (hello.Protocol != SupportedProtocol)
                            throw new InvalidDataException(
                                $"Receiver protocol {hello.Protocol} is not supported by this manager.");

                        _preferredPort = name;
                        ConnectionChanged?.Invoke(true, name, hello);
                        candidate.WriteLine($"@DVS GET_CONFIG id={Interlocked.Increment(ref _nextCommandId)}");
                        candidate.WriteLine($"@DVS GET_STATUS id={Interlocked.Increment(ref _nextCommandId)}");
                        return candidate;
                    }
                    catch (TimeoutException)
                    {
                        // Continue until the per-port handshake deadline.
                    }
                }
            }
            catch (OperationCanceledException)
            {
                candidate?.Dispose();
                throw;
            }
            catch
            {
                // Not our receiver, unavailable, or removed during handshake.
            }

            if (candidate is not null)
            {
                try { candidate.Close(); } catch { }
                candidate.Dispose();
            }
        }

        return null;
    }

    private static int PortSortKey(string name)
    {
        string digits = new(name.Where(char.IsDigit).ToArray());
        return int.TryParse(digits, out int number) ? number : int.MaxValue;
    }

    private void FlushOutgoing(SerialPort port)
    {
        while (_outgoing.TryDequeue(out string? command))
            port.WriteLine(command);
    }

    private bool TryProcessLine(string line)
    {
        if (!TryOpenMachineJson(line, out JsonDocument? document) || document is null)
            return false;

        using (document)
        {
            JsonElement root = document.RootElement;
            string type = GetString(root, "type");
            switch (type)
            {
                case "hello":
                    break;
                case "config":
                    ConfigReceived?.Invoke(ParseConfig(root));
                    break;
                case "status":
                    StatusReceived?.Invoke(ParseStatus(root));
                    break;
                case "event":
                    EventReceived?.Invoke(new ManagerEvent
                    {
                        Name = GetString(root, "name"),
                        Deck = GetInt(root, "deck"),
                        Detail = GetString(root, "detail"),
                        At = GetLong(root, "at")
                    });
                    break;
                case "response":
                    var response = new CommandResponse
                    {
                        Id = GetInt(root, "id"),
                        Ok = GetBool(root, "ok"),
                        Message = GetString(root, "message")
                    };
                    // Keepalive acknowledgements prove the link is healthy but
                    // are not user actions, so do not clutter Activity/toasts.
                    if (!string.Equals(response.Message, "pong", StringComparison.OrdinalIgnoreCase))
                        ResponseReceived?.Invoke(response);
                    break;
            }
        }
        return true;
    }

    private static bool TryParseHello(string line, out ReceiverHello? hello)
    {
        hello = null;
        if (!TryOpenMachineJson(line, out JsonDocument? document) || document is null)
            return false;
        using (document)
        {
            JsonElement root = document.RootElement;
            if (GetString(root, "type") != "hello")
                return false;
            hello = new ReceiverHello
            {
                Protocol = GetInt(root, "protocol"),
                Firmware = GetString(root, "firmware"),
                Device = GetString(root, "device"),
                Serial = GetString(root, "serial")
            };
            return true;
        }
    }

    private static bool TryOpenMachineJson(string line, out JsonDocument? document)
    {
        document = null;
        const string prefix = "@DVS ";
        if (!line.StartsWith(prefix, StringComparison.Ordinal))
            return false;
        try
        {
            document = JsonDocument.Parse(line[prefix.Length..]);
            return true;
        }
        catch (JsonException)
        {
            return false;
        }
    }

    private static ReceiverConfig ParseConfig(JsonElement root) => new()
    {
        Gain = GetDouble(root, "gain", 0.30),
        Format = GetInt(root, "format"),
        Brightness = GetInt(root, "brightness", 40),
        PairWindow = GetInt(root, "pairWindow", 60),
        BaseRpm = GetDouble(root, "baseRpm", 33.3333),
        BatteryLow = GetDouble(root, "batteryLow", 3.50),
        LowBatteryLedAlert = GetBool(root, "lowBatteryLedAlert", true),
        ApFallback = GetBool(root, "apFallback", true),
        Button1Action = GetInt(root, "button1Action", 2),
        Button2Action = GetInt(root, "button2Action", 0)
    };

    private static ReceiverStatus ParseStatus(JsonElement root)
    {
        var decks = new List<DeckTelemetry>();
        if (root.TryGetProperty("decks", out JsonElement deckArray) &&
            deckArray.ValueKind == JsonValueKind.Array)
        {
            foreach (JsonElement deck in deckArray.EnumerateArray())
            {
                decks.Add(new DeckTelemetry
                {
                    Deck = GetInt(deck, "deck"),
                    Assigned = GetBool(deck, "assigned"),
                    State = GetString(deck, "state", "offline"),
                    Rpm = GetDouble(deck, "rpm"),
                    Loss = GetNullableDouble(deck, "loss"),
                    Rssi = GetNullableInt(deck, "rssi"),
                    Battery = GetDouble(deck, "battery"),
                    LowBattery = GetBool(deck, "lowBattery"),
                    Age = GetLong(deck, "age"),
                    Calibration = GetString(deck, "cal"),
                    Mac = GetString(deck, "mac", "--")
                });
            }
        }

        return new ReceiverStatus
        {
            Uptime = GetLong(root, "uptime"),
            Format = GetInt(root, "format"),
            Gain = GetDouble(root, "gain", 0.30),
            PairOpen = GetBool(root, "pairOpen"),
            PairRemaining = GetInt(root, "pairRemaining"),
            PortalActive = GetBool(root, "portalActive"),
            Decks = decks
        };
    }

    private static string GetString(JsonElement root, string name, string fallback = "") =>
        root.TryGetProperty(name, out JsonElement value) && value.ValueKind == JsonValueKind.String
            ? value.GetString() ?? fallback
            : fallback;

    private static int GetInt(JsonElement root, string name, int fallback = 0) =>
        root.TryGetProperty(name, out JsonElement value) && value.TryGetInt32(out int parsed)
            ? parsed
            : fallback;

    private static long GetLong(JsonElement root, string name, long fallback = 0) =>
        root.TryGetProperty(name, out JsonElement value) && value.TryGetInt64(out long parsed)
            ? parsed
            : fallback;

    private static double GetDouble(JsonElement root, string name, double fallback = 0) =>
        root.TryGetProperty(name, out JsonElement value) && value.TryGetDouble(out double parsed)
            ? parsed
            : fallback;

    private static double? GetNullableDouble(JsonElement root, string name) =>
        root.TryGetProperty(name, out JsonElement value) &&
        value.ValueKind == JsonValueKind.Number &&
        value.TryGetDouble(out double parsed)
            ? parsed
            : null;

    private static int? GetNullableInt(JsonElement root, string name) =>
        root.TryGetProperty(name, out JsonElement value) &&
        value.ValueKind == JsonValueKind.Number &&
        value.TryGetInt32(out int parsed)
            ? parsed
            : null;

    private static bool GetBool(JsonElement root, string name, bool fallback = false) =>
        root.TryGetProperty(name, out JsonElement value) &&
        (value.ValueKind == JsonValueKind.True || value.ValueKind == JsonValueKind.False)
            ? value.GetBoolean()
            : fallback;

    public static void RunProtocolSelfTest()
    {
        const string helloLine =
            "@DVS {\"type\":\"hello\",\"id\":1,\"protocol\":1,\"firmware\":\"1.2.0\"," +
            "\"device\":\"DVS Receiver\",\"serial\":\"A1B2C3D4\"}";
        if (!TryParseHello(helloLine, out ReceiverHello? hello) ||
            hello is null ||
            hello.Protocol != 1 ||
            hello.Firmware != "1.2.0")
            throw new InvalidDataException("HELLO parser self-test failed.");

        const string configLine =
            "@DVS {\"type\":\"config\",\"id\":2,\"gain\":0.3,\"format\":0," +
            "\"brightness\":40,\"pairWindow\":60,\"baseRpm\":33.3333," +
            "\"batteryLow\":3.5,\"lowBatteryLedAlert\":false,\"apFallback\":true," +
            "\"button1Action\":1,\"button2Action\":3}";
        if (!TryOpenMachineJson(configLine, out JsonDocument? configDocument) || configDocument is null)
            throw new InvalidDataException("CONFIG JSON self-test failed.");
        using (configDocument)
        {
            ReceiverConfig config = ParseConfig(configDocument.RootElement);
            if (Math.Abs(config.Gain - 0.30) > 0.001 ||
                config.Brightness != 40 ||
                config.LowBatteryLedAlert ||
                !config.ApFallback ||
                config.Button1Action != 1 ||
                config.Button2Action != 3)
                throw new InvalidDataException("CONFIG parser self-test failed.");
        }

        const string statusLine =
            "@DVS {\"type\":\"status\",\"id\":0,\"uptime\":12000,\"format\":0," +
            "\"gain\":0.3,\"pairOpen\":false,\"pairRemaining\":0,\"portalActive\":false," +
            "\"decks\":[{\"deck\":1,\"assigned\":true,\"state\":\"live\",\"rpm\":33.33," +
            "\"loss\":0.4,\"rssi\":-56,\"battery\":3.91,\"lowBattery\":false,\"age\":8," +
            "\"cal\":\"trim loaded\",\"mac\":\"AA:BB:CC:DD:EE:01\"}," +
            "{\"deck\":2,\"assigned\":true,\"state\":\"holding\",\"rpm\":33.31," +
            "\"loss\":null,\"rssi\":null,\"battery\":3.42,\"lowBattery\":true,\"age\":1500," +
            "\"cal\":\"cal: command sent...\",\"mac\":\"AA:BB:CC:DD:EE:02\"}]}";
        if (!TryOpenMachineJson(statusLine, out JsonDocument? statusDocument) || statusDocument is null)
            throw new InvalidDataException("STATUS JSON self-test failed.");
        using (statusDocument)
        {
            ReceiverStatus status = ParseStatus(statusDocument.RootElement);
            if (status.Decks.Count != 2 ||
                status.Decks[0].State != "live" ||
                status.Decks[0].Rssi != -56 ||
                status.Decks[1].State != "holding" ||
                !status.Decks[1].LowBattery ||
                status.Decks[1].Loss is not null)
                throw new InvalidDataException("STATUS parser self-test failed.");
        }
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        _cancellation.Cancel();
        SerialPort? port;
        lock (_stateLock)
            port = _port;
        try { port?.Close(); } catch { }
        try { _worker?.Wait(TimeSpan.FromSeconds(2)); } catch { }
        _cancellation.Dispose();
    }
}
