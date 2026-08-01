namespace DVSManager;

public sealed class ReceiverHello
{
    public int Protocol { get; init; }
    public string Firmware { get; init; } = "";
    public string Device { get; init; } = "";
    public string Serial { get; init; } = "";
}

public sealed class ReceiverConfig
{
    public double Gain { get; init; } = 0.30;
    public int Format { get; init; }
    public int Brightness { get; init; } = 40;
    public int PairWindow { get; init; } = 60;
    public double BaseRpm { get; init; } = 33.3333;
    public double BatteryLow { get; init; } = 3.50;
    public bool LowBatteryLedAlert { get; init; } = true;
    public bool ApFallback { get; init; } = true;
    public int Button1Action { get; init; } = 2;
    public int Button2Action { get; init; } = 0;
}

public sealed class DeckTelemetry
{
    public int Deck { get; init; }
    public bool Assigned { get; init; }
    public string State { get; init; } = "offline";
    public double Rpm { get; init; }
    public double? Loss { get; init; }
    public int? Rssi { get; init; }
    public double Battery { get; init; }
    public bool LowBattery { get; init; }
    public long Age { get; init; }
    public string Calibration { get; init; } = "";
    public string Mac { get; init; } = "--";
}

public sealed class ReceiverStatus
{
    public long Uptime { get; init; }
    public int Format { get; init; }
    public double Gain { get; init; }
    public bool PairOpen { get; init; }
    public int PairRemaining { get; init; }
    public bool PortalActive { get; init; }
    public IReadOnlyList<DeckTelemetry> Decks { get; init; } = Array.Empty<DeckTelemetry>();
}

public sealed class ManagerEvent
{
    public string Name { get; init; } = "";
    public int Deck { get; init; }
    public string Detail { get; init; } = "";
    public long At { get; init; }
}

public sealed class CommandResponse
{
    public int Id { get; init; }
    public bool Ok { get; init; }
    public string Message { get; init; } = "";
}

public sealed record ActivityEntry(DateTime Time, string Title, string Detail);
