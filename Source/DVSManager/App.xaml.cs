using System.Threading;
using System.Windows;
using MessageBox = System.Windows.MessageBox;

namespace DVSManager;

public partial class App : System.Windows.Application
{
    private Mutex? _singleInstance;

    protected override void OnStartup(StartupEventArgs e)
    {
        if (e.Args.Contains("--self-test", StringComparer.OrdinalIgnoreCase))
        {
            try
            {
                SerialReceiverService.RunProtocolSelfTest();
                Environment.ExitCode = 0;
            }
            catch
            {
                Environment.ExitCode = 1;
            }
            Shutdown();
            return;
        }

        _singleInstance = new Mutex(true, "DIY-DVS-Manager-1C3B4FB5", out bool created);
        if (!created)
        {
            MessageBox.Show(
                "DVS Manager is already running in the system tray.",
                "DVS Manager",
                MessageBoxButton.OK,
                MessageBoxImage.Information);
            Shutdown();
            return;
        }

        base.OnStartup(e);
        var window = new MainWindow(e.Args.Contains("--minimized", StringComparer.OrdinalIgnoreCase));
        MainWindow = window;
        if (!e.Args.Contains("--minimized", StringComparer.OrdinalIgnoreCase))
            window.Show();
    }

    protected override void OnExit(ExitEventArgs e)
    {
        _singleInstance?.ReleaseMutex();
        _singleInstance?.Dispose();
        base.OnExit(e);
    }
}
