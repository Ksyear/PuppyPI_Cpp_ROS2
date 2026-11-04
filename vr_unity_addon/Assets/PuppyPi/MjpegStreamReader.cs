using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace PuppyPi
{
    // PuppyPi Media v1 (see README): robot web_video_server streams
    // multipart/x-mixed-replace JPEG parts. Pure C# (no UnityEngine) so it can be tested outside Unity.
    public sealed class MultipartJpegParser
    {
        private const int MaxBuffer = 8 * 1024 * 1024;
        private readonly List<byte> _buf = new List<byte>();
        private int _need = -1;        // bytes of the current JPEG part (from Content-Length)
        private bool _scanToEoi;       // part without Content-Length: read up to FF D9

        public int Frames { get; private set; }
        public int Dropped { get; private set; }

        // Feed raw multipart body bytes (after the HTTP response headers); returns complete JPEG frames in order.
        public List<byte[]> Feed(byte[] data, int count)
        {
            var frames = new List<byte[]>();
            for (int i = 0; i < count; i++) _buf.Add(data[i]);
            while (true)
            {
                if (_need >= 0)
                {
                    if (_buf.Count < _need) break;
                    Take(frames, _need);
                    _need = -1;
                    continue;
                }
                if (_scanToEoi)
                {
                    int eoi = IndexOf(0xFF, 0xD9, 0);
                    if (eoi < 0) break;
                    Take(frames, eoi + 2);
                    _scanToEoi = false;
                    continue;
                }
                // Part header block: [CRLF] --boundary CRLF Header: value CRLF ... CRLF CRLF
                int end = IndexOfHeaderEnd();
                if (end < 0)
                {
                    if (_buf.Count > MaxBuffer) _buf.Clear();
                    break;
                }
                string block = Encoding.ASCII.GetString(_buf.GetRange(0, end).ToArray());
                _buf.RemoveRange(0, end + 4);
                if (block.IndexOf("--", StringComparison.Ordinal) < 0) continue;  // not a part header
                int length = -1;
                foreach (string line in block.Split('\n'))
                {
                    int colon = line.IndexOf(':');
                    if (colon > 0 && line.Substring(0, colon).Trim().Equals("Content-Length", StringComparison.OrdinalIgnoreCase))
                        int.TryParse(line.Substring(colon + 1).Trim(), out length);
                }
                if (length > 0 && length <= MaxBuffer) _need = length; else _scanToEoi = true;
            }
            return frames;
        }

        private void Take(List<byte[]> frames, int n)
        {
            byte[] jpeg = _buf.GetRange(0, n).ToArray();
            _buf.RemoveRange(0, n);
            if (jpeg.Length > 3 && jpeg[0] == 0xFF && jpeg[1] == 0xD8) { frames.Add(jpeg); Frames++; }
            else Dropped++;
        }

        private int IndexOf(byte a, byte b, int from)
        {
            for (int i = from; i + 1 < _buf.Count; i++) if (_buf[i] == a && _buf[i + 1] == b) return i;
            return -1;
        }

        private int IndexOfHeaderEnd()
        {
            for (int i = 0; i + 3 < _buf.Count; i++)
                if (_buf[i] == '\r' && _buf[i + 1] == '\n' && _buf[i + 2] == '\r' && _buf[i + 3] == '\n') return i;
            return -1;
        }
    }

    // Reads one MJPEG URL on a background thread and keeps only the newest frame (no queue → no lag build-up).
    // Uses a plain TcpClient HTTP/1.1 GET: works with Unity's "Allow downloads over HTTP = Not allowed" (that setting
    // only gates UnityWebRequest) and needs no extra package.
    public sealed class MjpegStreamReader : IDisposable
    {
        public readonly string Host;
        public readonly int Port;
        public readonly string PathAndQuery;
        public int ReconnectDelayMs = 2000;
        public int TimeoutMs = 5000;
        public volatile string LastError = "";
        public long BytesReceived;
        public int Connects;

        private readonly object _lock = new object();
        private byte[] _latest;
        private int _seq;
        private Thread _thread;
        private volatile bool _stop;
        private TcpClient _tcp;

        public MjpegStreamReader(string url)
        {
            var uri = new Uri(url);
            Host = uri.Host;
            Port = uri.Port;
            PathAndQuery = uri.PathAndQuery;
        }

        public int FramesReceived { get { lock (_lock) return _seq; } }

        public void Start()
        {
            if (_thread != null) return;
            _stop = false;
            _thread = new Thread(Run) { IsBackground = true, Name = "PuppyPi MJPEG " + PathAndQuery };
            _thread.Start();
        }

        // True when a frame newer than lastSeq is available (lastSeq is updated). Never blocks.
        public bool TryTakeLatest(ref int lastSeq, out byte[] jpeg)
        {
            lock (_lock)
            {
                jpeg = null;
                if (_latest == null || _seq == lastSeq) return false;
                jpeg = _latest;
                lastSeq = _seq;
                return true;
            }
        }

        public void Dispose()
        {
            _stop = true;
            try { _tcp?.Close(); } catch (Exception) { }
            _thread?.Join(1000);
            _thread = null;
        }

        private void Run()
        {
            while (!_stop)
            {
                try { ReadOnce(); }
                catch (Exception e) { if (!_stop) LastError = e.GetType().Name + ": " + e.Message; }
                for (int waited = 0; waited < ReconnectDelayMs && !_stop; waited += 100) Thread.Sleep(100);
            }
        }

        private void ReadOnce()
        {
            using (var tcp = new TcpClient())
            {
                _tcp = tcp;
                tcp.ReceiveTimeout = TimeoutMs;
                tcp.SendTimeout = TimeoutMs;
                var connect = tcp.BeginConnect(Host, Port, null, null);
                if (!connect.AsyncWaitHandle.WaitOne(TimeoutMs)) throw new TimeoutException("connect " + Host + ":" + Port);
                tcp.EndConnect(connect);
                Connects++;
                Stream s = tcp.GetStream();
                byte[] req = Encoding.ASCII.GetBytes("GET " + PathAndQuery + " HTTP/1.1\r\nHost: " + Host + ":" + Port +
                                                     "\r\nConnection: close\r\nAccept: */*\r\n\r\n");
                s.Write(req, 0, req.Length);
                var head = new List<byte>();
                int b;
                while ((b = s.ReadByte()) >= 0)  // HTTP response headers end with CR LF CR LF
                {
                    head.Add((byte)b);
                    int n = head.Count;
                    if (n >= 4 && head[n - 4] == '\r' && head[n - 3] == '\n' && head[n - 2] == '\r' && head[n - 1] == '\n') break;
                    if (n > 16384) throw new IOException("HTTP header too long");
                }
                string headers = Encoding.ASCII.GetString(head.ToArray());
                if (!headers.StartsWith("HTTP/1.") || headers.Split(' ')[1] != "200")
                    throw new IOException("HTTP " + headers.Split('\r')[0]);
                var parser = new MultipartJpegParser();
                var buf = new byte[65536];
                LastError = "";
                while (!_stop)
                {
                    int n = s.Read(buf, 0, buf.Length);
                    if (n <= 0) throw new IOException("stream closed");
                    BytesReceived += n;
                    List<byte[]> frames = parser.Feed(buf, n);
                    if (frames.Count > 0)
                        lock (_lock) { _latest = frames[frames.Count - 1]; _seq++; }
                }
            }
        }
    }
}
