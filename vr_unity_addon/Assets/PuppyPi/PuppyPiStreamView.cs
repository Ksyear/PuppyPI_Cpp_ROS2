using UnityEngine;

namespace PuppyPi
{
    // Robot camera + LiDAR map as two panels in the headset (PuppyPi Media v1, see README).
    // Address comes from the existing PuppyPiV2Client (connect / discovery), so nothing else to configure.
    // Add this component to a GameObject in the VR app scene, or AddComponent it from an existing script.
    // Left X toggles the panels. Panels follow the BallGrip base like the status HUD and face the headset.
    public sealed class PuppyPiStreamView : MonoBehaviour
    {
        public PuppyPiV2Client client;
        public int videoPort = 8080;
        public string cameraTopic = "/usb_cam/image_raw";
        public string mapTopic = "/lidar_map/image";
        public bool showPanels = true;
        [Tooltip("JPEG decode runs on the main thread; keep below the headset frame budget.")]
        [Range(1, 30)] public int maxCameraFps = 15;
        [Range(1, 5)] public int maxMapFps = 2;
        public Vector3 cameraPanelOffset = new Vector3(0f, 0.20f, 0.60f);
        public float cameraPanelWidth = 0.50f;
        public Vector3 mapPanelOffset = new Vector3(0.40f, 0.14f, 0.50f);
        public float mapPanelWidth = 0.28f;
        [Tooltip("Optional. Default: URP Unlit (add it to Always Included Shaders for player builds).")]
        public Material panelMaterial;

        private Panel _camera, _map;
        private string _address = "";

        private sealed class Panel
        {
            public string Name;
            public GameObject Quad;
            public Texture2D Texture;
            public TextMesh Label;
            public MjpegStreamReader Reader;
            public int Seq;
            public double NextDecode;
            public double LastFrame = -1;
            public float Width;
            public Vector3 Offset;
        }

        private void Start()
        {
            _camera = MakePanel("CAMERA", cameraPanelWidth, cameraPanelOffset);
            _map = MakePanel("LIDAR MAP", mapPanelWidth, mapPanelOffset);
        }

        private Panel MakePanel(string name, float width, Vector3 offset)
        {
            var p = new Panel { Name = name, Width = width, Offset = offset };
            p.Quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
            p.Quad.name = "PuppyPi " + name;
            Destroy(p.Quad.GetComponent<Collider>());
            p.Quad.transform.SetParent(transform, false);
            p.Texture = new Texture2D(2, 2, TextureFormat.RGB24, false) { wrapMode = TextureWrapMode.Clamp };
            Material mat = panelMaterial != null ? new Material(panelMaterial) : new Material(FindShader());
            mat.mainTexture = p.Texture;
            p.Quad.GetComponent<MeshRenderer>().sharedMaterial = mat;
            p.Quad.transform.localScale = new Vector3(width, width * 9f / 16f, 1f);

            var label = new GameObject("PuppyPi " + name + " label");
            label.transform.SetParent(p.Quad.transform, false);
            p.Label = label.AddComponent<TextMesh>();
            p.Label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            p.Label.GetComponent<MeshRenderer>().sharedMaterial = p.Label.font.material;
            p.Label.fontSize = 48;
            p.Label.characterSize = 0.002f;
            p.Label.anchor = TextAnchor.LowerLeft;
            p.Label.color = Color.cyan;
            return p;
        }

        private static Shader FindShader()
        {
            foreach (string n in new[] { "Universal Render Pipeline/Unlit", "Unlit/Texture", "Sprites/Default" })
            {
                Shader s = Shader.Find(n);
                if (s != null) return s;
            }
            return Shader.Find("Hidden/InternalErrorShader");
        }

        private void Update()
        {
            if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.LTouch)) showPanels = !showPanels;
            string address = client != null && client.networkEnabled ? client.robotAddress : "";
            if (address != _address) Restart(address);
            double now = Time.realtimeSinceStartupAsDouble;
            Pump(_camera, now, 1.0 / Mathf.Max(1, maxCameraFps));
            Pump(_map, now, 1.0 / Mathf.Max(1, maxMapFps));
        }

        private void Restart(string address)
        {
            _address = address;
            foreach (Panel p in new[] { _camera, _map })
            {
                if (p == null) continue;
                p.Reader?.Dispose();
                p.Reader = null;
                p.Seq = 0;
                p.LastFrame = -1;
            }
            if (string.IsNullOrEmpty(address) || _camera == null) return;
            _camera.Reader = new MjpegStreamReader(StreamUrl(address, cameraTopic));
            _map.Reader = new MjpegStreamReader(StreamUrl(address, mapTopic));
            _camera.Reader.Start();
            _map.Reader.Start();
        }

        private string StreamUrl(string address, string topic)
        {
            return "http://" + address + ":" + videoPort + "/stream?topic=" + topic + "&type=ros_compressed";
        }

        private void Pump(Panel p, double now, double minInterval)
        {
            if (p == null) return;
            if (p.Reader != null && now >= p.NextDecode && p.Reader.TryTakeLatest(ref p.Seq, out byte[] jpeg))
            {
                if (p.Texture.LoadImage(jpeg, false))
                {
                    p.LastFrame = now;
                    float aspect = p.Texture.width > 0 ? (float)p.Texture.height / p.Texture.width : 9f / 16f;
                    p.Quad.transform.localScale = new Vector3(p.Width, p.Width * aspect, 1f);
                }
                p.NextDecode = now + minInterval;
            }
            bool live = p.LastFrame >= 0 && now - p.LastFrame < 2.0;
            string state = p.Reader == null ? "NOT CONNECTED" : live ? "" : "NO SIGNAL " + (p.Reader.LastError ?? "");
            p.Label.text = p.Name + (state.Length > 0 ? "  " + state : "");
        }

        private void LateUpdate()
        {
            foreach (Panel p in new[] { _camera, _map })
            {
                if (p == null) continue;
                p.Quad.SetActive(showPanels);
                if (!showPanels) continue;
                p.Quad.transform.position = transform.TransformPoint(p.Offset);
                if (Camera.main != null) p.Quad.transform.rotation = Camera.main.transform.rotation;
                // Label sits on the panel's top-left corner (quad local units are -0.5..0.5).
                p.Label.transform.localPosition = new Vector3(-0.5f, 0.52f, 0f);
                Vector3 s = p.Quad.transform.localScale;
                p.Label.transform.localScale = new Vector3(1f / Mathf.Max(0.01f, s.x), 1f / Mathf.Max(0.01f, s.y), 1f);
            }
        }

#if UNITY_EDITOR || UNITY_STANDALONE
        // PC monitor preview (Quest Link runs on the PC): small thumbnails at the bottom-right of the Game view.
        private void OnGUI()
        {
            if (!showPanels) return;
            float w = Mathf.Min(320, Screen.width * 0.25f);
            float y = Screen.height - 10;
            foreach (Panel p in new[] { _map, _camera })
            {
                if (p == null || p.LastFrame < 0) continue;
                float h = w * p.Texture.height / Mathf.Max(1, p.Texture.width);
                y -= h + 6;
                GUI.DrawTexture(new Rect(Screen.width - w - 10, y, w, h), p.Texture, ScaleMode.ScaleToFit);
            }
        }
#endif

        private void OnDestroy()
        {
            _camera?.Reader?.Dispose();
            _map?.Reader?.Dispose();
        }
    }
}
