using System.Collections;
using System.Text;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;

[DisallowMultipleComponent]
public class WhepReceiverSampleStyle : MonoBehaviour
{
    [Header("WHEP Endpoint (MediaMTX)")]
    public string whepUrl = "https://192.168.68.53:8889/vr360/whep";

    [Header("Targets")]
    public Renderer sphereRenderer;
    public bool spawnFullscreenRawImage = true;

    public string builtinProp = "_MainTex";
    public string urpProp     = "_BaseMap";
    public string hdrpProp    = "_BaseColorMap";

    private RTCPeerConnection _pc;
    private RawImage _raw;
    private Texture _lastTex;

    void Awake()
    {
        WebRTC.Initialize();
        Application.runInBackground = true;
        QualitySettings.vSyncCount = 0;
    }

    IEnumerator Start()
    {
        if (spawnFullscreenRawImage) CreateFullscreenRawImage();

        var conf = new RTCConfiguration { iceServers = new RTCIceServer[]{} };
        _pc = new RTCPeerConnection(ref conf);

        _pc.OnTrack = e =>
        {
            if (e.Track is VideoStreamTrack v)
            {
                v.OnVideoReceived += tex =>
                {
                    _lastTex = tex;
                    if (_raw != null && _raw.texture != tex) _raw.texture = tex;
                    ApplyTextureToSphere(tex);
                };
            }
        };

        _pc.AddTransceiver(TrackKind.Video, new RTCRtpTransceiverInit
        {
            direction = RTCRtpTransceiverDirection.RecvOnly
        });

        var offerOp = _pc.CreateOffer();
        yield return offerOp;
        var offer = offerOp.Desc;
        yield return _pc.SetLocalDescription(ref offer);

        var req = new UnityWebRequest(whepUrl, "POST");
        req.uploadHandler   = new UploadHandlerRaw(Encoding.UTF8.GetBytes(offer.sdp));
        req.downloadHandler = new DownloadHandlerBuffer();
        req.SetRequestHeader("Content-Type", "application/sdp");

        req.certificateHandler = new TrustAllCerts();
        req.disposeCertificateHandlerOnDispose = true;

        Debug.Log("[WHEP] POST -> " + whepUrl);
        yield return req.SendWebRequest();
        if (req.result != UnityWebRequest.Result.Success)
        {
            Debug.LogError($"[WHEP] HTTP Error: {req.responseCode} / {req.error}\n{req.downloadHandler.text}");
            yield break;
        }

        var answer = new RTCSessionDescription { type = RTCSdpType.Answer, sdp = req.downloadHandler.text };
        yield return _pc.SetRemoteDescription(ref answer);

        StartCoroutine(WebRTC.Update());
        Debug.Log("[WHEP] Handshake complete. Waiting frames...");
    }

    void OnDestroy()
    {
        _pc?.Close();
        _pc?.Dispose();
        WebRTC.Dispose();
    }

    void ApplyTextureToSphere(Texture tex)
    {
        if (sphereRenderer == null || tex == null) return;
        var mat = sphereRenderer.material;
        if (mat.HasProperty(builtinProp)) mat.SetTexture(builtinProp, tex);
        if (mat.HasProperty(urpProp))     mat.SetTexture(urpProp,     tex);
        if (mat.HasProperty(hdrpProp))    mat.SetTexture(hdrpProp,    tex);
    }

    void CreateFullscreenRawImage()
    {
        var canvasGO = new GameObject("[WHEP] Canvas");
        var canvas   = canvasGO.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvasGO.AddComponent<CanvasScaler>();
        canvasGO.AddComponent<GraphicRaycaster>();

        var rawGO = new GameObject("VideoRawImage");
        rawGO.transform.SetParent(canvasGO.transform, false);
        _raw = rawGO.AddComponent<RawImage>();
        var rt = _raw.rectTransform;
        rt.anchorMin = Vector2.zero; rt.anchorMax = Vector2.one;
        rt.offsetMin = Vector2.zero; rt.offsetMax = Vector2.zero;
    }

    class TrustAllCerts : CertificateHandler
    {
        protected override bool ValidateCertificate(byte[] certificateData) => true;
    }
}
