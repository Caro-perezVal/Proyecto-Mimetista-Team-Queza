import cv2
import urllib.request
import numpy as np

# URL de la ESP32-CAM
url = "http://10.51.87.246/capture"

# ------------------------------------------------
# FUNCIÓN PARA MOSTRAR COLORES DOMINANTES
# ------------------------------------------------
def mostrar_colores_dominantes(hist):

    img = np.zeros((300, 300, 3), dtype=np.uint8)

    hist_flat = hist.flatten()

    # Top 3 colores
    top3 = np.argsort(hist_flat)[-3:]
    top3 = top3[::-1]

    altura = 100

    for i, hue in enumerate(top3):

        color_hsv = np.uint8([[[hue, 255, 255]]])

        color_bgr = cv2.cvtColor(
            color_hsv,
            cv2.COLOR_HSV2BGR
        )[0][0]

        cv2.rectangle(
            img,
            (0, i * altura),
            (300, (i + 1) * altura),
            (
                int(color_bgr[0]),
                int(color_bgr[1]),
                int(color_bgr[2])
            ),
            -1
        )

        cv2.putText(
            img,
            f"HUE: {hue}",
            (10, i * altura + 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

    return img

# ------------------------------------------------
# OBTENER PRIMER FRAME
# ------------------------------------------------
img_resp = urllib.request.urlopen(url)

img_array = np.array(
    bytearray(img_resp.read()),
    dtype=np.uint8
)

frame = cv2.imdecode(
    img_array,
    cv2.IMREAD_COLOR
)

# ------------------------------------------------
# SELECCIONAR ROI
# ------------------------------------------------
roi = cv2.selectROI(
    "Selecciona objeto",
    frame,
    False
)

x, y, w, h = roi

roi_frame = frame[y:y+h, x:x+w]

# ------------------------------------------------
# HISTOGRAMA HSV
# ------------------------------------------------
hsv_roi = cv2.cvtColor(
    roi_frame,
    cv2.COLOR_BGR2HSV
)

roi_hist = cv2.calcHist(
    [hsv_roi],
    [0],
    None,
    [180],
    [0, 180]
)

cv2.normalize(
    roi_hist,
    roi_hist,
    0,
    255,
    cv2.NORM_MINMAX
)

# Mostrar colores dominantes
histograma_img = mostrar_colores_dominantes(
    roi_hist
)

# ------------------------------------------------
# TRACKER
# ------------------------------------------------
track_window = (x, y, w, h)

terminacion = (
    cv2.TERM_CRITERIA_EPS |
    cv2.TERM_CRITERIA_COUNT,
    10,
    1
)

centroide_anterior = None

# ------------------------------------------------
# LOOP PRINCIPAL
# ------------------------------------------------
while True:

    # Leer frame desde ESP32
    img_resp = urllib.request.urlopen(url)

    img_array = np.array(
        bytearray(img_resp.read()),
        dtype=np.uint8
    )

    frame = cv2.imdecode(
        img_array,
        cv2.IMREAD_COLOR
    )

    # Convertir a HSV
    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # Backprojection
    back_proj = cv2.calcBackProject(
        [hsv],
        [0],
        roi_hist,
        [0, 180],
        1
    )

    # CamShift
    ret_camshift, track_window = cv2.CamShift(
        back_proj,
        track_window,
        terminacion
    )

    # Obtener rectángulo
    pts = cv2.boxPoints(ret_camshift)

    pts = np.intp(pts)

    # Dibujar tracking
    cv2.polylines(
        frame,
        [pts],
        True,
        (0, 255, 0),
        2
    )

    # ------------------------------------------------
    # CENTROIDE
    # ------------------------------------------------
    cx = int(np.mean(pts[:, 0]))
    cy = int(np.mean(pts[:, 1]))

    cv2.circle(
        frame,
        (cx, cy),
        5,
        (0, 0, 255),
        -1
    )

    # ------------------------------------------------
    # MOVIMIENTO
    # ------------------------------------------------
    if centroide_anterior is not None:

        distancia = np.sqrt(
            (cx - centroide_anterior[0])**2 +
            (cy - centroide_anterior[1])**2
        )

        print("Movimiento:", distancia)

    centroide_anterior = (cx, cy)

    # ------------------------------------------------
    # MOSTRAR
    # ------------------------------------------------
    cv2.imshow("ESP32 Tracking", frame)

    cv2.imshow(
        "BackProjection",
        back_proj
    )

    cv2.imshow(
        "Colores Dominantes",
        histograma_img
    )

    # Salir con q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
