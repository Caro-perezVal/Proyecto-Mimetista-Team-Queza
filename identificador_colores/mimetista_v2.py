# proyecto mimetista v2.5 - Integrado con Comunicación ESP32-CAM
# ahora usa yolo para detectar personas, calcula movimiento en Hz,
# extrae el color del torso y piernas por separado, y envía de forma alternada
# los colores y el cuadrante de posición a la ESP32-CAM por Wi-Fi.

import cv2
import urllib.request
import numpy as np
import time
import csv
import signal
import sys
import requests
from collections import deque
from datetime import datetime
from ultralytics import YOLO

# --- CONFIGURACIÓN DE HARDWARE Y RED ---
# Dirección IP asignada a tu ESP32-CAM
ESP32_IP = "http://192.168.0.17"  
# Endpoint personalizado para enviar parámetros
URL_CUSTOM_CONTROL = f"{ESP32_IP}/custom_control"
# URL nativa si capturas video desde la misma ESP32-CAM
url_capture = "http://192.168.0.17/capture"

USAR_WEBCAM   = False           # True para usar la cámara local, False para usar la ESP32
YOLO_MODEL    = "yolo11n.pt"   # Modelo nano rápido para ejecución en tiempo real
CONFIANZA_MIN = 0.5            # Umbral de confianza mínimo para YOLO
CLASE_PERSONA = 0              # Índice de la clase persona en el dataset COCO

UMBRAL_MOV = 8                 # Píxeles mínimos de desplazamiento para registrar movimiento
VENTANA_HZ = 1.5               # Ventana de tiempo (segundos) para el cálculo de Hz

GUARDAR_METRICAS = True
ARCHIVO_METRICAS = f"metricas_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# --- VARIABLES GLOBALES DE CONTROL DE ENVÍO ---
ultimo_envio_tiempo = 0
intervalo_red = 0.4            # Límite para evitar saturar el flujo de red (segundos)
alternador_color = True        # Alterna True (Torso) y False (Piernas) en cada envío

# Variables para recordar el último estado enviado y evitar peticiones idénticas innecesarias
ultimo_r, ultimo_g, ultimo_b, ultima_pos = -1, -1, -1, -1


def detectar_color_dominante(roi_bgr):
    if roi_bgr is None or roi_bgr.size == 0:
        return "Desconocido", 0

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    h_ch, s_ch, v_ch = cv2.split(hsv)
    total_px = float(h_ch.size)

    mask_negro  = (v_ch < 60)
    mask_blanco = (v_ch > 180) & (s_ch < 50)
    mask_gris   = (s_ch < 40) & (v_ch >= 60) & (v_ch <= 180)
    mask_cromo  = (s_ch >= 40)

    pct_negro  = mask_negro.sum()  / total_px * 100
    pct_blanco = mask_blanco.sum() / total_px * 100
    pct_gris   = mask_gris.sum()   / total_px * 100
    pct_cromo  = mask_cromo.sum()  / total_px * 100

    if pct_cromo > 30:
        hues_cromo = h_ch[mask_cromo]
        if len(hues_cromo) == 0:
            return "Gris", 0

        hist = np.bincount(hues_cromo.flatten(), minlength=180).astype(float)
        hist = cv2.GaussianBlur(hist.reshape(1, -1), (1, 9), 0).flatten()
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)

        hue_dom = int(np.argmax(hist))
        return hue_a_nombre(hue_dom), hue_dom

    mejor = max(
        [("Negro", pct_negro), ("Blanco", pct_blanco), ("Gris", pct_gris)],
        key=lambda x: x[1]
    )
    return mejor[0], 0


def hue_a_nombre(hue):
    rangos = [
        (0,   10,  "Rojo"),
        (10,  25,  "Naranja"),
        (25,  35,  "Amarillo"),
        (35,  85,  "Verde"),
        (85,  100, "Verde azul"),
        (100, 130, "Azul"),
        (130, 150, "Morado"),
        (150, 170, "Rosa"),
        (170, 180, "Rojo"),
    ]
    for lo, hi, name in rangos:
        if lo <= hue < hi:
            return name
    return "Rojo"


def color_a_bgr(nombre, hue=0):
    acros = {
        "Negro":  (30,  30,  30),
        "Blanco": (240, 240, 240),
        "Gris":   (140, 140, 140),
    }
    if nombre in acros:
        return acros[nombre]
    c = np.uint8([[[hue, 220, 210]]])
    bgr = cv2.cvtColor(c, cv2.COLOR_HSV2BGR)[0][0]
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))


def mostrar_colores_dominantes(color_torso, hue_torso, color_pierna, hue_pierna, hz):
    img = np.zeros((300, 300, 3), dtype=np.uint8)

    bgr_t = color_a_bgr(color_torso, hue_torso)
    cv2.rectangle(img, (0, 0), (300, 140), (int(bgr_t[0]), int(bgr_t[1]), int(bgr_t[2])), -1)
    cv2.putText(img, f"TORSO: {color_torso}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(img, f"HUE: {hue_torso}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    bgr_p = color_a_bgr(color_pierna, hue_pierna)
    cv2.rectangle(img, (0, 150), (300, 290), (int(bgr_p[0]), int(bgr_p[1]), int(bgr_p[2])), -1)
    cv2.putText(img, f"PIERNA: {color_pierna}", (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(img, f"HUE: {hue_pierna}", (10, 245), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    cv2.putText(img, f"Hz: {hz:.1f}", (10, 285), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return img


def get_frame_esp32():
    try:
        img_resp  = urllib.request.urlopen(url_capture, timeout=5)
        img_array = np.array(bytearray(img_resp.read()), dtype=np.uint8)
        return cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"[warn] mi esp32 no respondio al streaming de video: {e}")
        return None


# --- FUNCIÓN INTEGRADA DE COMUNICACIÓN WI-FI ---
def mandar_datos_a_esp32(r, g, b, cuadrante):
    global ultimo_envio_tiempo, ultimo_r, ultimo_g, ultimo_b, ultima_pos
    ahora = time.time()
    
    # Evitamos saturar el hardware de red del ESP32 controlando los tiempos de envío
    if (ahora - ultimo_envio_tiempo) < intervalo_red:
        return

    # Si los datos no han cambiado en lo absoluto, evitamos hacer una petición redundante
    if r == ultimo_r and g == ultimo_g and b == ultimo_b and cuadrante == ultima_pos:
        return

    parametros = {'r': r, 'g': g, 'b': b, 'pos': cuadrante}
    try:
        # Se asigna un timeout corto para evitar congelamientos en el procesamiento visual
        response = requests.get(URL_CUSTOM_CONTROL, params=parametros, timeout=1.2)
        if response.status_code == 200:
            print(f"-> Wi-Fi exitoso: Color RGB({r},{g},{b}) | Posición cuadrante: {cuadrante}")
            # Guardamos el registro del último paquete enviado con éxito
            ultimo_envio_tiempo = ahora
            ultimo_r, ultimo_g, ultimo_b, ultima_pos = r, g, b, cuadrante
    except requests.exceptions.RequestException:
        # Falla silenciosa para evitar romper el flujo principal si el Wi-Fi tiene lag repentino
        pass


class Metricas:
    def __init__(self):
        self.total          = 0
        self.con_persona    = 0
        self.confianzas     = []
        self.tiempos        = []
        self.hz_hist        = []
        self.colores_torso  = []
        self.colores_pierna = []
        self.inicio         = time.time()

    def registrar(self, detectado, conf, ms, hz, c_torso, c_pierna):
        self.total += 1
        if detectado:
            self.con_persona += 1
            self.confianzas.append(conf)
            if c_torso  and c_torso  != "Desconocido": self.colores_torso.append(c_torso)
            if c_pierna and c_pierna != "Desconocido": self.colores_pierna.append(c_pierna)
        self.tiempos.append(ms)
        self.hz_hist.append(hz)

    def guardar(self):
        if self.total == 0: return
        fps = 1000 / np.mean(self.tiempos) if self.tiempos else 0
        ct  = max(set(self.colores_torso),  key=self.colores_torso.count)  if self.colores_torso  else "—"
        cp  = max(set(self.colores_pierna), key=self.colores_pierna.count) if self.colores_pierna else "—"

        datos = [
            ("frames_totales",          self.total),
            ("frames_con_persona",       self.con_persona),
            ("tasa_deteccion_%",         f"{self.con_persona/self.total*100:.1f}"),
            ("confianza_promedio",       f"{np.mean(self.confianzas):.3f}" if self.confianzas else 0),
            ("fps_promedio",             f"{fps:.1f}"),
            ("latencia_ms_promedio",     f"{np.mean(self.tiempos):.1f}"),
            ("hz_movimiento_promedio",   f"{np.mean(self.hz_hist):.2f}"),
            ("color_torso_frecuente",    ct),
            ("color_piernas_frecuente",  cp),
            ("duracion_segundos",        f"{time.time()-self.inicio:.1f}"),
        ]

        print("\n" + "="*50)
        print("  Métricas Finales Integradas")
        print("="*50)
        for k, v in datos:
            print(f"  {k:<35} {v}")
        print("="*50)

        if GUARDAR_METRICAS:
            with open(ARCHIVO_METRICAS, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["Metrica", "Valor"])
                for k, v in datos: w.writerow([k, v])


metricas_global = None

def salir(sig=None, frame=None):
    print("\nCerrando aplicación y guardando métricas...")
    if metricas_global:
        metricas_global.guardar()
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, salir)


def main():
    global metricas_global, alternador_color

    print("Cargando modelo YOLO...")
    model = YOLO(YOLO_MODEL)
    print("Detección y emparejamiento Wi-Fi activados.")

    metricas = Metricas()
    metricas_global = metricas

    centroide_anterior = None
    timestamps_mov     = deque()
    historial_dist     = deque(maxlen=10)

    if USAR_WEBCAM:
        cap = None
        for idx in [0, 1, 2]:
            c = cv2.VideoCapture(idx)
            if c.isOpened():
                ret, frame = c.read()
                if ret and frame is not None:
                    cap = c
                    print(f"Cámara local encontrada en el índice: {idx}")
                    break
                c.release()
        if cap is None:
            print("No se encontró ninguna cámara disponible.")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    else:
        cap = None
        print(f"Capturando streaming de video desde: {url_capture}")

    panel_colores = mostrar_colores_dominantes("—", 0, "—", 0, 0.0)

    while True:
        t0 = time.time()

        if USAR_WEBCAM:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue
        else:
            frame = get_frame_esp32()
            if frame is None:
                time.sleep(0.1)
                continue

        H, W = frame.shape[:2]

        results = model(frame, classes=[CLASE_PERSONA], conf=CONFIANZA_MIN, verbose=False)

        detectado    = False
        mejor_box    = None
        mejor_conf   = 0.0
        color_torso  = "Desconocido"
        color_pierna = "Desconocido"
        hue_torso    = 0
        hue_pierna   = 0
        cx, cy       = W // 2, H // 2
        hz           = 0.0
        cuadrante    = 3  # Cuadrante por defecto (centro)

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf > mejor_conf:
                    mejor_conf = conf
                    mejor_box  = box.xyxy[0].cpu().numpy().astype(int)

        if mejor_box is not None:
            detectado = True
            x1, y1, x2, y2 = mejor_box
            x1=max(0, x1); y1=max(0, y1); x2=min(W, x2); y2=min(H, y2)

            alto = y2 - y1

            # Segmentación de zonas corporales
            yt1 = y1 + int(alto * 0.30)
            yt2 = y1 + int(alto * 0.55)
            roi_torso = frame[yt1:yt2, x1:x2]

            yp1 = y1 + int(alto * 0.60)
            yp2 = y1 + int(alto * 0.90)
            roi_pierna = frame[yp1:yp2, x1:x2]

            color_torso,  hue_torso  = detectar_color_dominante(roi_torso)
            color_pierna, hue_pierna = detectar_color_dominante(roi_pierna)

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # --- CÁLCULO DE LA POSICIÓN (CUADRANTES 1 A 5) ---
            # Dividimos el ancho de pantalla W en 5 segmentos verticales
            # Multiplicamos por 5 y sumamos 1 para obtener valores enteros del 1 al 5
            cuadrante = int((cx / W) * 5) + 1
            cuadrante = max(1, min(5, cuadrante)) # Acotación de seguridad

            # Análisis cinemático (Velocidad de movimiento en Hz)
            ahora = time.time()
            if centroide_anterior is not None:
                distancia = np.sqrt((cx - centroide_anterior[0])**2 + (cy - centroide_anterior[1])**2)
                historial_dist.append(distancia)
                if np.mean(historial_dist) > UMBRAL_MOV:
                    timestamps_mov.append(ahora)

            while timestamps_mov and (ahora - timestamps_mov[0]) > VENTANA_HZ:
                timestamps_mov.popleft()

            hz = len(timestamps_mov) / VENTANA_HZ
            centroide_anterior = (cx, cy)

            # --- SELECTOR Y CONVERTIDOR DE COLOR PARA ENVÍO ALTERNADO ---
            if alternador_color:
                bgr_enviar = color_a_bgr(color_torso, hue_torso)
            else:
                bgr_enviar = color_a_bgr(color_pierna, hue_pierna)
            
            # Alternamos el flag para el siguiente bucle
            alternador_color = not alternador_color

            # Envío de comandos RGB y Cuadrante de posición a la ESP32-CAM
            mandar_datos_a_esp32(int(bgr_enviar[2]), int(bgr_enviar[1]), int(bgr_enviar[0]), cuadrante)

            # Elementos gráficos sobre el visor de video
            col_box = (0,255,0) if hz<1 else (0,165,255) if hz<3.5 else (0,0,255)
            cv2.rectangle(frame, (x1,y1), (x2,y2), col_box, 2)
            cv2.rectangle(frame, (x1,yt1), (x2,yt2), (255,200,0), 1)
            cv2.rectangle(frame, (x1,yp1), (x2,yp2), (0,255,200), 1)
            cv2.putText(frame, f"Conf: {mejor_conf:.2f} | Pos: C{cuadrante}", (x1, y1-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col_box, 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            panel_colores = mostrar_colores_dominantes(color_torso, hue_torso, color_pierna, hue_pierna, hz)

        # Dibujamos las líneas guía divisoras de los 5 cuadrantes en pantalla
        for i in range(1, 5):
            linea_x = int(W * (i / 5))
            cv2.line(frame, (linea_x, 0), (linea_x, H), (100, 100, 100), 1, cv2.LINE_AA)

        estado = "quieto" if hz<0.5 else "caminando" if hz<3.5 else "corriendo"
        ms = (time.time() - t0) * 1000
        fps_act = 1000 / ms if ms > 0 else 0

        cv2.putText(frame, f"Torso: {color_torso}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(frame, f"Pierna: {color_pierna}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(frame, f"Hz: {hz:.1f} | {estado}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(frame, f"FPS: {fps_act:.1f}", (W-110, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200,200,200), 2)

        if not detectado:
            cv2.putText(frame, "No se detectan personas", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        metricas.registrar(detectado, mejor_conf, ms, hz, color_torso, color_pierna)

        cv2.imshow("ESP32 Tracking v2.5", frame)
        cv2.imshow("Colores Dominantes", panel_colores)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()