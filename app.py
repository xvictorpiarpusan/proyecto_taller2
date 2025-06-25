# app.py
import os, time, threading, asyncio
import asyncio
from flask import Flask, render_template, request, jsonify
from bleak import BleakClient, BleakScanner
import time

app = Flask(__name__)

# UUIDs típicos HM‑10 (servicio UART y característica UART)
UART_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
UART_CHAR_UUID    = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Si ya conoces la dirección MAC de tu HM‑10, ponla aquí; si no, la buscaremos
BLE_ADDRESS = os.getenv("HM10_ADDRESS", None)
BLE_NAME    = os.getenv("HM10_NAME", "JDY-08")


robot_readings = []
robot_logs = []   # <--- aquí acumularemos los mensajes de Arduino

async def ble_send_coordinates(address: str, coords: list):
    """
    Se conecta al HM‑10, envía coords, recibe notificaciones BLE en fragmentos,
    recompone líneas completas, parsea lecturas de posición y las guarda.
    """
    async with BleakClient(address) as client:
        if not client.is_connected:
            raise RuntimeError(f"No se pudo conectar a {address}")
        robot_logs.append(f"🔗 Conectado a {address}")
        print(f"🔗 Conectado a {address}")

        # Buffer donde concatenar fragmentos hasta encontrar '\n'
        text_buffer = ""

        # Handler: recibe fragmentos, reconstruye líneas y parsea lecturas
        def notification_handler(_, data):
            nonlocal text_buffer
            chunk = data.decode(errors='ignore')
            text_buffer += chunk
            # Mientras haya saltos de línea en el buffer:
            while "\n" in text_buffer:
                line, text_buffer = text_buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                # Guardamos TODO log de Arduino
                robot_logs.append(f"📲 Arduino: {line}")
                print("📲 Arduino:", line)

                # Intentar extraer x, y, theta de "Posición global: (x, y) - Ángulo: θ°"
                # Intentar extraer x, y, theta de "Posición: (x, y)  θ=123.4°"
            try:
                # 1) Quitamos prefijos y sufijos
                body = line.replace("Posición: (", "").replace("°", "")
                # body ahora es "x, y)  θ=123.4"
                # 2) Separamos coordenadas y ángulo
                coord_part, theta_part = body.split(")  θ=")
                # coord_part == "x, y"
                # theta_part == "123.4"
                x_str, y_str = coord_part.split(", ")
                x_val = float(x_str)
                y_val = float(y_str)
                theta = float(theta_part)
                robot_readings.append({
                    'x': x_val,
                    'y': y_val,
                    'theta': theta,
                    't': time.time()
                })
            except Exception:
                pass

        # Suscribirse a notificaciones BLE
        await client.start_notify(UART_CHAR_UUID, notification_handler)
        robot_logs.append("🛎 Suscripción a notificaciones iniciada")
        print("🛎 Suscripción a notificaciones iniciada")

        # Enviar cada coordenada al Arduino
        for line in coords:
            msg = f"{line}\n"
            robot_logs.append(f"➡️ Enviando: {line}")
            print("➡️ Enviando:", msg.strip())
            await client.write_gatt_char(UART_CHAR_UUID, msg.encode())
            await asyncio.sleep(0.5)

        # Esperar para que el Arduino imprima todo el recorrido
        TOTAL_WAIT = 40  # segundos (ajusta según tu rutina)
        robot_logs.append(f"⏳ Esperando {TOTAL_WAIT}s a que Arduino termine de imprimir...")
        print(f"⏳ Esperando {TOTAL_WAIT}s a que Arduino termine de imprimir...")
        await asyncio.sleep(TOTAL_WAIT)

        # Detener notificaciones y desconectar
        await client.stop_notify(UART_CHAR_UUID)
        robot_logs.append("🛑 Notificaciones detenidas, desconectando")
        print("🛑 Notificaciones detenidas, desconectando")

    # Una vez desconectado, mostrar todas las lecturas acumuladas
    print("\n📊 Lecturas acumuladas:")
    for r in robot_readings:
        print(f"  → x={r['x']:.2f}, y={r['y']:.2f}, θ={r['theta']:.2f}°, t={r['t']:.1f}")

def find_ble_address():
    """Si no tenemos MAC fija, escanea y busca por nombre HM‑10."""
    devices = asyncio.run(BleakScanner.discover(timeout=4.0))
    for dev in devices:
        if dev.name and BLE_NAME in dev.name:
            return dev.address
    raise RuntimeError(f"No se encontró dispositivo BLE '{BLE_NAME}'")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send', methods=['POST'])
def send():
    data = request.get_json(force=True)
    # Extraemos directamente A, B, C y heading
    A = data['A']       # { "x": X0, "y": Y0 }
    B = data['B']       # { "x": Xb, "y": Yb }
    C = data['C']       # { "x": Xc, "y": Yc }
    heading = data.get('heading0', 0)  # ángulo inicial

    # Preparamos las líneas que enviaremos tal cual al Arduino
    coords = [
        f"{A['x']} {A['y']}",
        f"{B['x']} {B['y']}",
        f"{C['x']} {C['y']}",
        f"{heading}"
    ]

    # Determina la dirección del HM‑10
    address = BLE_ADDRESS or find_ble_address()
    def task():
        try:
            asyncio.run(ble_send_coordinates(address, coords))
        except Exception as e:
            print("❌ Error BLE (en hilo):", e)

    threading.Thread(target=task, daemon=True).start()

    # Respuesta inmediata
    return jsonify({ 'status': 'started', 'sent': coords }), 202
    

@app.route('/readings', methods=['GET'])
def get_readings():
    # Solo devolvemos los pares x,y de robot_readings
    simple = [{'x': r['x'], 'y': r['y']} for r in robot_readings]
    return jsonify(simple), 200

@app.route('/logs', methods=['GET'])
def get_logs():
    """Devuelve y limpia los logs pendientes."""
    global robot_logs
    out = robot_logs
    robot_logs = []
    return jsonify(out), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
