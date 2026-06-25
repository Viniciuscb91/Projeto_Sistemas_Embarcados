import serial
import time
import json
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# --- CONFIGURAÇÕES ---
# porta do ESP32 
PORTA_SERIAL = 'COM5' 
BAUD_RATE = 115200

# Dados do InfluxDB
INFLUX_URL = "http://localhost:8086"
TOKEN = "CbLL53_Yfks-DapDjDTXqgCuxt7nMDZn--4UYLD35InCO4g-DTiOkntJjjhdY0uT7RtSfvCORBYjF5Kx4fza0g=="
ORG = "Embarcados"
BUCKET = "ProjetoFinal"
# ---------------------

# Inicializa o cliente do InfluxDB
client = InfluxDBClient(url=INFLUX_URL, token=TOKEN, org=ORG)
write_api = client.write_api(write_options=SYNCHRONOUS)

print(f"Tentando conectar na porta {PORTA_SERIAL}...")
try:
    ser = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=1)
    print("Conectado com sucesso à PCB!")
except Exception as e:
    print(f"Erro ao abrir a porta serial: {e}")
    print("Verifique se o Monitor Serial do VSCode está FECHADO, pois ele bloqueia a porta.")
    exit()

time.sleep(2) # Aguarda o ESP32 reiniciar após a conexão estável

while True:
    if ser.in_waiting > 0:
        try:
            # Lê a linha enviada pelo ESP32
            linha = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # Se a linha começar com a chave do JSON, processa
            if linha.startswith('{') and linha.endswith('}'):
                dados = json.loads(linha)
                
                # Separa as variáveis vindas da PCB
                pitch = float(dados['pitch'])
                roll = float(dados['roll'])
                joy_x = int(dados['joy_x'])
                joy_y = int(dados['joy_y'])
                btn = int(dados['btn'])
                srv_x = int(dados['srv_x'])
                srv_y = int(dados['srv_y'])

                print(f"PCB -> Pitch: {pitch}° | Roll: {roll}° | Joy_X: {joy_x} | Joy_Y: {joy_y} | Servo_X: {srv_x} | Servo_Y: {srv_y}")

                # Estrutura o ponto incluindo absolutamente todos os dados no Banco
                ponto = Point("dados_mesa") \
                    .field("pitch", pitch) \
                    .field("roll", roll) \
                    .field("joy_x", joy_x) \
                    .field("joy_y", joy_y) \
                    .field("btn", btn) \
                    .field("servo_x", srv_x) \
                    .field("servo_y", srv_y)

                # Escreve de fato no InfluxDB
                write_api.write(bucket=BUCKET, org=ORG, record=ponto)
                
        except json.JSONDecodeError:
            pass # Ignora linhas corrompidas de inicialização
        except Exception as e:
            print(f"Erro ao processar dados: {e}")
            
    time.sleep(0.01) # Pequena pausa para aliviar o processador