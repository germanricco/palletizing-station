import serial
import threading
import sys

# ==========================================
# CONFIGURACIÓN DEL HARDWARE
# ==========================================
PUERTO_SERIAL = '/dev/ttyUSB0' 
BAUD_RATE = 115200

def escuchar_puerto(ser):
    """
    Hilo en segundo plano (Daemon). 
    Lee la telemetría del STM32 de forma asíncrona sin bloquear el teclado.
    """
    while True:
        try:
            if ser.in_waiting > 0:
                # Leemos la línea, la decodificamos a string y quitamos espacios/saltos extra
                linea = ser.readline().decode('utf-8', errors='replace').strip()
                if linea:
                    # Imprimimos la telemetría borrando temporalmente el prompt de entrada
                    sys.stdout.write(f"\r[STM32] {linea}\n")
                    sys.stdout.write("Comando > ")
                    sys.stdout.flush()
        except serial.SerialException:
            print("\n[Error] Conexión serial interrumpida.")
            break
        except Exception as e:
            print(f"\n[Error de lectura] {e}")
            break

def main():
    print(f"Iniciando terminal bidireccional en {PUERTO_SERIAL} a {BAUD_RATE} baudios...")
    
    try:
        ser = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error abriendo el puerto {PUERTO_SERIAL}: Verifique que esté conectado y no esté en uso.")
        sys.exit(1)

    # Iniciamos el hilo de lectura asíncrona
    hilo_lectura = threading.Thread(target=escuchar_puerto, args=(ser,), daemon=True)
    hilo_lectura.start()

    print("Escriba su comando (ej. 'SE1 V10') y presione Enter. Escriba 'salir' para terminar.\n")

    # Bucle principal: Solo se encarga de esperar comandos del usuario (Bloqueante)
    try:
        while True:
            comando = input("Comando > ")
            
            if comando.lower() == 'salir':
                break
            
            if comando.strip():
                # Enviamos el comando sumando obligatoriamente el delimitador \n
                trama_tx = f"{comando}\n".encode('utf-8')
                ser.write(trama_tx)
                
    except KeyboardInterrupt:
        print("\nCierre forzado detectado.")
    finally:
        ser.close()
        print("Puerto serial cerrado.")

if __name__ == "__main__":
    main()