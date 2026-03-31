import sys
import os
import time
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
    
from communication.fake_uart import FakeUARTCommunicator
from control.trajectory_streamer import TrajectoryStreamer

def generar_trayectoria_dummy(num_puntos: int) -> np.ndarray:
    """Genera una trayectoria senoidal de prueba de Nx3."""
    t = np.linspace(0, 10, num_puntos)
    q1 = np.sin(t) * 90      # Articulación base
    q2 = np.cos(t) * 45      # Hombro
    q3 = np.sin(t * 0.5) * 30 # Codos
    return np.column_stack((q1, q2, q3))

def main():
    print("--- Iniciando Prueba Unitaria de Arquitectura de Control ---")
    
    # 1. Instanciar Infraestructura (El Mock)
    stm32_mock = FakeUARTCommunicator(consumption_rate_hz=50, chunk_size=15)
    stm32_mock.connect()
    
    # 2. Instanciar Lógica de Dominio
    streamer = TrajectoryStreamer(comm=stm32_mock, chunk_size=15)
    
    # 3. Crear una matriz de trayectoria pesada (ej. 60 puntos = 4 chunks)
    trayectoria_numpy = generar_trayectoria_dummy(60)
    print(f"Trayectoria generada con {trayectoria_numpy.shape[0]} puntos.")
    
    # 4. Iniciar proceso no bloqueante
    print("\nHilo Principal: Dando orden de inicio...")
    streamer.start_streaming(trayectoria_numpy)
    
    print("Hilo Principal: Libre para renderizar GUI o leer otras entradas!\n")
    
    # Mantener el hilo principal vivo mientras la comunicación ocurre en background
    # En una app real, aquí estaría el loop de tu GUI (como Tkinter o PyQt)
    while streamer.state != "IDLE":
        time.sleep(0.1)
        
    print("\n--- Prueba Unitaria Finalizada con Éxito ---")
    stm32_mock.disconnect()

if __name__ == "__main__":
    main()