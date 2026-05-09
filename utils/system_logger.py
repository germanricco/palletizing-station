import logging
import os
from datetime import datetime

class SystemLogger:
    """
    Configurador central del sistema de logging.
    Se instancia una única vez al arrancar el programa para establecer 
    las reglas globales de escritura a disco.
    """
    def __init__(self, log_dir: str = "logs", log_level: int = logging.INFO):
        os.makedirs(log_dir, exist_ok=True)
        
        # Obtenemos el logger base de nuestro dominio
        self.base_logger = logging.getLogger("STM32_Controller")
        # Aplicamos el nivel global parametrizado
        self.base_logger.setLevel(log_level)
        
        # Evitar duplicidad de manejadores si se llamara por error más de una vez
        if not self.base_logger.handlers:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(log_dir, f"ejecucion_{timestamp}.log")
            
            # --- MANEJADOR DE ARCHIVO (Para auditoría y debug post-mortem) ---
            file_handler = logging.FileHandler(filename, encoding='utf-8')
            file_handler.setLevel(log_level)
            
            formatter = logging.Formatter('%(asctime)s - [%(name)s] - [%(levelname)s] - %(message)s')
            file_handler.setFormatter(formatter)
            
            self.base_logger.addHandler(file_handler)