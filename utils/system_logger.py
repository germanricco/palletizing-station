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
        
        # Obtenemos el logger RAÍZ de nuestra aplicación
        self.base_logger = logging.getLogger("STM32_Controller")
        # Aplicamos el nivel global parametrizado
        self.base_logger.setLevel(log_level)
        
        # Evitar duplicidad de manejadores si se llamara por error más de una vez
        if not self.base_logger.handlers:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(log_dir, f"ejecucion_{timestamp}.log")
            
            file_handler = logging.FileHandler(filename, encoding='utf-8')
            # El archivo guardará mensajes a partir del nivel indicado
            file_handler.setLevel(log_level)
            
            # ATENCIÓN AQUÍ: %(name)s es la variable que imprimirá el submódulo dinámicamente
            formatter = logging.Formatter('%(asctime)s - [%(name)s] - [%(levelname)s] - %(message)s')
            file_handler.setFormatter(formatter)
            
            # Opcional: También lo mandamos a la consola para facilitar el desarrollo
            console_handler = logging.StreamHandler()
            # La consola también respeta el nivel indicado
            console_handler.setLevel(log_level)
            console_handler.setFormatter(formatter)

            self.base_logger.addHandler(file_handler)
            self.base_logger.addHandler(console_handler)