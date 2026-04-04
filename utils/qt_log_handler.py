import logging
from datetime import datetime
from PySide6.QtCore import QObject, Signal

class QtLogHandler(logging.Handler, QObject):
    """
    Handler de logging personalizado que intercepta registros y los emite 
    como señales de Qt. Asegura que los logs generados en hilos secundarios 
    lleguen de forma segura a la GUI.
    """
    
    # Señal que coincide exactamente con los argumentos de LogViewer.append_log
    # Firma: timestamp (str), level (str), module (str), message (str)
    log_emitted = Signal(str, str, str, str)

    def __init__(self):
        logging.Handler.__init__(self)
        QObject.__init__(self)

    def emit(self, record: logging.LogRecord) -> None:
        """
        Método invocado automáticamente por el módulo logging cada vez que 
        se hace un logger.info(), logger.error(), etc.
        """
        try:
            # 1. Formatear el timestamp
            timestamp = datetime.fromtimestamp(record.created).strftime('%H:%M:%S.%f')[:-3]
            
            # 2. Extraer nivel y módulo
            level = record.levelname
            module = record.name  # Ej: "STM32_Controller.Orquestador"
            
            # 3. Formatear el mensaje (sin prefijos, solo el texto)
            message = self.format(record)
            
            # 4. Emitir la señal hacia el hilo de la GUI
            self.log_emitted.emit(timestamp, level, module, message)
            
        except Exception:
            self.handleError(record)