"""
Gestor dinámico de hojas de estilo (QSS) para PySide6.
Permite el uso de variables de diseño procesando plantillas QSS.
"""

import os
from typing import Dict, Any
from PySide6.QtWidgets import QApplication
from core.constants import AppConfig

class StyleManager:
    """
    Clase responsable de cargar, procesar y aplicar hojas de estilo QSS.
    
    Extrae los atributos de las DataClasses de configuración y reemplaza
    los marcadores (ej. {{COLORS_BACKGROUND}}) en los archivos QSS puros.
    """

    @staticmethod
    def _get_context_dict() -> Dict[str, str]:
        """
        Genera un diccionario de contexto plano a partir de la configuración.
        
        Returns:
            Dict[str, str]: Diccionario con formato {'COLORS_BACKGROUND': '#121212', ...}
        """
        context: Dict[str, str] = {}
        
        # Mapeo de sub-configuraciones para el motor de plantillas
        configs = {
            "COLORS": AppConfig.COLORS,
            "TYPOGRAPHY": AppConfig.TYPOGRAPHY,
            "SPACING": AppConfig.SPACING
        }

        for prefix, dataclass_obj in configs.items():
            for key, value in dataclass_obj.__dict__.items():
                if not key.startswith("__"):
                    # Construye el key esperado en el QSS, ej: COLORS_SEMANTIC_ESTOP
                    template_key = f"{prefix}_{key}"
                    context[template_key] = str(value)
                    
        return context

    @classmethod
    def load_and_apply_qss(cls, app: QApplication, qss_filepath: str) -> None:
        """
        Lee el archivo QSS, inyecta las constantes de diseño y lo aplica a la app.
        
        Args:
            app (QApplication): Instancia principal de la aplicación PySide6.
            qss_filepath (str): Ruta absoluta o relativa al archivo .qss base.
            
        Raises:
            FileNotFoundError: Si la ruta al archivo QSS no existe.
        """
        if not os.path.exists(qss_filepath):
            raise FileNotFoundError(f"Archivo de estilos no encontrado: {qss_filepath}")

        with open(qss_filepath, "r", encoding="utf-8") as file:
            raw_qss = file.read()

        context = cls._get_context_dict()
        compiled_qss = raw_qss

        # Reemplazar los placeholders {{VAR_NAME}} en el texto QSS
        for key, value in context.items():
            placeholder = f"{{{{{key}}}}}"
            compiled_qss = compiled_qss.replace(placeholder, value)

        # Configurar estilos base nativos definidos en el Handoff
        app.setStyleSheet(compiled_qss)

    @classmethod
    def apply_base_palette(cls, app: QApplication) -> None:
        """
        Alternativa por código para forzar colores a nivel de QPalette de la app.
        Útil para componentes que no responden bien a QSS puro.
        
        Args:
            app (QApplication): Instancia principal de la aplicación PySide6.
        """
        from PySide6.QtGui import QPalette, QColor
        
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor(AppConfig.COLORS.BACKGROUND))
        palette.setColor(QPalette.ColorRole.WindowText, QColor(AppConfig.COLORS.TEXT_HIGH))
        palette.setColor(QPalette.ColorRole.Base, QColor(AppConfig.COLORS.SURFACE))
        palette.setColor(QPalette.ColorRole.Text, QColor(AppConfig.COLORS.TEXT_HIGH))
        palette.setColor(QPalette.ColorRole.Button, QColor(AppConfig.COLORS.SURFACE))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor(AppConfig.COLORS.TEXT_HIGH))
        
        app.setPalette(palette)