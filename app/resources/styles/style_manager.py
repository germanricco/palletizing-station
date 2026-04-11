"""
Gestor dinámico de hojas de estilo (QSS) para PySide6.
Permite el uso de variables de diseño procesando plantillas QSS.
"""
import os
from typing import Dict
from PySide6.QtWidgets import QApplication
from app.core.constants import AppConfig

class StyleManager:
    """
    Clase responsable de cargar, procesar y aplicar hojas de estilo QSS.
    Separa la capa de presentacion visual de la logica de los componentes.
    """
    @classmethod
    def apply_global_styles(cls, app: QApplication) -> None:
        """
        Método de entrada principal. Aplica la paleta nativa y el QSS global.
        """
        cls.apply_base_palette(app)
        
        # Obtenemos el QSS centralizado y lo aplicamos a toda la aplicación
        global_qss = cls._get_global_stylesheet()
        app.setStyleSheet(global_qss)

    @staticmethod
    def _get_global_stylesheet() -> str:
        """
        Concentra todas las reglas QSS estáticas que estaban hardcodeadas
        en las clases Python. Utiliza f-strings para inyectar dinámicamente
        la configuración central de AppConfig.
        """
        return f"""
        /* =========================================
           1. ESTRUCTURA Y LAYOUT MAESTRO
           ========================================= */
        QMainWindow {{
            background-color: {AppConfig.COLORS.BACKGROUND};
        }}
        
        QFrame#RightColumn {{
            border-left: 1px solid {AppConfig.COLORS.BORDERS};
        }}
        
        QFrame#BottomLeftContainer {{
            border-top: 1px solid {AppConfig.COLORS.BORDERS};
        }}

        TerminalUART {{
            border-top: 1px solid {AppConfig.COLORS.BORDERS};
        }}

        /* =========================================
           2. PANELES PRINCIPALES
           ========================================= */
        ControlPanel {{
            background-color: {AppConfig.COLORS.BACKGROUND};
            border-top: 1px solid {AppConfig.COLORS.BORDERS};
        }}
        
        SafetyPanelWidget {{
            background-color: {AppConfig.COLORS.SURFACE};
        }}

        /* =========================================
           3. GEMELO DIGITAL Y HUD
           ========================================= */
        DigitalTwinCanvas {{
            background-color: {AppConfig.COLORS.BACKGROUND};
        }}

        HUDPanel {{
            background-color: rgba(20, 20, 20, 204);
            border: 1px solid {AppConfig.COLORS.BORDERS};
            border-radius: 6px;
        }}

        /* Barra de Progreso Inferior */
        QProgressBar {{
            background-color: #1A1A1A;
            border: none;
        }}
        QProgressBar::chunk {{
            background-color: {AppConfig.COLORS.SEMANTIC_MANUAL}; 
        }}

        /* =========================================
           4. TARJETAS DE TELEOPERACIÓN (JOGGING)
           ========================================= */
        JoggingCard {{
            background-color: #1A1A1A;
            border: 1px solid {AppConfig.COLORS.BORDERS};
            border-radius: 8px;
        }}

        JoggingCard QDoubleSpinBox {{
            background-color: #0A0A0A;
            color: {AppConfig.COLORS.TEXT_HIGH};
            border: 1px solid {AppConfig.COLORS.BORDERS};
            border-radius: 4px;
            font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};
            font-size: 14px;
        }}

        JoggingCard QPushButton {{
            background-color: #2A2A2A;
            color: {AppConfig.COLORS.TEXT_HIGH};
            border-radius: 4px;
            font-size: 18px;
            font-weight: bold;
            border: none;
        }}

        JoggingCard QPushButton:hover {{
            background-color: {AppConfig.COLORS.BORDERS};
        }}

        /* =========================================
           5. DIÁLOGOS Y POPUPS (Save Dialog)
           ========================================= */
        SaveTrajectoryDialog {{
            background-color: {AppConfig.COLORS.SURFACE};
            border: 1px solid {AppConfig.COLORS.BORDERS};
        }}

        SaveTrajectoryDialog QLabel {{
            color: {AppConfig.COLORS.TEXT_HIGH};
            font-family: {AppConfig.TYPOGRAPHY.FAMILY_UI};
            font-size: 14px;
        }}

        SaveTrajectoryDialog QLineEdit {{
            background-color: #0A0A0A;
            border: 1px solid {AppConfig.COLORS.BORDERS};
            border-radius: 4px;
            padding: 8px;
            color: {AppConfig.COLORS.SEMANTIC_MANUAL};
            font-family: {AppConfig.TYPOGRAPHY.FAMILY_DATA};
        }}

        SaveTrajectoryDialog QLineEdit:focus {{
            border: 1px solid {AppConfig.COLORS.SEMANTIC_MANUAL};
        }}
        """
    
    @classmethod
    def apply_base_palette(cls, app: QApplication) -> None:
        """
        Alternativa por código para forzar colores a nivel de QPalette de la app.
        Útil para componentes nativos (scrollbars, menus) que no responden bien a QSS puro.
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

    # ==== METODOS DE PLANTILLAS EXTERNAS ====
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
            compiled_qss = file.read()

        context = cls._get_context_dict()

        # Reemplazar los placeholders {{VAR_NAME}} en el texto QSS
        for key, value in context.items():
            placeholder = f"{{{{{key}}}}}"
            compiled_qss = compiled_qss.replace(placeholder, value)

        # Configurar estilos base nativos definidos en el Handoff
        app.setStyleSheet(compiled_qss)
