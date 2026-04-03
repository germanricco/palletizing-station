"""
Módulo de Configuración Global.
Contiene todas las definiciones estáticas provenientes del Design Handoff.
"""

from dataclasses import dataclass

@dataclass(frozen=True)
class Colors:
    """
    Paleta de colores basada en WCAG 2.1 AA para entornos de baja iluminación.
    """
    BACKGROUND: str = "#121212"
    SURFACE: str = "#1E1E1E"
    BORDERS: str = "#333333"
    TEXT_HIGH: str = "#FFFFFF"
    TEXT_MED: str = "#A0A0A0"
    
    # Semántica Crítica
    SEMANTIC_ESTOP: str = "#FF3B30"
    SEMANTIC_AUTO: str = "#34C759"
    SEMANTIC_MANUAL: str = "#00F0FF"
    SEMANTIC_RECORD: str = "#FF9500"
    SEMANTIC_OFFLINE: str = "#FFB300"

@dataclass(frozen=True)
class Typography:
    """
    Definiciones tipográficas estandarizadas para UI y Telemetría.
    """
    FAMILY_UI: str = "Inter, Roboto, sans-serif"
    FAMILY_DATA: str = "'JetBrains Mono', 'Fira Code', monospace"
    
    SIZE_BTN_LABEL: str = "14px"
    SIZE_PANEL_TITLE: str = "16px"
    SIZE_HUD_CONSOLE: str = "14px"
    SIZE_LOGS: str = "12px"
    SIZE_ESTOP: str = "18px"
    
    WEIGHT_REGULAR: str = "400"
    WEIGHT_SEMIBOLD: str = "600"
    WEIGHT_BOLD: str = "bold"

@dataclass(frozen=True)
class Spacing:
    """
    Sistema de espaciado basado en grid de 8px.
    """
    XS: str = "8px"
    SM: str = "16px"
    MD: str = "24px"
    XL: str = "40px"

@dataclass(frozen=True)
class LayoutDimensions:
    """
    Dimensiones estructurales relativas al Viewport.
    Nota: PySide6 requerirá cálculos dinámicos basados en la resolución de pantalla
    para simular vw/vh, estas constantes actúan como factor de proporción.
    """
    LEFT_COL_WIDTH_PCT: float = 0.75
    RIGHT_COL_WIDTH_PCT: float = 0.25
    
    TWIN_HEIGHT_PCT: float = 0.70
    TERMINAL_HEIGHT_PCT: float = 0.30
    
    SAFETY_HEIGHT_PCT: float = 0.15
    CONTROLS_HEIGHT_PCT: float = 0.85

class AppConfig:
    """
    Punto de acceso centralizado a las configuraciones inmutables de la UI.
    """
    COLORS = Colors()
    TYPOGRAPHY = Typography()
    SPACING = Spacing()
    LAYOUT = LayoutDimensions()