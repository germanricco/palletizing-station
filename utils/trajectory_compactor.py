"""
Compactacion de trayectorias grabadas (eliminacion de tiempos muertos).

Una trayectoria grabada es una matriz N x 4 [q1, q2, q3, a1] (q en rad, a1 = efector 0/1)
con una fila por trama T: (20 ms). En AUTO el firmware consume una fila cada 20 ms, asi que
la trayectoria esta indexada en tiempo (t_k = k * 20 ms): toda pausa grabada mientras el
operador piensa se reproduce tal cual.

Regla de compactacion:
  * Una fila es "quieta" si ninguna articulacion se aparto mas de eps de la fila de
    referencia (la ultima conservada por movimiento o por flanco de a1) y a1 no cambio.
    Las filas quietas se descartan. Comparar contra la referencia, y no contra la fila
    anterior, acota a eps el error de posicion aunque el movimiento sea muy lento.
    Los tramos quietos empiezan y terminan con velocidad nula (rampa de Manual), por eso al
    unirlos la posicion y la velocidad siguen siendo continuas.
  * Despues de cada flanco de a1 se garantiza una espera minima antes de mover:
    t_agarre (0->1, formacion del vacio) y t_suelta (1->0, despegue de la caja). Si la
    grabacion tenia mas, se recorta; si tenia menos, se rellena repitiendo la fila.

Efecto colateral: todo tramo mas lento que eps/dt se reproduce a ~eps/dt (las vacilaciones
del operador tambien se comprimen).

Ahorro: como solo se quitan filas con el brazo quieto, el movimiento (y su energia) no
cambia. Lo que se ahorra por ciclo es el tiempo quieto con el sistema energizado:
    E_ahorro = P_reposo * (T_original - T_compacta) + P_bomba * (Tb_original - Tb_compacta)
con T = duracion de la trayectoria y Tb = tiempo con la bomba encendida.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

DT_S = 0.020            # s por fila: periodo de T: y del metronomo de AUTO en el firmware
UMBRAL_A1 = 0.5         # a1 > umbral -> efector encendido (la GUI graba 0.0 / 1.0)

# eps por defecto: 0.04 grados <= 1 us de PWM en las tres articulaciones
# (hombro 25 us/grado, codo 14.8 us/grado, base 11.1 us/grado con el mapeo de Config.hpp).
EPS_DEG_DEFECTO = 0.04
T_AGARRE_DEFECTO = 2.0  # = fase de succion al 100 % del firmware (pasa a 50 % a los 2 s)
T_SUELTA_DEFECTO = 0.5

# Potencias medias para estimar la energia ahorrada [W]. None = todavia sin medir: el informe
# y el grafico muestran solo los tiempos. Medir con la fuente de banco:
#   POTENCIA_REPOSO_W: brazo quieto sosteniendo una pose (servos + logica + cinta frenada).
#   POTENCIA_BOMBA_W:  consumo ADICIONAL con la bomba encendida (promedio 100 % / 50 %).
POTENCIA_REPOSO_W: Optional[float] = None
POTENCIA_BOMBA_W: Optional[float] = None

SUFIJO_COMPACTA = "_compacta"
CARPETA_GRAFICOS = Path("ensayos") / "compactar"


@dataclass
class Flanco:
    """Flanco del efector en la trayectoria original."""
    t_s: float              # instante del flanco [s]
    encendido: bool         # True: 0->1 (agarre), False: 1->0 (suelta)
    pausa_grabada_s: float  # tiempo entre el flanco y el primer movimiento [s]


@dataclass
class InformeCompactacion:
    n_original: int
    n_compacta: int
    filas_relleno: int
    t_bomba_original_s: float
    t_bomba_compacta_s: float
    eps_deg: float
    t_agarre_s: float
    t_suelta_s: float
    flancos: List[Flanco] = field(default_factory=list)
    rth_deg: float = 0.0    # |inicio - fin| maximo de la trayectoria compacta [grados]

    @property
    def t_original_s(self) -> float:
        return self.n_original * DT_S

    @property
    def t_compacta_s(self) -> float:
        return self.n_compacta * DT_S

    @property
    def ahorro_s(self) -> float:
        return self.t_original_s - self.t_compacta_s

    @property
    def ahorro_pct(self) -> float:
        return 100.0 * (1.0 - self.n_compacta / self.n_original) if self.n_original else 0.0

    def energia_ahorrada_j(self, p_reposo_w: Optional[float] = POTENCIA_REPOSO_W,
                           p_bomba_w: Optional[float] = POTENCIA_BOMBA_W) -> Optional[float]:
        """Energia ahorrada por ciclo [J], o None si falta alguna potencia medida."""
        if p_reposo_w is None or p_bomba_w is None:
            return None
        return p_reposo_w * self.ahorro_s + p_bomba_w * (self.t_bomba_original_s - self.t_bomba_compacta_s)


def _filas_espera(t_s: float) -> int:
    """Filas quietas tras el flanco para que flanco -> primer movimiento dure t_s."""
    return max(0, round(t_s / DT_S) - 1)


def tiempo_bomba_s(traj: np.ndarray) -> float:
    """Tiempo con el efector encendido [s]."""
    return float(np.count_nonzero(traj[:, 3] > UMBRAL_A1)) * DT_S


def pausas_tras_flancos(traj: np.ndarray, eps_deg: float = EPS_DEG_DEFECTO) -> List[Flanco]:
    """Lista los flancos de a1 y la pausa grabada despues de cada uno."""
    q, on, eps = traj[:, :3], traj[:, 3] > UMBRAL_A1, np.deg2rad(eps_deg)
    flancos = []
    for k in np.flatnonzero(np.diff(on.astype(np.int8))) + 1:
        j = k + 1
        while j < len(traj) and on[j] == on[k] and np.max(np.abs(q[j] - q[k])) < eps:
            j += 1
        flancos.append(Flanco(k * DT_S, bool(on[k]), (j - k) * DT_S))
    return flancos


def compactar(traj: np.ndarray,
              eps_deg: float = EPS_DEG_DEFECTO,
              t_agarre_s: float = T_AGARRE_DEFECTO,
              t_suelta_s: float = T_SUELTA_DEFECTO) -> Tuple[np.ndarray, InformeCompactacion]:
    """
    Elimina los tiempos muertos de una trayectoria N x 4 [q1, q2, q3, a1].

    Retorna:
        (trayectoria compacta con el mismo dtype, informe)
    """
    if traj.ndim != 2 or traj.shape[1] != 4 or traj.shape[0] == 0:
        raise ValueError(f"Formato inesperado {traj.shape}: se espera N x 4 [q1, q2, q3, a1]")

    q = traj[:, :3].astype(np.float64)
    on = traj[:, 3] > UMBRAL_A1
    eps = np.deg2rad(eps_deg)
    n_agarre, n_suelta = _filas_espera(t_agarre_s), _filas_espera(t_suelta_s)

    idx = [0]       # indices (del original) de las filas conservadas
    ref = 0         # referencia de quietud (no se actualiza con las filas de la espera)
    espera = 0      # filas quietas que todavia exige el ultimo flanco
    relleno = 0
    for k in range(1, len(traj)):
        if on[k] != on[k - 1]:                          # flanco del efector
            idx.append(k)
            ref = k
            espera = n_agarre if on[k] else n_suelta
        elif np.max(np.abs(q[k] - q[ref])) < eps:       # quieta
            if espera > 0:
                idx.append(k)
                espera -= 1
        else:                                           # en movimiento
            if espera > 0:                              # pausa grabada mas corta que la minima
                idx += [idx[-1]] * espera
                relleno += espera
                espera = 0
            idx.append(k)
            ref = k
    if espera > 0:                                      # flanco al final de la trayectoria
        idx += [idx[-1]] * espera
        relleno += espera

    compacta = traj[np.asarray(idx)].astype(traj.dtype, copy=False)
    informe = InformeCompactacion(
        n_original=len(traj),
        n_compacta=len(compacta),
        filas_relleno=relleno,
        t_bomba_original_s=tiempo_bomba_s(traj),
        t_bomba_compacta_s=tiempo_bomba_s(compacta),
        eps_deg=eps_deg,
        t_agarre_s=t_agarre_s,
        t_suelta_s=t_suelta_s,
        flancos=pausas_tras_flancos(traj, eps_deg),
        rth_deg=float(np.rad2deg(np.max(np.abs(compacta[0, :3] - compacta[-1, :3])))),
    )
    return compacta, informe


# ==== GRAFICO COMPARATIVO ====================================================================
# Se usa la API orientada a objetos (Figure + FigureCanvasAgg), nunca pyplot: la GUI ya tiene
# matplotlib cargado con el backend Qt, y pyplot/matplotlib.use() cambiarian ese estado global.
# Asi el grafico se puede generar en un hilo aparte mientras la GUI sigue respondiendo.

_COLOR_ORIGINAL = "#eb6834"     # categorica 2 (paleta validada: CVD dE 24.7, contraste >= 3:1)
_COLOR_COMPACTA = "#2a78d6"     # categorica 1
_TINTA = "#0b0b0b"
_TINTA_2 = "#52514e"
_GRILLA = "#e4e3df"
_FONDO = "#fcfcfb"


def _num(x: float, dec: int = 1) -> str:
    """Numero con coma decimal."""
    return f"{x:.{dec}f}".replace(".", ",")


def graficar_comparacion(orig: np.ndarray, comp: np.ndarray, informe: InformeCompactacion,
                         ruta: Path, nombre: str,
                         p_reposo_w: Optional[float] = POTENCIA_REPOSO_W,
                         p_bomba_w: Optional[float] = POTENCIA_BOMBA_W) -> Path:
    """Guarda un PNG: resumen del ahorro + q1, q2, q3 y bomba, original contra compacta."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FuncFormatter

    fig = Figure(figsize=(10, 9.4), dpi=120, facecolor=_FONDO)
    FigureCanvasAgg(fig)
    gs = fig.add_gridspec(5, 1, height_ratios=[1.25, 2, 2, 2, 1.1], hspace=0.28,
                          left=0.09, right=0.97, top=0.875, bottom=0.11)

    # --- Titulo (el hallazgo) y resumen ---
    fig.text(0.09, 0.962, f"{nombre}: la versión compacta dura {_num(informe.t_compacta_s)} s "
             f"en vez de {_num(informe.t_original_s)} s (−{_num(informe.ahorro_pct, 0)} %)",
             fontsize=14, fontweight="bold", color=_TINTA, ha="left", va="center")

    resumen = fig.add_subplot(gs[0])
    resumen.axis("off")
    bloques = [
        ("Duración de la trayectoria", f"{_num(informe.t_original_s)} → {_num(informe.t_compacta_s)} s"),
        ("Tiempo con la bomba encendida",
         f"{_num(informe.t_bomba_original_s)} → {_num(informe.t_bomba_compacta_s)} s"),
    ]
    energia = informe.energia_ahorrada_j(p_reposo_w, p_bomba_w)
    if energia is not None:
        bloques.append(("Energía ahorrada por caja", f"≈ {_num(energia, 0)} J"))
    else:
        bloques.append(("Tiempo quieto eliminado por caja", f"{_num(informe.ahorro_s)} s"))
    for i, (etiqueta, valor) in enumerate(bloques):
        x = i / len(bloques)
        resumen.text(x, 0.78, etiqueta, fontsize=10, color=_TINTA_2, transform=resumen.transAxes)
        resumen.text(x, 0.30, valor, fontsize=17, fontweight="bold", color=_TINTA,
                     transform=resumen.transAxes)

    # --- Series (angulos cinematicos en grados; bomba como escalon) ---
    t_o = np.arange(len(orig)) * DT_S
    t_c = np.arange(len(comp)) * DT_S
    x_max = t_o[-1] * 1.12 if len(t_o) > 1 else 1.0
    coma = FuncFormatter(lambda v, _: f"{v:g}".replace(".", ","))
    nombres = ("q1 base [°]", "q2 hombro [°]", "q3 codo [°]", "Bomba")
    ejes = []
    for i, nom in enumerate(nombres):
        ax = fig.add_subplot(gs[i + 1], sharex=ejes[0] if ejes else None)
        ejes.append(ax)
        ax.set_facecolor(_FONDO)
        if i < 3:
            yo, yc = np.rad2deg(orig[:, i]), np.rad2deg(comp[:, i])
            ax.plot(t_o, yo, color=_COLOR_ORIGINAL, lw=1.6, solid_capstyle="round")
            ax.plot(t_c, yc, color=_COLOR_COMPACTA, lw=2.0, solid_capstyle="round")
        else:
            ax.step(t_o, orig[:, 3] > UMBRAL_A1, where="post", color=_COLOR_ORIGINAL, lw=1.6)
            ax.step(t_c, comp[:, 3] > UMBRAL_A1, where="post", color=_COLOR_COMPACTA, lw=2.0)
            ax.set_ylim(-0.25, 1.25)
            ax.set_yticks([0, 1], ["apagada", "encendida"])
        ax.set_ylabel(nom, color=_TINTA_2, fontsize=10)
        ax.grid(True, color=_GRILLA, lw=0.8)
        ax.set_axisbelow(True)
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
        for lado in ("left", "bottom"):
            ax.spines[lado].set_color(_GRILLA)
        ax.tick_params(colors=_TINTA_2, labelsize=9, length=0)
        ax.xaxis.set_major_formatter(coma)
        if i < 3:
            ax.yaxis.set_major_formatter(coma)
        if i < len(nombres) - 1:
            ax.tick_params(labelbottom=False)
    ejes[0].set_xlim(0, x_max)
    ejes[-1].set_xlabel("Tiempo [s]  (una fila cada 20 ms)", color=_TINTA_2, fontsize=10)

    # Rotulos directos al final de cada serie (ademas de la leyenda)
    ax0 = ejes[0]
    for t, y, texto, color in ((t_c[-1], np.rad2deg(comp[-1, 0]), "compacta", _COLOR_COMPACTA),
                               (t_o[-1], np.rad2deg(orig[-1, 0]), "original", _COLOR_ORIGINAL)):
        ax0.annotate(texto, (t, y), xytext=(6, 0), textcoords="offset points", va="center",
                     fontsize=9.5, color=_TINTA, fontweight="bold")
        ax0.plot([t], [y], "o", ms=5, color=color, mec=_FONDO, mew=1.5)

    fig.legend(handles=[Line2D([], [], color=_COLOR_ORIGINAL, lw=1.6),
                        Line2D([], [], color=_COLOR_COMPACTA, lw=2.0)],
               labels=[f"original ({_num(informe.t_original_s)} s)", f"compacta ({_num(informe.t_compacta_s)} s)"],
               loc="upper left", bbox_to_anchor=(0.082, 0.948), ncol=2, frameon=False,
               fontsize=9.5, labelcolor=_TINTA)

    nota = (f"Se quitan solo los tramos con el brazo quieto (|Δq| < {_num(informe.eps_deg, 2)}°): "
            f"el recorrido no cambia. Espera tras encender la bomba: {_num(informe.t_agarre_s)} s; "
            f"tras apagarla: {_num(informe.t_suelta_s)} s.")
    if energia is None:
        nota += ("\nEnergía ahorrada = P_reposo · Δt + P_bomba · Δt_bomba; cargar las potencias "
                 "medidas en utils/trajectory_compactor.py para estimarla.")
    fig.text(0.09, 0.025, nota, fontsize=8.5, color=_TINTA_2, ha="left", va="bottom")

    ruta = Path(ruta)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(ruta, facecolor=_FONDO)
    return ruta


def graficar_en_segundo_plano(orig: np.ndarray, comp: np.ndarray, informe: InformeCompactacion,
                              ruta: Path, nombre: str, logger=None) -> threading.Thread:
    """Genera el grafico en un hilo aparte (la GUI no espera). Errores al log, nunca excepciones."""
    def tarea():
        try:
            salida = graficar_comparacion(orig, comp, informe, ruta, nombre)
            if logger is not None:
                logger.info(f"Gráfico comparativo guardado en '{salida.as_posix()}'.")
        except Exception as e:      # el grafico es un extra: nunca debe afectar el guardado
            if logger is not None:
                logger.warning(f"No se pudo generar el gráfico comparativo de '{nombre}': {e}")

    hilo = threading.Thread(target=tarea, name=f"grafico-{nombre}", daemon=True)
    hilo.start()
    return hilo
