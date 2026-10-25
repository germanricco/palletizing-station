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
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

DT_S = 0.020            # s por fila: periodo de T: y del metronomo de AUTO en el firmware
UMBRAL_A1 = 0.5         # a1 > umbral -> efector encendido (la GUI graba 0.0 / 1.0)

# eps por defecto: 0.04 grados <= 1 us de PWM en las tres articulaciones
# (hombro 25 us/grado, codo 14.8 us/grado, base 11.1 us/grado con el mapeo de Config.hpp).
EPS_DEG_DEFECTO = 0.04
T_AGARRE_DEFECTO = 2.0
T_SUELTA_DEFECTO = 0.5


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
    flancos: List[Flanco] = field(default_factory=list)
    rth_deg: float = 0.0    # |inicio - fin| maximo de la trayectoria compacta [grados]

    @property
    def t_original_s(self) -> float:
        return self.n_original * DT_S

    @property
    def t_compacta_s(self) -> float:
        return self.n_compacta * DT_S

    @property
    def ahorro_pct(self) -> float:
        return 100.0 * (1.0 - self.n_compacta / self.n_original) if self.n_original else 0.0


def _filas_espera(t_s: float) -> int:
    """Filas quietas tras el flanco para que flanco -> primer movimiento dure t_s."""
    return max(0, round(t_s / DT_S) - 1)


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
        flancos=pausas_tras_flancos(traj, eps_deg),
        rth_deg=float(np.rad2deg(np.max(np.abs(compacta[0, :3] - compacta[-1, :3])))),
    )
    return compacta, informe
