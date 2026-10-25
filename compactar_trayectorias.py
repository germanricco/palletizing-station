#!/usr/bin/env python3
"""
Compacta trayectorias grabadas: elimina los tiempos muertos y garantiza la espera minima
del vacio despues de cada flanco del efector (ver utils/trajectory_compactor.py).

Desde la v1.1 la GUI ya guarda automaticamente la original, la compacta y el grafico al
terminar una grabacion. Este script queda para compactar grabaciones viejas o probar otros
parametros.

Uso (desde la raiz del repo):
    python compactar_trayectorias.py                 # todas las de trajectories/
    python compactar_trayectorias.py pick_caja1      # solo esa (nombre sin .npy, o ruta)
    python compactar_trayectorias.py a b c --png     # varias, con grafico comparativo
    python compactar_trayectorias.py --png --p-reposo 9 --p-bomba 6   # + energia estimada

Por cada <nombre>.npy genera <nombre>_compacta.npy en la misma carpeta (el original no se
toca) y, con --png, ensayos/compactar/<nombre>_compacta.png.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from utils.trajectory_compactor import (  # noqa: E402
    CARPETA_GRAFICOS, EPS_DEG_DEFECTO, POTENCIA_BOMBA_W, POTENCIA_REPOSO_W, SUFIJO_COMPACTA,
    T_AGARRE_DEFECTO, T_SUELTA_DEFECTO, compactar, graficar_comparacion,
)

CARPETA = Path("trajectories")


def resolver_entradas(nombres: list[str]) -> list[Path]:
    """Sin nombres: todas las .npy de trajectories/ salvo las ya compactadas."""
    if not nombres:
        return sorted(p for p in CARPETA.glob("*.npy") if not p.stem.endswith(SUFIJO_COMPACTA))
    rutas = []
    for n in nombres:
        p = Path(n)
        if p.suffix != ".npy":
            p = p.with_suffix(".npy")
        if not p.exists() and p.parent == Path("."):
            p = CARPETA / p.name
        if not p.exists():
            raise SystemExit(f"No existe la trayectoria '{n}' (buscada en {p}).")
        rutas.append(p)
    return rutas


def main() -> None:
    ap = argparse.ArgumentParser(description="Elimina los tiempos muertos de trayectorias grabadas.")
    ap.add_argument("nombres", nargs="*", help="trayectorias a compactar (por defecto, todas)")
    ap.add_argument("--eps", type=float, default=EPS_DEG_DEFECTO,
                    help=f"umbral de quietud [°] (def. {EPS_DEG_DEFECTO})")
    ap.add_argument("--t-agarre", type=float, default=T_AGARRE_DEFECTO,
                    help=f"espera minima tras encender el efector [s] (def. {T_AGARRE_DEFECTO})")
    ap.add_argument("--t-suelta", type=float, default=T_SUELTA_DEFECTO,
                    help=f"espera minima tras apagar el efector [s] (def. {T_SUELTA_DEFECTO})")
    ap.add_argument("--png", action="store_true", help=f"guardar graficos en {CARPETA_GRAFICOS.as_posix()}/")
    ap.add_argument("--p-reposo", type=float, default=POTENCIA_REPOSO_W,
                    help="potencia con el brazo quieto [W] (para estimar la energia ahorrada)")
    ap.add_argument("--p-bomba", type=float, default=POTENCIA_BOMBA_W,
                    help="potencia adicional con la bomba encendida [W]")
    a = ap.parse_args()

    entradas = resolver_entradas(a.nombres)
    if not entradas:
        raise SystemExit(f"No hay trayectorias para compactar en {CARPETA}/.")

    print(f"{'trayectoria':28s} {'duracion [s]':>15s} {'ahorro':>7s} {'bomba [s]':>12s} "
          f"{'energia':>9s}  agarre/suelta (grabada -> final)  RTH")
    for ruta in entradas:
        traj = np.load(ruta, allow_pickle=False)
        try:
            comp, inf = compactar(traj, a.eps, a.t_agarre, a.t_suelta)
        except ValueError as e:
            print(f"{ruta.stem:28s} OMITIDA: {e}")
            continue

        salida = ruta.with_name(ruta.stem + SUFIJO_COMPACTA + ".npy")
        np.save(salida, comp)

        esperas = []
        for f in inf.flancos:
            minimo = a.t_agarre if f.encendido else a.t_suelta
            marca = "+" if f.pausa_grabada_s < minimo else ""
            esperas.append(f"{'A' if f.encendido else 'S'} {f.pausa_grabada_s:.2f}->{minimo:.2f}{marca}")
        energia = inf.energia_ahorrada_j(a.p_reposo, a.p_bomba)
        energia_txt = f"{energia:7.0f} J" if energia is not None else "      —  "
        rth = "OK" if inf.rth_deg <= 2.0 else f"FALLA ({inf.rth_deg:.1f}°)"
        print(f"{ruta.stem:28s} {inf.t_original_s:6.1f}->{inf.t_compacta_s:<6.1f} {inf.ahorro_pct:6.0f}% "
              f"{inf.t_bomba_original_s:5.1f}->{inf.t_bomba_compacta_s:<5.1f} {energia_txt}  "
              f"{', '.join(esperas) or 'sin flancos de a1':33s} {rth}")

        if a.png:
            graficar_comparacion(traj, comp, inf, CARPETA_GRAFICOS / f"{salida.stem}.png", ruta.stem,
                                 a.p_reposo, a.p_bomba)

    print("\nA = agarre, S = suelta; '+' = pausa grabada mas corta que la minima (se extendio).")
    print("Energia: solo con --p-reposo y --p-bomba (o con las potencias en utils/trajectory_compactor.py).")
    print(f"Archivos generados: <nombre>{SUFIJO_COMPACTA}.npy en {CARPETA}/ (los originales no se modifican).")


if __name__ == "__main__":
    main()
