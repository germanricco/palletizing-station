#!/usr/bin/env python3
"""
Compacta trayectorias grabadas: elimina los tiempos muertos y garantiza la espera minima
del vacio despues de cada flanco del efector (ver utils/trajectory_compactor.py).

Uso (desde la raiz del repo):
    python compactar_trayectorias.py                 # todas las de trajectories/
    python compactar_trayectorias.py pick_caja1      # solo esa (nombre sin .npy, o ruta)
    python compactar_trayectorias.py a b c --png     # varias, con grafico comparativo

Por cada <nombre>.npy genera <nombre>_compacta.npy en la misma carpeta (el original no se
toca) y, con --png, ensayos/compactar/<nombre>_compacta.png. Las *_compacta.npy se cargan
desde la GUI con CARGAR TRAY. como cualquier otra trayectoria.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from utils.trajectory_compactor import (  # noqa: E402
    DT_S, EPS_DEG_DEFECTO, T_AGARRE_DEFECTO, T_SUELTA_DEFECTO, compactar,
)

CARPETA = Path("trajectories")
SUFIJO = "_compacta"


def resolver_entradas(nombres: list[str]) -> list[Path]:
    """Sin nombres: todas las .npy de trajectories/ salvo las ya compactadas."""
    if not nombres:
        return sorted(p for p in CARPETA.glob("*.npy") if not p.stem.endswith(SUFIJO))
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


def graficar(orig: np.ndarray, comp: np.ndarray, ruta: Path, titulo: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ejes = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
    t_o, t_c = np.arange(len(orig)) * DT_S, np.arange(len(comp)) * DT_S
    nombres = ("q1 base [°]", "q2 hombro [°]", "q3 codo [°]", "a1 efector")
    for i, (ax, nom) in enumerate(zip(ejes, nombres)):
        conv = np.rad2deg if i < 3 else (lambda x: x)
        ax.plot(t_o, conv(orig[:, i]), color="0.65", lw=1, label="original")
        ax.plot(t_c, conv(comp[:, i]), lw=1.4, label="compacta")
        ax.set_ylabel(nom)
        ax.grid(alpha=0.3)
    ejes[0].legend(loc="upper right")
    ejes[0].set_title(titulo)
    ejes[-1].set_xlabel("t [s]  (20 ms por fila)")
    fig.tight_layout()
    ruta.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(ruta, dpi=120)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Elimina los tiempos muertos de trayectorias grabadas.")
    ap.add_argument("nombres", nargs="*", help="trayectorias a compactar (por defecto, todas)")
    ap.add_argument("--eps", type=float, default=EPS_DEG_DEFECTO,
                    help=f"umbral de quietud [°] (def. {EPS_DEG_DEFECTO})")
    ap.add_argument("--t-agarre", type=float, default=T_AGARRE_DEFECTO,
                    help=f"espera minima tras encender el efector [s] (def. {T_AGARRE_DEFECTO})")
    ap.add_argument("--t-suelta", type=float, default=T_SUELTA_DEFECTO,
                    help=f"espera minima tras apagar el efector [s] (def. {T_SUELTA_DEFECTO})")
    ap.add_argument("--png", action="store_true", help="guardar graficos en ensayos/compactar/")
    a = ap.parse_args()

    entradas = resolver_entradas(a.nombres)
    if not entradas:
        raise SystemExit(f"No hay trayectorias para compactar en {CARPETA}/.")

    print(f"{'trayectoria':28s} {'filas':>11s} {'duracion [s]':>15s} {'ahorro':>7s}  agarre/suelta (grabada -> final)  RTH")
    for ruta in entradas:
        traj = np.load(ruta, allow_pickle=False)
        try:
            comp, inf = compactar(traj, a.eps, a.t_agarre, a.t_suelta)
        except ValueError as e:
            print(f"{ruta.stem:28s} OMITIDA: {e}")
            continue

        salida = ruta.with_name(ruta.stem + SUFIJO + ".npy")
        np.save(salida, comp)

        esperas = []
        for f in inf.flancos:
            minimo = a.t_agarre if f.encendido else a.t_suelta
            marca = "+" if f.pausa_grabada_s < minimo else ""
            esperas.append(f"{'A' if f.encendido else 'S'} {f.pausa_grabada_s:.2f}->{minimo:.2f}{marca}")
        rth = "OK" if inf.rth_deg <= 2.0 else f"FALLA ({inf.rth_deg:.1f}°)"
        print(f"{ruta.stem:28s} {inf.n_original:5d}->{inf.n_compacta:<5d} "
              f"{inf.t_original_s:6.1f}->{inf.t_compacta_s:<6.1f} {inf.ahorro_pct:6.0f}%  "
              f"{', '.join(esperas) or 'sin flancos de a1':33s} {rth}")

        if a.png:
            graficar(traj, comp, Path("ensayos") / "compactar" / f"{salida.stem}.png", ruta.stem)

    print("\nA = agarre, S = suelta; '+' = pausa grabada mas corta que la minima (se extendio).")
    print(f"Archivos generados: <nombre>{SUFIJO}.npy en {CARPETA}/ (los originales no se modifican).")


if __name__ == "__main__":
    main()
