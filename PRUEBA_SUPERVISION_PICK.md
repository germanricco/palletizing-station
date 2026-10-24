# Prueba: supervisión del pick con el sensor IR y compactación de trayectorias

Guía para validar en el hardware real la rama `feature/supervision-pick`. Parte de lo que ya pasó `PRUEBA_CICLO_CINTA.md` (pasos 1 a 17 OK con 02751eb). Lleva unos 20 minutos.

## Qué cambió

- **Firmware: el sensor IR supervisa el pick.** Durante cada trayectoria del ciclo, el firmware cruza el estado del IR con los flancos del efector (a1) y espera esta secuencia:

  | Fase | a1 | IR esperado | Si no se cumple → causa |
  |---|---|---|---|
  | Aproximación | 0 | ocupado | IR libre ≥ 150 ms → `CAJA_RETIRADA` |
  | Agarre (bomba encendida, brazo quieto) | 1 | ocupado | a1 se apaga con el IR ocupado → `OCUPADO_EN_SUELTA` |
  | Elevación (primer movimiento después del agarre) | 1 | se libera | sigue ocupado 1,5 s después de empezar a subir → `NO_LIBERO` |
  | Traslado | 1 | libre | vuelve a ocuparse ≥ 150 ms → `CAJA_CAIDA` |
  | Fin de trayectoria sin pick confirmado | — | — | `SIN_AGARRE` / `NO_LIBERO` |

- **Ante un desvío:** se aborta la trayectoria, se purga el vacío y el brazo vuelve a Home **con rampa** (estado `PickRecovery`).
  - Si la caja sigue frente al IR, se **reintenta una vez** automáticamente.
  - Si vuelve a fallar, o si no hay caja, pasa a **alarma** (`PickAlarm`) y espera al operador:
    - **REANUDAR** (`M115`): si la caja sigue frente al sensor, la vuelve a tomar; si se retiró, la cinta avanza.
    - **CANCELAR** (`M113`): termina el ciclo y detiene la cinta.
- **Contador:** la caja que falla **no se cuenta** ni avanza la receta.
- **Eventos nuevos:** `EVT:PICK_OK,<ms>`, `EVT:PICK_FAIL,<causa>,<intento>`, `EVT:PICK_RETRY,<intento>`, `EVT:PICK_ALARM,<causa>`. Comando nuevo: `M115`.
- **GUI:** estados `AUTO_PICK_RECOVERY` (ámbar) y `AUTO_PICK_ALARM` (rojo) en el gemelo digital. En alarma se habilitan REANUDAR y CANCELAR. El log explica la causa.
- **PC:** `compactar_trayectorias.py` elimina los tiempos muertos de las trayectorias grabadas (ver al final).

## Preparación

1. Cambiar a la rama `feature/supervision-pick`.
2. **STM32CubeIDE → Debug con `-Og`** (obligatorio: a `-O0` la flash queda al ~99 %):
   - Project → Properties → C/C++ Build → Settings → **MCU GCC Compiler → Optimization → Optimize for debug (-Og)**.
   - Lo mismo en **MCU G++ Compiler → Optimization**.
   - Project → Build, después Run para flashear.
3. Verificar al resetear que la línea `PROTO:` incluya `M115`.
4. `python main.py`. Usar la trayectoria de pick grabada en el paso 5 de `PRUEBA_CICLO_CINTA.md`.

## Secuencia de prueba

Si un paso falla, anotar el número y copiar el panel de logs.

| # | Acción | Resultado esperado |
|---|---|---|
| 1 | AUTO → CARGAR TRAY. → INICIAR CICLO. Pasar **3 cajas** normales | Cada caja: `Pick confirmado: ... X ms` en el log y `CAJAS` +1. **Anotar los X ms** (calibración de `PICK_T_LIBERACION_MS`). |
| 2 | Con la caja siguiente, **sujetarla contra la cinta** mientras el brazo sube; soltarla después | `PICK FALLIDO (intento 1): ... la ventosa no la tomó`. Estado `AUTO_PICK_RECOVERY`: el brazo vuelve a Home **con rampa**. Unos 0,5 s después de llegar, reintenta solo y esta vez toma la caja. `CAJAS` +1 (una sola vez). |
| 3 | Sujetar la caja **en los dos intentos** | Después del 2.º fallo: `ALARMA DE PICK`, estado `AUTO_PICK_ALARM`, REANUDAR y CANCELAR habilitados. La cinta sigue detenida. |
| 4 | En la alarma, sin tocar la caja: **REANUDAR** | Vuelve a tomar la misma caja y el ciclo sigue. |
| 5 | Repetir 3 y, en la alarma, **retirar la caja a mano** → REANUDAR | No hay pick: la cinta arranca y la caja siguiente se procesa normal. |
| 6 | **Retirar la caja mientras el brazo baja** (antes de que encienda la bomba) | `CAJA_RETIRADA`. Sin reintento (no hay caja): alarma directa. **CANCELAR** → `AUTO_READY`, cinta `DETENIDA`, ciclo terminado. |
| 7 | Después del pick, **despegar la caja de la ventosa y dejarla frente al sensor** | `CAJA_CAIDA` (≈ 0,2 s después). Purga, Home y reintento. |
| 8 | Durante el reintento (estado `AUTO_PICK_RECOVERY`), **E-STOP** | `EMERGENCY` como siempre. RESET (> 1 s) → `INIT`. |
| 9 | Provocar una alarma y tocar **IDLE** | Termina el ciclo, cinta detenida, brazo a Home. |
| 10 | Durante la subida con la caja, **PAUSAR**, esperar 3 s y **REANUDAR** | La pausa no consume el plazo de 1,5 s: **no** debe aparecer `NO_LIBERO`. |
| 11 | *(Opcional)* Grabar una trayectoria **sin accionar la bomba** y ejecutarla en el ciclo | Al final: `SIN_AGARRE` → reintento → `SIN_AGARRE` → alarma. CANCELAR. |

## Calibración de `PICK_T_LIBERACION_MS`

Con los tiempos `X ms` del paso 1, ajustar en `Config.hpp`:

$$T_{lib} \approx 2 \cdot \max(X), \qquad T_{lib} \ge \frac{h_{lib}}{v_z} + T_{margen}$$

- $h_{lib}$ ≈ 15 mm: elevación medida para que el FC-51 deje de ver la caja.
- Si aparecen `NO_LIBERO` con picks buenos, el plazo es corto o el brazo sube muy lento en la trayectoria grabada.
- Si aparecen `CAJA_CAIDA` sin que la caja caiga, el FC-51 está parpadeando en el borde de detección: subir `PICK_PERSISTENCIA_MS` o ajustar el preset del sensor.

## Compactar trayectorias (tiempos muertos)

```
python compactar_trayectorias.py                  # todas las de trajectories/
python compactar_trayectorias.py pick_caja1 --png # una, con gráfico en ensayos/compactar/
```

- Genera `<nombre>_compacta.npy` al lado del original, que no se toca. Se carga con CARGAR TRAY. como cualquier otra.
- Garantiza **0,5 s quieto después de encender la bomba** (vacío) y después de apagarla (despegue). Si la grabación tenía menos, la alarga; si tenía más, la recorta.
- **Prueba:**
  1. Ciclo con la versión compacta: el ciclo dura menos y cada caja sigue dando `PICK_OK`.
  2. Si aparece `NO_LIBERO` con la compacta pero no con la original, 0,5 s no alcanzan para formar el vacío: subir `--t-agarre` (p. ej. 0,8).

## Limitaciones conocidas

- Una caja que cae **fuera del haz** durante el traslado no se detecta con un solo sensor. Esa caja se cuenta como colocada. Lo cerraría un vacuostato en la línea de la ventosa.
- El reintento repite la misma trayectoria. Si la caja quedó corrida, va a volver a fallar y termina en alarma: es lo esperado.
