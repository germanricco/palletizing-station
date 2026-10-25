# Prueba: supervisión del pick con el sensor IR y compactación de trayectorias

Guía para validar en el hardware real la rama `feature/supervision-pick` (v1.1). Parte de lo que ya pasó `PRUEBA_CICLO_CINTA.md` (pasos 1 a 17 OK con 02751eb). La secuencia completa lleva unos 25 minutos. **Si ya pasaste la versión del 24/10, alcanza con los pasos marcados (nuevo).**

## Qué cambió

- **Firmware: el sensor IR supervisa el pick.** Durante cada trayectoria del ciclo, el firmware cruza el estado del IR con los flancos del efector (a1) y espera esta secuencia:

  | Etapa | a1 | IR esperado | Si no se cumple → código |
  |---|---|---|---|
  | 1. Bajada a la caja | 0 | ocupado | IR libre ≥ 150 ms → `CAJA_MOVIDA` |
  | 2. Succión (bomba encendida, brazo quieto) | 1 | ocupado | IR libre ≥ 150 ms → `CAJA_MOVIDA` |
  | 3. Subida (primer movimiento después del agarre) | 1 | se libera | sigue ocupado 1,5 s después de empezar a subir → `CAJA_NO_LEVANTADA` |
  | 4. Traslado | 1 | libre | vuelve a ocuparse ≥ 150 ms → `CAJA_CAIDA_EN_CINTA` |
  | 5. Place | 0 | libre | se apaga la bomba con el IR ocupado y sin pick confirmado → `PLACE_SIN_PICK` |
  | Fin de la trayectoria | — | — | nunca se encendió la bomba → `TRAYECTORIA_SIN_SUCCION` |

- **Ante un desvío:** se aborta la trayectoria, se purga el vacío y el brazo vuelve a Home **con rampa** (estado `PickRecovery`).
  - Si la caja sigue frente al IR, se **reintenta una vez** automáticamente.
  - Si vuelve a fallar, si no hay caja, o si la falla es `TRAYECTORIA_SIN_SUCCION`, pasa a **alarma** (`PickAlarm`) y espera al operador:
    - **REANUDAR** (`M115`): si la caja sigue frente al sensor, la vuelve a tomar; si se retiró, la cinta avanza.
    - **CANCELAR** (`M113`): termina el ciclo y detiene la cinta.
- **Contador:** la caja que falla **no se cuenta** ni avanza la receta.
- **Eventos:** `EVT:PICK_OK,<ms>`, `EVT:PICK_FAIL,<código>,<intento>`, `EVT:PICK_RETRY,<intento>`, `EVT:PICK_ALARM,<código>`. Comando nuevo: `M115`.
- **GUI:** estados `AUTO_PICK_RECOVERY` (ámbar) y `AUTO_PICK_ALARM` (rojo) en el gemelo digital. En alarma se habilitan REANUDAR y CANCELAR. El log explica la causa en castellano.
- **Grabación (nuevo):** al guardar una grabación se guardan automáticamente la original, la versión compacta (sin tiempos muertos) y un gráfico comparativo (ver al final).

### Cambios del 25/10 respecto del primer ensayo

| Antes | Ahora |
|---|---|
| `CAJA_RETIRADA` | `CAJA_MOVIDA`; además se detecta durante la succión (antes la tomaba como pick confirmado) |
| `NO_LIBERO` | `CAJA_NO_LEVANTADA` |
| `OCUPADO_EN_SUELTA` | `PLACE_SIN_PICK` |
| `CAJA_CAIDA` | `CAJA_CAIDA_EN_CINTA` |
| `SIN_AGARRE` (con reintento) | `TRAYECTORIA_SIN_SUCCION`, directo a alarma, sin reintento |
| Compactar a mano con el script | Original + compacta + gráfico automáticos al guardar |

## Preparación

1. Traer la rama actualizada y compilar. La rama ya tiene Debug con `-Og` en el `.cproject`, así que no hay que tocar la configuración:
   - CubeIDE: clic derecho en el proyecto → **Refresh**, después **Project → Clean** y **Build**, y **Run** para flashear.
2. Verificar al resetear que la línea `PROTO:` incluya `M115`.
3. `python main.py`. Usar la trayectoria de pick grabada en el paso 5 de `PRUEBA_CICLO_CINTA.md`, o una nueva (paso 13).

## Secuencia de prueba

Si un paso falla, anotar el número y copiar el panel de logs.

| # | Acción | Resultado esperado |
|---|---|---|
| 1 | AUTO → CARGAR TRAY. → INICIAR CICLO. Pasar **3 cajas** normales | Cada caja: `Pick confirmado: ... X ms` en el log y `CAJAS` +1. **Anotar los X ms** (calibración de `PICK_T_LIBERACION_MS`). |
| 2 | Con la caja siguiente, **sujetarla contra la cinta** mientras el brazo sube; soltarla después | `PICK FALLIDO (intento 1): El brazo subió pero la caja sigue en la cinta...` (`CAJA_NO_LEVANTADA`). Estado `AUTO_PICK_RECOVERY`: el brazo vuelve a Home **con rampa**. Unos 0,5 s después de llegar, reintenta solo y toma la caja. `CAJAS` +1 (una sola vez). |
| 3 | Sujetar la caja **en los dos intentos** | Después del 2.º fallo: `ALARMA DE PICK`, estado `AUTO_PICK_ALARM`, REANUDAR y CANCELAR habilitados. La cinta sigue detenida. |
| 4 | En la alarma, sin tocar la caja: **REANUDAR** | Vuelve a tomar la misma caja y el ciclo sigue. |
| 5 | Repetir 3 y, en la alarma, **retirar la caja a mano** → REANUDAR | No hay pick: la cinta arranca y la caja siguiente se procesa normal. |
| 6 | **Retirar la caja mientras el brazo baja** (antes de que encienda la bomba) | `CAJA_MOVIDA`. Sin reintento (no hay caja): alarma directa. **CANCELAR** → `AUTO_READY`, cinta `DETENIDA`, ciclo terminado. |
| 7 | **(nuevo)** Con la bomba **ya encendida** y antes de que el brazo suba, **sacar la caja** | `CAJA_MOVIDA` ≈ 0,2 s después (antes daba `Pick confirmado: ... 0 ms`). Alarma directa. **REANUDAR** → la cinta avanza y la caja siguiente se procesa normal. |
| 8 | Después del pick, **despegar la caja de la ventosa y dejarla frente al sensor** | `CAJA_CAIDA_EN_CINTA` (≈ 0,2 s después). Purga, Home y reintento. |
| 9 | Durante el reintento (estado `AUTO_PICK_RECOVERY`), **E-STOP** | `EMERGENCY` como siempre. RESET (> 1 s) → `INIT`. |
| 10 | Provocar una alarma y tocar **IDLE** | Termina el ciclo, cinta detenida, brazo a Home. |
| 11 | Durante la subida con la caja, **PAUSAR**, esperar 3 s y **REANUDAR** | La pausa no consume el plazo de 1,5 s: **no** debe aparecer `CAJA_NO_LEVANTADA`. |
| 12 | **(nuevo)** Grabar una trayectoria **sin accionar la bomba** y ejecutarla en el ciclo con una caja | Al final: `TRAYECTORIA_SIN_SUCCION` → **alarma directa, sin reintento**. CANCELAR. |
| 13 | **(nuevo)** MANUAL → Home → GRABAR TRAY. → pick y place con **pausas largas a propósito** → Home → TERMINAR GRAB. → guardar como `prueba_pausas` | En el log: `Trayectoria 'prueba_pausas' guardada (X s)`, luego `Version compacta 'prueba_pausas_compacta' guardada: X s -> Y s (-Z %)...` y `Gráfico comparativo guardado en 'ensayos/compactar/prueba_pausas_compacta.png'`. La GUI no se traba al guardar. |
| 14 | **(nuevo)** AUTO → CARGAR TRAY.: aparecen `prueba_pausas` y `prueba_pausas_compacta`. **SIMULAR** cada una | La compacta hace el mismo recorrido sin las pausas. |
| 15 | **(nuevo)** Ciclo con `prueba_pausas_compacta` y 2 cajas | Cada caja da `PICK_OK` y el ciclo dura menos que con la original. |

## Calibración de `PICK_T_LIBERACION_MS`

Con los tiempos `X ms` del paso 1, ajustar en `Config.hpp`:

$$T_{lib} \approx 2 \cdot \max(X), \qquad T_{lib} \ge \frac{h_{lib}}{v_z} + T_{margen}$$

- $h_{lib}$ ≈ 15 mm: elevación medida para que el FC-51 deje de ver la caja.
- Si aparecen `CAJA_NO_LEVANTADA` con picks buenos, el plazo es corto o el brazo sube muy lento en la trayectoria grabada.
- Si aparecen `CAJA_CAIDA_EN_CINTA` sin que la caja caiga, el FC-51 está parpadeando en el borde de detección: subir `PICK_PERSISTENCIA_MS` o ajustar el preset del sensor.

## Compactación de trayectorias

**Automática al grabar.** Al confirmar el nombre en el diálogo de guardado:

1. Se guarda la original, `trajectories/<nombre>.npy`, siempre primero.
2. Se guarda la compacta, `trajectories/<nombre>_compacta.npy`: sin los tramos con el brazo quieto, con 2 s quieto después de encender la bomba y 0,5 s después de apagarla.
3. Se genera en segundo plano el gráfico `ensayos/compactar/<nombre>_compacta.png`: duración y tiempo con la bomba encendida, original contra compacta, y las tres articulaciones y la bomba en función del tiempo.

Si la compactación falla, o la compacta no vuelve a Home, queda solo la original y el log lo avisa. Ninguna se carga sola: se elige con CARGAR TRAY.

**Energía.** Como solo se quitan tramos con el brazo quieto, lo que se ahorra por caja es ese tiempo con el sistema energizado:

$$E_{ahorro} = P_{reposo} \cdot \Delta t + P_{bomba} \cdot \Delta t_{bomba}$$

El gráfico muestra la energía cuando están cargadas las potencias medidas en `utils/trajectory_compactor.py` (`POTENCIA_REPOSO_W`: brazo quieto sosteniendo una pose; `POTENCIA_BOMBA_W`: consumo adicional con la bomba encendida). Mientras sean `None`, muestra solo los tiempos.

**Script**, para grabaciones viejas u otros parámetros:

```
python compactar_trayectorias.py                         # todas las de trajectories/
python compactar_trayectorias.py pick_caja1 --png        # una, con gráfico
python compactar_trayectorias.py --png --p-reposo 9 --p-bomba 6   # con energía estimada (W de ejemplo)
```

Si aparece `CAJA_NO_LEVANTADA` con la compacta pero no con la original, la espera de agarre no alcanza: subir `T_AGARRE_DEFECTO` en `utils/trajectory_compactor.py` o usar `--t-agarre`.

## Limitaciones conocidas

- Una caja que cae **fuera del haz** durante el traslado no se detecta con un solo sensor. Esa caja se cuenta como colocada. Lo cerraría un vacuostato en la línea de la ventosa.
- El IR no distingue la caja de otro objeto: una mano frente al sensor durante el traslado da `CAJA_CAIDA_EN_CINTA`.
- El reintento repite la misma trayectoria. Si la caja quedó corrida, va a volver a fallar y termina en alarma: es lo esperado.
- La supervisión supone que la bomba se enciende con la ventosa ya apoyada en la caja. Si se enciende todavía bajando, el plazo de 1,5 s arranca antes y puede dar un falso `CAJA_NO_LEVANTADA`.
- Si la grabación tenía menos de 2 s quieto después de encender la bomba, la compacta lo alarga: el tiempo con la bomba encendida puede subir aunque la trayectoria total baje.
