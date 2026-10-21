# Prueba integral: ciclo automático con cinta y sensor IR

Guía para validar en el hardware real la integración de la cinta transportadora y el ciclo automático. Es **una sola prueba de unos 15 minutos**, desde la GUI.

## Qué cambió

- **Firmware**
  - La cinta ya no corre en modo de prueba aislada (se eliminó `CINTA_BRINGUP`). Ahora la controla la máquina de estados:
    - `M120` arma la cinta (solo en AUTO);
    - `M121` la desarma (siempre se acepta);
    - `M122` / `M122 S0` es la marcha manual (solo en IDLE/MANUAL).
  - El firmware informa cada cambio de la cinta con `EVT:CINTA,<estado>`.
  - La cinta frena sola al detectar una caja.
  - Rearranca **500 ms después** de que el robot termina la trayectoria, y solo si el sensor quedó libre.
- **GUI**
  - **INICIAR CICLO** reemplaza a EJECUTAR. Funciona con una trayectoria única (se repite en cada caja) o con una receta.
  - Cada caja que llega al sensor dispara la trayectoria sola.
  - **DETENER CICLO** frena la cinta.
  - Nueva tarjeta **CINTA TRANSPORTADORA**: muestra el estado en vivo y tiene el botón MARCHA/PARADA (solo en IDLE/MANUAL).

## Preparación

1. Actualizar el repo y cambiar a la rama de la cinta (o la rama que se indique).
2. STM32CubeIDE: **Project → Build** y después **Run** para flashear.
   - La Flash queda al ~94 % en Debug. Si no entra, avisar: se pasa a `-Og`.
3. Alimentar el robot y la cinta. **Medir la polaridad de los 12 V antes de energizar.**
4. Ejecutar `python main.py`.

## Secuencia de prueba

Marcar cada paso. Si uno falla, anotar el número y copiar el panel de logs de la GUI.

| # | Acción | Resultado esperado |
|---|---|---|
| 1 | **CONECTAR STM32** | La tarjeta CINTA pasa de `DESCONOCIDO` a `DETENIDA`. La cinta está quieta. |
| 2 | Modo **IDLE**, botón **MARCHA** | La cinta arranca suave (~0,3 s). La tarjeta muestra `MARCHA MANUAL` en cian y el botón cambia a PARADA. |
| 3 | Con la cinta en marcha, poner una caja que llegue al sensor | La cinta frena sola y la tarjeta vuelve a `DETENIDA`. |
| 4 | Con la caja frente al sensor, tocar **MARCHA** | No arranca (protección: no tira la caja). El log muestra `ERR:SENSOR_ACTIVO`. |
| 5 | **Grabar la trayectoria de pick** con esa caja tal como la frenó la cinta (**no moverla**): MANUAL → GRABAR TRAY. → Home → pick → place → volver a Home (pulsador izquierdo del joystick) → TERMINAR GRAB. → guardar | Se guarda la trayectoria. Grabarla con la caja en la posición real de frenado compensa el sobrepaso de la cinta. |
| 6 | Sacar la caja. **AUTO** → CARGAR TRAY. (la del paso 5) → **INICIAR CICLO** | La cinta arranca y la tarjeta muestra `AVANZANDO` en verde. El botón cambia a DETENER CICLO. |
| 7 | Poner una caja en la cinta | Al llegar al sensor: la cinta frena, la tarjeta muestra `ESPERANDO PICK` en ámbar y **el robot arranca solo**. |
| 8 | Observar el pick | Cuando el robot levanta la caja, la cinta **sigue frenada**. Rearranca recién cuando el robot vuelve a Home (~0,5 s después de terminar). La etiqueta muestra `CAJAS: 1`. |
| 9 | Repetir con 2 cajas más, sin tocar nada | Cada caja dispara el ciclo. La etiqueta llega a `CAJAS: 3`. |
| 10 | **Pick fallido:** quitar la caja de la ventosa durante el pick, o dejar la caja frente al sensor | Al terminar la trayectoria, la cinta **no arranca** mientras haya una caja frente al sensor. Al retirarla a mano, arranca. |
| 11 | **DETENER CICLO**. Con el ciclo detenido, poner una caja frente al sensor y tocar **INICIAR CICLO** | La cinta no arranca y el robot hace el pick de inmediato (la caja ya estaba esperando). |
| 12 | Con la cinta avanzando, **E-STOP** | La cinta frena al instante y la tarjeta muestra `DETENIDA`. Esperar más de 1 s, **RESET**, IDLE. |
| 13 | AUTO → INICIAR CICLO → **DETENER CICLO** | La cinta frena y vuelve a `DETENIDA`. Queda listo para iniciar de nuevo. |
| 14 | Durante un pick, **PAUSAR** → **CANCELAR** | El robot se detiene, el ciclo termina y la cinta queda `DETENIDA`. INICIAR CICLO vuelve a funcionar. |

## Si algo falla: diagnóstico rápido desde la terminal de la GUI

Escribir el comando en la terminal (abajo a la izquierda) y mirar la respuesta en los logs:

| Comando | Estado | Respuesta esperada |
|---|---|---|
| `STATUS` | cualquiera | `EVT:STATE,...` y `EVT:CINTA,<estado>` |
| `M122` / `M122 S0` | IDLE o MANUAL | `ACK_M122`, la cinta arranca o frena y llega `EVT:CINTA,MANUAL` / `DESHABILITADA` |
| `M122 S0.4` | IDLE o MANUAL | La cinta gira más rápido (calibración de velocidad) |
| `M120` / `M121` | AUTO | `ACK_M120` + `EVT:CINTA,AVANZANDO` / `ACK_M121` + `EVT:CINTA,DESHABILITADA` |

- **Si responde el firmware pero la GUI no reacciona,** el problema está del lado de la PC.
- **Si el firmware no responde,** revisar que se haya flasheado la versión nueva: al resetear, la línea `PROTO:` tiene que incluir `M120 M121 M122`.

## Ajustes posibles (en `firmware_estacion_paletizado/App/Inc/Config.hpp`)

| Constante | Valor actual | Cuándo cambiarla |
|---|---|---|
| `CINTA_VELOCIDAD_NOMINAL` | 0.27 | La caja llega demasiado rápido o lento. |
| `CINTA_RAMPA_ARRANQUE_MS` | 300 | La caja patina al arrancar (subir) o la cinta tarda en arrancar (bajar). |
| `CINTA_RETARDO_LIBERACION_MS` | 500 | La cinta arranca antes de que el brazo salga de la zona de pick (subir). |

## Limitaciones conocidas

- **DETENER CICLO con el robot arrancando:** si la caja ya disparó la trayectoria, ese pick se completa y recién después el ciclo termina. Durante ese lapso el botón sigue visible; el log muestra "se completa la caja en curso".
- **Reiniciar con receta:** INICIAR CICLO siempre arranca la receta desde la caja 1. Si se detiene a mitad de un pallet, el siguiente arranque vuelve a la primera posición.
- **Pick fallido:** si la caja queda frente al sensor, la cinta no arranca y el ciclo queda esperando. Retirar la caja a mano: la cinta arranca y el ciclo sigue. El contador ya cuenta esa caja.
