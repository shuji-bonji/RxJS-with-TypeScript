---
description: "El operador de auditoría es un operador de filtrado RxJS que sólo emite el último valor dentro del periodo controlado por el Observable personalizado. Es ideal para el control dinámico del tiempo."
---

# auditoría - último valor del período de control emitido

El operador `audit` espera hasta que un Observable personalizado emite un valor y emite el **último valor** emitido por la fuente dentro de ese periodo.
Mientras que `auditTime` está controlado por un tiempo fijo, `audit` permite **controlar el periodo** con un Observable dinámico.

## 🔰 Sintaxis básica y uso

```ts
import { fromEvent, interval } from 'rxjs';
import { audit } from 'rxjs';

// Evento clic
const clicks$ = fromEvent(document, 'click');

// 1Períodos de tiempo separados cada segundo
clicks$.pipe(
  audit(() => interval(1000))
).subscribe(() => {
  console.log('Se ha registrado el clic');
});
```

- Cuando se produce un clic, comienza un periodo de un segundo.
- Sólo se emite el último clic de ese periodo de un segundo.
- Después de un segundo, comienza el siguiente periodo.

[🌐 Documentación oficial de RxJS - `audit`](https://rxjs.dev/api/operators/audit)

> [!WARNING] Atención en código de producción

> El ejemplo anterior omite la cancelación de la suscripción `fromEvent` para simplificar la explicación. En código real, utilice `takeUntil(destroy$)`, `take(N)` o `Subscription.unsubscribe()` para gestionar explícitamente el ciclo de vida. Más información: [Superando dificultades: gestión del ciclo de vida](/es/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Patrones típicos de utilización

- **Muestreo dinámico de intervalos**: ajuste de la duración en función de la carga.
- **Control de tiempo personalizado**: control de periodo basado en otro Observable.
- **Limitación adaptativa de eventos**: adelgazamiento sensible al contexto.

## 🔍 Diferencias con auditTime

| Operador. | Control de períodos | Caso de uso. |
|---|---|---|
| `auditTime`. | Tiempo fijo (milisegundos) | Control simple basado en el tiempo |
| `audit`. | **Observable personalizado**. | **Control dinámico de periodo**. |

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, auditTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// auditTime - Fijo1segundos
clicks$.pipe(
  auditTime(1000)
).subscribe(() => console.log('Fijo1segundos'));

// audit - Periodo dinámico
let period = 1000;
clicks$.pipe(
  audit(() => {
    period = Math.random() * 2000; // 0~.2Periodo aleatorio de segundos
    return timer(period);
  })
).subscribe(() => console.log(`Periodo dinámico: ${period}ms`));
```

## 🧠 Ejemplo práctico de código 1: Muestreo dinámico basado en la carga

Este es un ejemplo de ajuste del intervalo de muestreo en función de la carga del sistema.

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map } from 'rxjs';

// UICreación
const output = document.createElement('div');
output.innerHTML = '<h3>Muestreo dinámico</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Cambio de carga';
document.body.appendChild(button);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
output.appendChild(statusDiv);

const logDiv = document.createElement('div');
logDiv.style.marginTop = '10px';
logDiv.style.maxHeight = '200px';
logDiv.style.overflow = 'auto';
output.appendChild(logDiv);

// Nivel de carga (0: Carga baja1: Carga media,2: Carga alta)
let loadLevel = 0;

fromEvent(button, 'click').subscribe(() => {
  loadLevel = (loadLevel + 1) % 3;
  const levels = ['Carga baja', 'Carga media', 'Carga alta'];
  statusDiv.textContent = `Carga actual: ${levels[loadLevel]}`;
});

// Evento de movimiento del ratón
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  audit(() => {
    // Duración en función de la carga
    const periods = [2000, 1000, 500]; // Carga baja→Larga duración, carga alta→Corta duración
    return timer(periods[loadLevel]);
  }),
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe(pos => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Posición del ratón: (${pos.x}, ${pos.y})`;
  logDiv.insertBefore(log, logDiv.firstChild);

  // Máx.10Visualizar hasta
  while (logDiv.children.length > 10) {
    logDiv.removeChild(logDiv.lastChild!);
  }
});
```

- Muestreo fino a intervalos de 2 s cuando la carga es baja (modo de ahorro de energía).
- Muestreo fino a intervalos de 500 ms cuando la carga es alta.
- El periodo puede ajustarse dinámicamente en función de la carga.

## 🎯 Ejemplo práctico de código 2: Control del periodo basado en otros flujos

```ts
import { fromEvent, timer } from 'rxjs';
import { audit, map, startWith } from 'rxjs';

// UICreación
const container = document.createElement('div');
document.body.appendChild(container);

const slider = document.createElement('input');
slider.type = 'range';
slider.min = '100';
slider.max = '2000';
slider.value = '1000';
container.appendChild(document.createTextNode('Intervalo: '));
container.appendChild(slider);

const intervalDisplay = document.createElement('span');
intervalDisplay.textContent = ' 1000ms';
container.appendChild(intervalDisplay);

const output = document.createElement('div');
output.style.marginTop = '10px';
container.appendChild(output);

// Supervisar los valores del control deslizante
const sliderValue$ = fromEvent(slider, 'input').pipe(
  map(() => Number(slider.value)),
  startWith(1000)
);

sliderValue$.subscribe(value => {
  intervalDisplay.textContent = ` ${value}ms`;
});

// Evento clic
const clicks$ = fromEvent(document, 'click');

let currentInterval = 1000;

// Actualizar los valores del control deslizante
sliderValue$.subscribe(value => {
  currentInterval = value;
});

// Haga clic enauditControlado por
clicks$.pipe(
  audit(() => timer(currentInterval))
).subscribe(() => {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] Registro de clics (intervalo: ${currentInterval}ms)`;
  output.insertBefore(log, output.firstChild);
});
```

## ⚠️ Notas.

### 1. el primer valor no se emite inmediatamente

Después de que el `audit` recibe el primer valor, espera hasta el final del periodo.

```ts
import { interval, timer } from 'rxjs';
import { audit, take } from 'rxjs';

interval(100).pipe(
  audit(() => timer(1000)),
  take(3)
).subscribe(val => {
  console.log(val);
});
// Salida:
// 9  (1Segundos después0~.9Último valor de)
// 19 (2Segundos después10~.19Último valor de)
// 29 (3Segundos después20~.29Último valor de)
```

### 2. El Observable de duración se genera de nuevo cada vez.

Las funciones pasadas a `audit` **deben devolver un nuevo Observable cada vez**.

```ts
// ❌ Mal ejemplo: Si la mismaObservableinstancia se utiliza y se vuelve a utilizar
const duration$ = timer(1000);
source$.pipe(
  audit(() => duration$) // 2No funciona después de la segunda vez
).subscribe();

// ✅ Buen ejemplo: Se crea una nueva instancia cada vezObservableGenerar un
source$.pipe(
  audit(() => timer(1000))
).subscribe();
```

### 3. memoria y rendimiento

Utilizar `audit` en flujos donde se emiten valores con frecuencia consume memoria.

```ts
import { interval, timer } from 'rxjs';
import { audit } from 'rxjs';

// flujo rápido (10mspor segundo)
interval(10).pipe(
  audit(() => timer(1000)) // 1Muestreo cada segundo
).subscribe();
// 1por segundo100Los valores se almacenan en memoria y sólo el último1Sólo se emite el último
```

## 🆚 Comparación con operadores similares

| Operadores | Cuándo emitir | Valor a emitir | Caso de uso. |
|---|---|---|---|
| Auditoría. | Al **fin** del período | El **último** valor dentro del periodo | Obtener el último estado dentro del periodo |
| Acelerador | Al **inicio** del periodo | Valor del **primero** en el período | Obtener el inicio de una secuencia de eventos. |
| `debounce`. | Después** de** estacionario**. | Valor justo antes de estacionario | Esperar a que se complete la entrada |
| `sample`. | **Cuando se dispara otro Observable**. | Valor más reciente en ese momento | Instantánea periódica |

```ts
import { fromEvent, interval, timer } from 'rxjs';
import { audit, throttle, debounce, sample } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// audit: 1Último clic en segundos
clicks$.pipe(
  audit(() => timer(1000))
).subscribe(() => console.log('audit: Último clic'));

// throttle: 1Primer clic en segundos
clicks$.pipe(
  throttle(() => timer(1000))
).subscribe(() => console.log('throttle: Primer'));

// debounce: Después de que cese el clic1Segundos después
clicks$.pipe(
  debounce(() => timer(1000))
).subscribe(() => console.log('debounce: Después de parar'));

// sample: 1Muestreo cada segundo
clicks$.pipe(
  sample(interval(1000))
).subscribe(() => console.log('sample: Periódico'));
```

## 📚 Operadores relacionados.

- **[auditTime](. /auditTime)** - controlado por tiempo fijo (versión simplificada de `audit`).
- **[throttle](. /throttleTime)** - primer valor emitido al inicio del periodo.
- **[debounce](. /debounceTime)** - emite un valor tras un periodo de inactividad.
- Muestra](. /sampleTime)** - muestreo en el momento de otro Observable.

## Resumen.

El operador `audit` emite el último valor dentro de un periodo controlado dinámicamente por un Observable personalizado.

- ✅ Es posible el control dinámico del periodo.
- ✅ Muestreo adaptativo basado en la carga.
- ✅ Control basado en otros flujos.
- ⚠️ Es necesario generar un nuevo Observable cada vez
- ⚠️ Memoria sensible para la emisión frecuente
