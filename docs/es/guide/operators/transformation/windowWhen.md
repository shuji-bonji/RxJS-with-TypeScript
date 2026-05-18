---
description: "El operador windowWhen divide Observables controlando dinámicamente la condición de fin. Como la siguiente ventana comienza inmediatamente después del fin de la ventana, es ideal para delimitación continua de datos. Se explican diferencias con bufferWhen, implementación con seguridad de tipos en TypeScript y ejemplos prácticos."
---

# windowWhen - Ventana con control dinámico de fin

El operador `windowWhen` divide Observables **controlando dinámicamente la condición de fin**. Realiza un patrón de procesamiento de stream continuo donde la siguiente ventana comienza inmediatamente cuando termina una ventana.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // Emitir valores cada 0.5 segundos

// Condición de fin: después de 1 segundo
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Valor en ventana:', value);
});

// Ventana1: 0       (Inicia en 0s → termina en 1s)
// Ventana2: 1, 2    (Inicia en 1s → termina en 2s)
// Ventana3: 3, 4    (Inicia en 2s → termina en 3s)
// Ventana4: 5, 6    (Inicia en 3s → termina en 4s)
```

**Flujo de operación**:
1. La primera ventana comienza automáticamente
2. El Observable devuelto por `closingSelector()` emite un valor → Fin de ventana
3. **La siguiente ventana comienza inmediatamente**
4. Repetir 2-3

[🌐 Documentación oficial de RxJS - `windowWhen`](https://rxjs.dev/api/operators/windowWhen)

## 💡 Patrones de uso típicos

- Recopilación de datos con intervalos de tiempo dinámicos
- Procesamiento de stream adaptativo según carga
- Control de ventana basado en resultados anteriores
- Agrupación continua de datos

## 🔍 Diferencia con bufferWhen

| Operador | Salida | Caso de uso |
|:---|:---|:---|
| `bufferWhen` | **Array (T[])** | Procesar valores agrupados juntos |
| `windowWhen` | **Observable\\<T>** | Procesamiento de stream diferente por grupo |

```ts
import { interval } from 'rxjs';
import { bufferWhen, windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500);
const closing = () => interval(1000);

// bufferWhen - emite como array
source$.pipe(
  bufferWhen(closing),
  take(3)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Salida: Buffer (array): [0]
  // Salida: Buffer (array): [1, 2]
  // Salida: Buffer (array): [3, 4]
});

// windowWhen - emite como Observable
source$.pipe(
  windowWhen(closing),
  take(3),
  mergeAll()
).subscribe(value => {
  console.log('Valor en ventana:', value);
  // Salida: Valor en ventana: 0
  // Salida: Valor en ventana: 1
  // Salida: Valor en ventana: 2
  // ...
});
```

## 🧠 Ejemplo de código práctico 1: Recopilación de datos con intervalo de tiempo dinámico

Ejemplo de ajustar el período de la siguiente ventana según el resultado de la ventana anterior.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, scan, map } from 'rxjs';

// Datos de sensor (generación continua)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10 // 20-30 grados
  }))
);

let windowNumber = 0;
let previousAvgTemp = 25;

sensorData$.pipe(
  windowWhen(() => {
    const current = ++windowNumber;
    // Intervalo más corto cuanto mayor sea la temperatura
    const duration = previousAvgTemp > 27 ? 500 : 1000;
    console.log(`Inicio de ventana ${current} (período: ${duration}ms)`);
    return timer(duration);
  }),
  mergeMap(window$ => {
    const currentWindow = windowNumber;  // Guardar número de ventana actual
    return window$.pipe(
      toArray(),
      map(data => {
        const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
        previousAvgTemp = avgTemp;
        return {
          window: currentWindow,  // Usar número de ventana guardado
          count: data.length,
          avgTemp
        };
      })
    );
  })
).subscribe(stats => {
  console.log(`Ventana ${stats.window}: temperatura promedio ${stats.avgTemp.toFixed(1)}°C, ${stats.count} muestras`);
});
```

## 🎯 Ejemplo de código práctico 2: Procesamiento de stream adaptativo según carga

Ejemplo de cambiar dinámicamente la longitud de la ventana según la carga del sistema.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { windowWhen, mergeMap, scan, map } from 'rxjs';

// Crear área de salida
const container = document.createElement('div');
document.body.appendChild(container);

const loadButton = document.createElement('button');
loadButton.textContent = 'Generar carga';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Carga baja: recopilación en intervalos de 5 segundos';
container.appendChild(status);

const logDisplay = document.createElement('div');
logDisplay.style.marginTop = '10px';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Stream de logs (generación continua)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    timestamp: new Date()
  }))
);

// Nivel de carga
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// Reducir carga cada 30 segundos
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getWindowDuration(loadLevel);
  const loadText = loadLevel === 0 ? 'Carga baja' :
                   loadLevel <= 2 ? 'Carga media' : 'Carga alta';
  status.textContent = `${loadText} (Nivel ${loadLevel}): recopilación en intervalos de ${interval / 1000}s`;
}

function getWindowDuration(load: number): number {
  // Intervalo más corto cuanto mayor sea la carga
  switch (load) {
    case 0: return 5000;
    case 1: return 3000;
    case 2: return 2000;
    case 3: return 1000;
    case 4: return 500;
    default: return 300;
  }
}

let windowNum = 0;

// Procesamiento de ventana adaptativa
logs$.pipe(
  windowWhen(() => {
    windowNum++;
    return timer(getWindowDuration(loadLevel));
  }),
  mergeMap(window$ =>
    window$.pipe(
      scan((stats, log) => ({
        count: stats.count + 1,
        errors: stats.errors + (log.level === 'ERROR' ? 1 : 0),
        window: windowNum
      }), { count: 0, errors: 0, window: windowNum })
    )
  )
).subscribe(stats => {
  const timestamp = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.textContent = `[${timestamp}] Ventana ${stats.window}: ${stats.count} eventos (errores: ${stats.errors})`;
  logDisplay.insertBefore(div, logDisplay.firstChild);
});
```

## 🆚 Diferencia con windowToggle

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowWhen: solo controlar fin (siguiente comienza inmediatamente después del fin)
source$.pipe(
  windowWhen(() => timer(1000)),
  mergeAll()
).subscribe();

// windowToggle: controlar inicio y fin por separado
source$.pipe(
  windowToggle(
    interval(1000),          // Trigger de inicio
    () => timer(500)         // Trigger de fin (500ms después del inicio)
  ),
  mergeAll()
).subscribe();
```

| Operador | Control | Período de ventana | Caso de uso |
|:---|:---|:---|:---|
| `windowWhen(closing)` | Solo control de fin | Continuo | Ventana periódica simple |
| `windowToggle(open$, close)` | Control separado inicio/fin | Superposición posible | Condiciones inicio/fin complejas |

**Puntos de selección**:
- **`windowWhen`**: Cuando se desea procesar todos los datos continuamente sin pérdida (recopilación de logs, agregación de datos, etc.)
- **`windowToggle`**: Cuando se desea procesar datos solo en períodos específicos (durante horario comercial, mientras se presiona botón, etc.)

## 🎯 Ejemplo de uso: Control de tamaño de ventana adaptativo

Ejemplo de ajustar automáticamente el período de la siguiente ventana según el resultado de la ventana anterior.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, map } from 'rxjs';

interface WindowStats {
  count: number;
  nextDuration: number;
}

const data$ = interval(100);

let previousCount = 0;

// Ajustar período de siguiente ventana según cantidad de datos
function getNextDuration(count: number): number {
  if (count > 20) {
    return 500;  // Mucha cantidad de datos → intervalo corto
  } else if (count > 10) {
    return 1000; // Cantidad media → intervalo medio
  } else {
    return 2000; // Poca cantidad de datos → intervalo largo
  }
}

data$.pipe(
  windowWhen(() => timer(getNextDuration(previousCount))),
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(data => {
        previousCount = data.length;
        return {
          count: data.length,
          nextDuration: getNextDuration(data.length)
        } as WindowStats;
      })
    )
  )
).subscribe(stats => {
  console.log(`Tamaño de ventana: ${stats.count} eventos, período siguiente: ${stats.nextDuration}ms`);
});
```

## ⚠️ Puntos de atención

### 1. Gestión de suscripciones de ventanas

Como cada ventana es un Observable independiente, es necesario suscribirse explícitamente o aplanar con `mergeAll()`, etc.

```ts
source$.pipe(
  windowWhen(closing)
).subscribe(window$ => {
  // Si no se suscribe a la ventana misma, no fluyen los valores
  window$.subscribe(value => {
    console.log('Valor:', value);
  });
});
```

### 2. Necesidad de devolver nuevo Observable cada vez

La función `closingSelector` **debe devolver un nuevo Observable cada vez**. Si devuelve la misma instancia, no funcionará correctamente.

```ts
// ❌ Mal ejemplo: reutilizar misma instancia de Observable
const closingObservable = timer(1000);

source$.pipe(
  windowWhen(() => closingObservable) // ¡No funciona desde la segunda vez!
).subscribe();

// ✅ Buen ejemplo: generar nuevo Observable cada vez
source$.pipe(
  windowWhen(() => timer(1000)) // Generar nuevo timer cada vez
).subscribe();
```

### 3. Atención a complejización de condición de fin

Si la condición de fin se vuelve demasiado compleja, la depuración se vuelve difícil.

```ts
// Ejemplo demasiado complejo
let counter = 0;
source$.pipe(
  windowWhen(() => {
    counter++;
    const duration = counter % 3 === 0 ? 500 :
                     counter % 2 === 0 ? 1000 : 1500;
    return timer(duration);
  })
).subscribe();
// Depuración difícil
```

## 🆚 Comparación de operadores de la familia window

| Operador | Control | Período de ventana | Caso de uso |
|:---|:---|:---|:---|
| `window` | Emisión de otro Observable | Continuo | División basada en eventos |
| `windowTime` | Tiempo fijo | Continuo | División basada en tiempo |
| `windowCount` | Cantidad fija | Continuo | División basada en cantidad |
| `windowToggle` | Control separado inicio/fin | Superposición posible | Condiciones inicio/fin complejas |
| `windowWhen` | **Solo control dinámico de fin** | **Continuo** | **Procesamiento de ventana adaptativo** |

## 📚 Operadores relacionados

- [`bufferWhen`](./bufferWhen) - Agrupar valores como array (versión de array de windowWhen)
- [`window`](./window) - División de ventana según timing de otro Observable
- [`windowTime`](./windowTime) - División de ventana basada en tiempo
- [`windowCount`](./windowCount) - División de ventana basada en cantidad
- [`windowToggle`](./windowToggle) - Control de ventana con Observable de inicio/fin

## Resumen

El operador `windowWhen` es una herramienta conveniente que controla dinámicamente la condición de fin y realiza procesamiento de ventana continuo.

- ✅ Control dinámico de condición de fin posible
- ✅ Procesamiento de ventana continuo (sin perder datos)
- ✅ Puede ajustar siguiente ventana según resultado anterior
- ⚠️ Requiere gestión de suscripciones
- ⚠️ Necesidad de devolver nuevo Observable cada vez
- ⚠️ Atención a no complicar demasiado la condición de fin
