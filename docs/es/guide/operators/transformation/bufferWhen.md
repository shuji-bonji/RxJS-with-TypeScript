---
description: bufferWhen es un operador de transformación de RxJS que controla dinámicamente la condición de fin para agrupar valores y emitirlos como arrays. Realiza buffering continuo donde inmediatamente después de terminar un buffer comienza el siguiente, útil para procesamiento por lotes adaptativo, recopilación de logs, y agregación flexible de datos según la carga. La inferencia de tipos de TypeScript permite buffering dinámico con seguridad de tipos.
---

# bufferWhen - Buffer con control dinámico de fin

El operador `bufferWhen` **controla dinámicamente la condición de fin** para agrupar valores y emitirlos como arrays. Cuando un buffer termina, inmediatamente comienza el siguiente buffer, realizando un patrón de buffering continuo.


## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(500); // Emite un valor cada 0.5 segundos

// Condición de fin: después de 1 segundo
const closingSelector = () => interval(1000);

source$.pipe(
  bufferWhen(closingSelector),
  take(4)
).subscribe(console.log);
// Salida:
// [0]           (inicio 0s → fin 1s, solo valor 0)
// [1, 2, 3]     (inicio 1s → fin 2s, valores 1,2,3)
// [4, 5]        (inicio 2s → fin 3s, valores 4,5)
// [6, 7]        (inicio 3s → fin 4s, valores 6,7)
```

**Flujo de operación**:
1. El primer buffer comienza automáticamente
2. El Observable que devuelve `closingSelector()` emite un valor → El buffer termina, emite el array
3. **Inmediatamente comienza el siguiente buffer** (frecuentemente simultáneo con la emisión de source$)
4. Se repiten los pasos 2-3

> [!NOTE]
> El primer buffer es de 1 segundo hasta que `interval(1000)` emite su primer valor, por lo que solo contiene `[0]`. Desde el segundo en adelante, el inicio del buffer y la emisión de `source$` son simultáneos, por lo que contienen más valores.

[🌐 Documentación oficial de RxJS - `bufferWhen`](https://rxjs.dev/api/operators/bufferWhen)


## 🆚 Diferencias con bufferToggle

`bufferWhen` y `bufferToggle` son similares, pero **el método de control y el patrón de operación son significativamente diferentes**.

### Operación de bufferWhen

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Emite 0-11 cada 300ms

// bufferWhen: solo controla fin (el siguiente inicia inmediatamente después de terminar)
source$.pipe(
  bufferWhen(() => interval(1000))
).subscribe(console.log);
// Salida: [0, 1, 2], [3, 4, 5], [6, 7, 8, 9], [10, 11]
//
// Timeline:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms 3600ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//  [----------1s----------][----------1s----------][----------1s----------][-----1s-----]
//   Buffer1(0-2)           Buffer2(3-5)           Buffer3(6-9)          Buffer4(10-11)
//   Continuo・Sin superposición・Inicio inmediato del siguiente
```

### Operación de bufferToggle

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Emite 0-11 cada 300ms

// bufferToggle: control separado de inicio y fin (superposición posible)
const opening$ = interval(1000); // Inicia cada segundo
const closing = () => interval(800); // Termina 800ms después del inicio

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Salida: [3, 4, 5], [6, 7, 8], [9, 10, 11]
//
// Timeline:
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//        ----Inicio1(1000ms)----[---Fin 800ms después(1800ms)---]
//                        3      4      5
//                        └→ [3,4,5]
//                    ----Inicio2(2000ms)----[---Fin 800ms después(2800ms)---]
//                                            6      7      8
//                                            └→ [6,7,8]
//                              ----Inicio3(3000ms)----[---Fin 800ms después(3800ms)---]
//                                                      9      10     11
//                                                      └→ [9,10,11]
//  Espera trigger de inicio, los períodos son independientes (0-2 no se incluyen porque están antes del inicio del buffer)
```

### Diferencias principales

| Operador | Control de inicio | Control de fin | Período de buffer | Característica |
|---|---|---|---|---|
| `bufferWhen(closing)` | Automático (inmediatamente después de terminar) | Dinámico | Continuo | Sin espacios entre buffers |
| `bufferToggle(open$, close)` | Observable independiente | Dinámico | Independiente・Superposición posible | Con espacios entre buffers |

**Puntos de diferenciación**:
- **`bufferWhen`**: Cuando deseas bufferizar todos los datos continuamente sin pérdidas (recopilación de logs, agregación de datos, etc.)
- **`bufferToggle`**: Cuando deseas recopilar datos solo durante períodos específicos (durante horario comercial, mientras se mantiene presionado botón, etc.)

> [!TIP]
> - **Buffering continuo** (sin perder datos) → `bufferWhen`
> - **Buffering por período limitado** (control explícito de inicio/fin) → `bufferToggle`


## 💡 Patrones típicos de uso

1. **Recopilación de datos a intervalos de tiempo dinámicos**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Datos del sensor
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       temperature: 20 + Math.random() * 10
     }))
   );

   // Condición de fin: cambia dinámicamente según la temperatura anterior
   let previousAvgTemp = 25;

   sensorData$.pipe(
     bufferWhen(() => {
       // Buffer a intervalos más cortos cuanto mayor sea la temperatura
       const duration = previousAvgTemp > 27 ? 500 : 1000;
       return timer(duration);
     })
   ).subscribe(data => {
     const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
     previousAvgTemp = avgTemp;
     console.log(`Temperatura promedio: ${avgTemp.toFixed(1)}°C, Número de muestras: ${data.length}`);
   });
   ```

2. **Procesamiento por lotes adaptativo según la carga**
   ```ts
   import { fromEvent, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   interface Task {
     id: number;
     timestamp: number;
   }

   // Stream de tareas
   let taskCounter = 0;
   const tasks$ = fromEvent(document, 'click').pipe(
     map(() => ({
       id: taskCounter++,
       timestamp: Date.now()
     } as Task))
   );

   // Ajusta el siguiente período de buffer según el tamaño del buffer
   tasks$.pipe(
     bufferWhen(() => timer(2000))
   ).subscribe(bufferedTasks => {
     if (bufferedTasks.length > 0) {
       console.log(`Procesamiento por lotes: ${bufferedTasks.length} tareas`);
       console.log('IDs de tareas:', bufferedTasks.map(t => t.id));

       // Determina dinámicamente el siguiente período de buffer
       // (En realidad, mover esta lógica dentro de la función bufferWhen)
     }
   });
   ```

3. **Muestreo a intervalos aleatorios**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferWhen, map } from 'rxjs';

   // Stream de datos
   const data$ = interval(100).pipe(
     map(i => ({
       value: Math.sin(i / 10) * 100,
       timestamp: Date.now()
     }))
   );

   // Buffer a intervalos aleatorios (500ms〜2000ms)
   data$.pipe(
     bufferWhen(() => {
       const randomDelay = 500 + Math.random() * 1500;
       return timer(randomDelay);
     })
   ).subscribe(samples => {
     const avg = samples.reduce((sum, s) => sum + s.value, 0) / samples.length;
     console.log(`Número de muestras: ${samples.length}, Valor promedio: ${avg.toFixed(2)}`);
   });
   ```


## 🧠 Ejemplo de código práctico (recopilación de logs según carga)

Un ejemplo que cambia dinámicamente la frecuencia de recopilación de logs según la carga del sistema.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { bufferWhen, map, share } from 'rxjs';

// Crear elementos de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Sistema de recopilación de logs adaptativo';
container.appendChild(title);

const loadButton = document.createElement('button');
loadButton.textContent = 'Generar carga';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
status.textContent = 'Carga baja: recopilación cada 5 segundos';
container.appendChild(status);

const logDisplay = document.createElement('pre');
logDisplay.style.marginTop = '10px';
logDisplay.style.padding = '10px';
logDisplay.style.backgroundColor = '#f9f9f9';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Stream de logs (generados constantemente)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    message: `Log message ${logCounter}`,
    timestamp: new Date()
  })),
  share()
);

// Contador de carga (incrementa con clic de botón)
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// Reduce la carga cada 30 segundos
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getBufferInterval(loadLevel);
  const loadText = loadLevel === 0 ? 'Carga baja' :
                   loadLevel <= 2 ? 'Carga media' : 'Carga alta';
  status.textContent = `${loadText} (Nivel ${loadLevel}): recopilación cada ${interval / 1000} segundos`;
  status.style.backgroundColor =
    loadLevel === 0 ? '#d4edda' :
    loadLevel <= 2 ? '#fff3cd' : '#f8d7da';
}

function getBufferInterval(load: number): number {
  // Intervalo más corto cuanto mayor sea la carga
  switch (load) {
    case 0: return 5000;  // 5 segundos
    case 1: return 3000;  // 3 segundos
    case 2: return 2000;  // 2 segundos
    case 3: return 1000;  // 1 segundo
    case 4: return 500;   // 0.5 segundos
    default: return 300;  // 0.3 segundos
  }
}

// Buffering adaptativo
logs$.pipe(
  bufferWhen(() => timer(getBufferInterval(loadLevel)))
).subscribe(bufferedLogs => {
  if (bufferedLogs.length > 0) {
    const errors = bufferedLogs.filter(log => log.level === 'ERROR').length;
    const timestamp = new Date().toLocaleTimeString();

    const summary = `[${timestamp}] Recopilación: ${bufferedLogs.length} registros (Errores: ${errors} registros)\n`;
    logDisplay.textContent = summary + logDisplay.textContent;

    console.log('Logs recopilados:', bufferedLogs);
  }
});
```


## 📋 Uso con seguridad de tipos

Ejemplo de implementación con seguridad de tipos utilizando genéricos en TypeScript.

```ts
import { Observable, interval, timer } from 'rxjs';
import { bufferWhen, map } from 'rxjs';

interface MetricData {
  value: number;
  timestamp: Date;
  source: string;
}

interface BufferConfig {
  minDuration: number;
  maxDuration: number;
  adaptive: boolean;
}

class AdaptiveBuffer<T> {
  constructor(private config: BufferConfig) {}

  private getNextBufferDuration(previousCount: number): number {
    if (!this.config.adaptive) {
      return this.config.minDuration;
    }

    // Ajusta el siguiente período de buffer según la cantidad de datos
    const ratio = Math.min(previousCount / 10, 1);
    const duration =
      this.config.minDuration +
      (this.config.maxDuration - this.config.minDuration) * (1 - ratio);

    return Math.floor(duration);
  }

  apply(source$: Observable<T>): Observable<T[]> {
    let previousCount = 0;

    return source$.pipe(
      bufferWhen(() => {
        const duration = this.getNextBufferDuration(previousCount);
        return timer(duration);
      }),
      map(buffer => {
        previousCount = buffer.length;
        return buffer;
      })
    );
  }
}

// Ejemplo de uso
const metricsStream$ = interval(300).pipe(
  map(i => ({
    value: Math.random() * 100,
    timestamp: new Date(),
    source: `sensor-${i % 3}`
  } as MetricData))
);

const buffer = new AdaptiveBuffer<MetricData>({
  minDuration: 1000,  // Mínimo 1 segundo
  maxDuration: 5000,  // Máximo 5 segundos
  adaptive: true      // Adaptativo
});

buffer.apply(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avg = metrics.reduce((sum, m) => sum + m.value, 0) / metrics.length;
    console.log(`Tamaño de buffer: ${metrics.length}, Valor promedio: ${avg.toFixed(2)}`);
  }
});
```


## 🎯 Comparación con otros operadores de buffer

```ts
import { interval, timer, Subject } from 'rxjs';
import { buffer, bufferTime, bufferCount, bufferWhen, bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9

// 1. buffer: trigger fijo
const trigger$ = new Subject<void>();
source$.pipe(buffer(trigger$)).subscribe(console.log);
setInterval(() => trigger$.next(), 1000);
// Salida: [0, 1, 2], [3, 4, 5], ... (en el momento del trigger)

// 2. bufferTime: intervalo de tiempo fijo
source$.pipe(bufferTime(1000)).subscribe(console.log);
// Salida: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 3. bufferCount: cantidad fija
source$.pipe(bufferCount(3)).subscribe(console.log);
// Salida: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 4. bufferWhen: control dinámico de fin (continuo)
source$.pipe(
  bufferWhen(() => timer(1000))
).subscribe(console.log);
// Salida: [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 5. bufferToggle: control independiente de inicio y fin (superposición posible)
const opening$ = interval(1000);
const closing = () => timer(800);
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Salida: [3, 4, 5], [6, 7, 8]
```

| Operador | Trigger | Control dinámico | Superposición | Caso de uso |
|---|---|---|---|---|
| `buffer` | Observable externo | ❌ | ❌ | Basado en eventos |
| `bufferTime` | Tiempo fijo | ❌ | ❌ | Agregación periódica |
| `bufferCount` | Cantidad fija | ❌ | ❌ | Procesamiento cuantitativo |
| `bufferWhen` | Dinámico (solo fin) | ✅ | ❌ | Procesamiento por lotes adaptativo |
| `bufferToggle` | Dinámico (inicio y fin) | ✅ | ✅ | Gestión de períodos complejos |


## ⚠️ Errores comunes

> [!WARNING]
> La función de condición de fin de `bufferWhen` **debe devolver un nuevo Observable cada vez**. Si devuelve la misma instancia de Observable, no funcionará correctamente.

### Incorrecto: Devolver la misma instancia de Observable

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ❌ Mal ejemplo: reutilizar la misma instancia de Observable
const closingObservable = timer(1000);

source$.pipe(
  bufferWhen(() => closingObservable) // ¡No funciona desde la segunda vez!
).subscribe(console.log);
// Solo se emite el primer buffer, después no se emite nada
```

### Correcto: Devolver un nuevo Observable cada vez

```ts
import { interval, timer } from 'rxjs';
import { bufferWhen } from 'rxjs';

const source$ = interval(500);

// ✅ Buen ejemplo: generar un nuevo Observable cada vez
source$.pipe(
  bufferWhen(() => timer(1000)) // Generar un nuevo timer cada vez
).subscribe(console.log);
// Salida: [0, 1], [2, 3], [4, 5], ...
```

> [!IMPORTANT]
> La función `closingSelector` se **siempre llama** cada vez que termina el buffer anterior, y se espera que devuelva un nuevo Observable.


## 🎓 Resumen

### Cuándo usar bufferWhen
- ✅ Cuando deseas controlar dinámicamente la condición de fin
- ✅ Cuando necesitas períodos de buffering continuos
- ✅ Cuando deseas ajustar el siguiente período basándote en el resultado del buffer anterior
- ✅ Cuando deseas implementar procesamiento por lotes adaptativo

### Cuándo usar bufferToggle
- ✅ Cuando deseas controlar inicio y fin de forma independiente
- ✅ Cuando hay posibilidad de que los períodos de buffer se superpongan
- ✅ Cuando hay eventos claros de inicio/fin como mientras se mantiene presionado un botón

### Cuándo usar bufferTime
- ✅ Cuando es suficiente buffering a intervalos de tiempo fijos
- ✅ Cuando se requiere una implementación simple

### Puntos de atención
- ⚠️ `closingSelector` debe devolver un nuevo Observable cada vez
- ⚠️ Si la condición de fin se vuelve demasiado compleja, la depuración se vuelve difícil
- ⚠️ En control adaptativo, las pruebas son importantes para evitar comportamientos inesperados


## 🚀 Próximos pasos

- **[buffer](./buffer)** - Aprender buffering básico
- **[bufferTime](./bufferTime)** - Aprender buffering basado en tiempo
- **[bufferCount](./bufferCount)** - Aprender buffering basado en cantidad
- **[bufferToggle](./bufferToggle)** - Aprender buffering con control independiente de inicio y fin
- **[Ejemplos prácticos de operadores de transformación](./practical-use-cases)** - Aprender casos de uso reales
