---
description: El operador bufferToggle es un operador de buffering avanzado que controla los triggers de inicio y fin con Observables separados, permitiendo gestionar múltiples períodos de buffering de forma independiente.
---

# bufferToggle - Buffer con Trigger

El operador `bufferToggle` controla el **trigger de inicio** y el **trigger de fin** con Observables separados, agrupando valores y emitiéndolos como arrays. Es un operador de buffering avanzado que puede gestionar múltiples períodos de buffering simultáneamente.


## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // Emite un valor cada 0.5 segundos

// Trigger de inicio: cada 2 segundos
const opening$ = interval(2000);

// Trigger de fin: 1 segundo después del inicio
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Salida:
// [3, 4, 5]     (inicia a los 2s, termina a los 3s)
// [7, 8, 9]     (inicia a los 4s, termina a los 5s)
// [11, 12, 13]  (inicia a los 6s, termina a los 7s)
```

**Flujo de operación**:
1. `opening$` emite un valor → Comienza el buffering
2. El Observable que devuelve `closing()` emite un valor → Termina el buffering, emite el array
3. Múltiples períodos de buffering pueden superponerse

[🌐 Documentación oficial de RxJS - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)


## 🆚 Comparación con otros operadores de buffer

`bufferToggle` se caracteriza por **controlar inicio y fin de forma independiente** en comparación con otros operadores de buffer.

### Comparación de cada operador

| Operador | Trigger | Característica | Caso de uso |
|---|---|---|---|
| `buffer(trigger$)` | Un solo Observable | Simple | Buffering basado en eventos |
| `bufferTime(ms)` | Tiempo | Periódico | Agregación de datos a intervalos fijos |
| `bufferCount(n)` | Cantidad | Cuantitativo | Procesamiento en unidades de N elementos |
| `bufferToggle(open$, close)` | Control separado de inicio y fin | Flexible | Gestión de períodos complejos |

### Comparación con ejemplos de código

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // Emite 0-9 cada 300ms

// bufferToggle: control independiente de inicio y fin
const opening$ = interval(1000); // Inicia cada segundo
const closing = () => interval(500); // Termina 500ms después del inicio

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Salida: [3, 4], [6, 7], [9]
//
// Timeline:
// 0ms  300ms 600ms 900ms 1200ms 1500ms 1800ms 2100ms 2400ms 2700ms
// 0    1     2     3     4      5      6      7      8      9
//                  [inicio      fin]   [inicio      fin]   [inicio fin]
//                  └→ [3,4]            └→ [6,7]            └→ [9]
```

**Diferenciación con otros operadores**:
- **`buffer`** → Emite buffer cada vez que el Observable trigger emite un valor
- **`bufferTime`** → Emite buffer automáticamente a intervalos de tiempo fijos
- **`bufferCount`** → Emite buffer cuando se acumula la cantidad especificada
- **`bufferToggle`** → Control separado de inicio y fin, períodos superpuestos posibles

> [!TIP]
> Para más detalles sobre cada operador, consulta [buffer](./buffer), [bufferTime](./bufferTime), [bufferCount](./bufferCount).


## 💡 Patrones típicos de uso

1. **Recopilación de datos durante horario comercial**
   ```ts
   import { interval, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Datos del sensor (obtenidos constantemente)
   const sensorData$ = interval(100).pipe(
     map(() => ({
       timestamp: Date.now(),
       value: Math.random() * 100
     }))
   );

   // Apertura de negocio: 9:00 (simulación: después de 2 segundos)
   const businessOpen$ = timer(2000, 10000); // Después de 2s, luego cada 10s

   // Cierre de negocio: 5 segundos después del inicio
   const businessClose = () => timer(5000);

   sensorData$.pipe(
     bufferToggle(businessOpen$, businessClose)
   ).subscribe(data => {
     console.log(`Datos durante horario comercial: ${data.length} registros`);
     console.log(`Promedio: ${(data.reduce((sum, d) => sum + d.value, 0) / data.length).toFixed(2)}`);
   });
   ```

2. **Registro de eventos mientras se mantiene presionado el botón**
   ```ts
   import { fromEvent, interval } from 'rxjs';
   import { bufferToggle, map, take } from 'rxjs';

   const button = document.createElement('button');
   button.textContent = 'Mantener presionado';
   document.body.appendChild(button);

   const display = document.createElement('div');
   display.style.marginTop = '10px';
   document.body.appendChild(display);

   // Stream de datos
   const data$ = interval(100).pipe(
     map(i => ({ id: i, timestamp: Date.now() }))
   );

   // Inicio: mousedown
   const mouseDown$ = fromEvent(button, 'mousedown');

   // Fin: mouseup (hasta mouseup que ocurre desde mousedown)
   const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

   data$.pipe(
     bufferToggle(mouseDown$, mouseUp)
   ).subscribe(events => {
     display.textContent = `Eventos registrados mientras se mantenía presionado: ${events.length} eventos`;
     console.log('Eventos registrados:', events);
   });
   ```

3. **Registro de acciones de usuario activo**
   ```ts
   import { fromEvent, merge, timer } from 'rxjs';
   import { bufferToggle, map } from 'rxjs';

   // Acciones del usuario
   const clicks$ = fromEvent(document, 'click').pipe(
     map(() => ({ type: 'click' as const, timestamp: Date.now() }))
   );

   const scrolls$ = fromEvent(window, 'scroll').pipe(
     map(() => ({ type: 'scroll' as const, timestamp: Date.now() }))
   );

   const keypresses$ = fromEvent(document, 'keypress').pipe(
     map(() => ({ type: 'keypress' as const, timestamp: Date.now() }))
   );

   const actions$ = merge(clicks$, scrolls$, keypresses$);

   // Inicio de estado activo: primera acción
   const activeStart$ = actions$;

   // Fin de estado activo: 5 segundos sin acciones
   const activeEnd = () => timer(5000);

   actions$.pipe(
     bufferToggle(activeStart$, activeEnd)
   ).subscribe(bufferedActions => {
     console.log(`Sesión activa: ${bufferedActions.length} acciones`);
     const summary = bufferedActions.reduce((acc, action) => {
       acc[action.type] = (acc[action.type] || 0) + 1;
       return acc;
     }, {} as Record<string, number>);
     console.log('Desglose:', summary);
   });
   ```


## 🧠 Ejemplo de código práctico (gestión de período de descarga)

Un ejemplo que gestiona el período de descarga de datos con botones de inicio y detención.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { bufferToggle, map, take } from 'rxjs';

// Crear elementos de UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Gestión de descarga de datos';
container.appendChild(title);

const startButton = document.createElement('button');
startButton.textContent = 'Iniciar';
container.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Detener';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
container.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'En espera...';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Stream de datos (genera datos de descarga cada segundo)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// Triggers de inicio y fin
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = 'Detenido';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'Descargando...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// Buffering
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Descarga completada</strong><br>
    Cantidad: ${downloads.length} archivos<br>
    Tamaño total: ${(totalSize / 1024).toFixed(2)} MB<br>
    Tamaño promedio: ${avgSize.toFixed(0)} KB
  `;

  console.log('Datos de descarga:', downloads);
});
```


## 🎯 Períodos de buffer superpuestos

Una característica de `bufferToggle` es que puede gestionar múltiples períodos de buffering simultáneamente.

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Inicio: cada segundo
const opening$ = interval(1000);

// Fin: 1.5 segundos después del inicio
const closing = () => interval(1500);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Salida:
// [4, 5, 6]        (inicia a 1s → termina a 2.5s)
// [9, 10, 11, 12]  (inicia a 2s → termina a 3.5s) ※Superposición parcial
// [14, 15, 16, 17] (inicia a 3s → termina a 4.5s)
```

**Timeline**:
```
Fuente:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Inicio:    ----1s----2s----3s----4s
Período1:  [------1.5s-----]
           └→ Salida: [4,5,6]
Período2:         [------1.5s-----]
                  └→ Salida: [9,10,11,12]
Período3:                [------1.5s-----]
                         └→ Salida: [14,15,16,17]
```


## 📋 Uso con seguridad de tipos

Ejemplo de implementación con seguridad de tipos utilizando genéricos en TypeScript.

```ts
import { Observable, Subject, interval } from 'rxjs';
import { bufferToggle, map } from 'rxjs';

interface MetricData {
  timestamp: Date;
  cpu: number;
  memory: number;
}

interface SessionControl {
  start$: Observable<void>;
  stop$: Observable<void>;
}

class MetricsCollector {
  private startSubject = new Subject<void>();
  private stopSubject = new Subject<void>();

  start(): void {
    this.startSubject.next();
  }

  stop(): void {
    this.stopSubject.next();
  }

  collectMetrics(source$: Observable<MetricData>): Observable<MetricData[]> {
    return source$.pipe(
      bufferToggle(
        this.startSubject,
        () => this.stopSubject
      )
    );
  }
}

// Ejemplo de uso
const metricsStream$ = interval(500).pipe(
  map(() => ({
    timestamp: new Date(),
    cpu: Math.random() * 100,
    memory: Math.random() * 100
  } as MetricData))
);

const collector = new MetricsCollector();

collector.collectMetrics(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avgCpu = metrics.reduce((sum, m) => sum + m.cpu, 0) / metrics.length;
    const avgMemory = metrics.reduce((sum, m) => sum + m.memory, 0) / metrics.length;
    console.log(`Período de recopilación: ${metrics.length} registros`);
    console.log(`CPU promedio: ${avgCpu.toFixed(1)}%`);
    console.log(`Memoria promedio: ${avgMemory.toFixed(1)}%`);
  }
});

// Inicia después de 3 segundos
setTimeout(() => {
  console.log('Iniciar recopilación');
  collector.start();
}, 3000);

// Detiene después de 6 segundos
setTimeout(() => {
  console.log('Detener recopilación');
  collector.stop();
}, 6000);
```


## 🔄 Diferencias con bufferWhen

`bufferToggle` y `bufferWhen` son similares, pero difieren en el método de control.

```ts
import { interval, timer } from 'rxjs';
import { bufferToggle, bufferWhen } from 'rxjs';

const source$ = interval(200);

// bufferToggle: controla inicio y fin por separado
source$.pipe(
  bufferToggle(
    interval(1000),          // Trigger de inicio
    () => timer(500)         // Trigger de fin (500ms después del inicio)
  )
).subscribe(console.log);

// bufferWhen: solo controla el momento de fin (el siguiente inicia inmediatamente después de terminar)
source$.pipe(
  bufferWhen(() => timer(1000)) // Buffer cada segundo
).subscribe(console.log);
```

| Operador | Control | Período de buffer | Caso de uso |
|---|---|---|---|
| `bufferToggle(open$, close)` | Control separado de inicio y fin | Superposición posible | Condiciones complejas de inicio/fin |
| `bufferWhen(closing)` | Solo controla fin | Continuo | Buffer periódico simple |


## ⚠️ Errores comunes

> [!WARNING]
> `bufferToggle` puede gestionar múltiples períodos de buffer simultáneamente, pero si el trigger de inicio se dispara con frecuencia, existirán muchos buffers simultáneamente, consumiendo memoria.

### Incorrecto: Trigger de inicio demasiado frecuente

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ❌ Mal ejemplo: inicio cada 100ms, fin después de 5 segundos
const opening$ = interval(100); // Demasiado frecuente
const closing = () => interval(5000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Posibilidad de 50 buffers simultáneos → Riesgo de memoria
```

### Correcto: Establecer intervalo apropiado

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(100);

// ✅ Buen ejemplo: inicio a intervalo apropiado
const opening$ = interval(2000); // Cada 2 segundos
const closing = () => interval(1000); // Buffer durante 1 segundo

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Máximo 1-2 buffers simultáneos solamente
```


## 🎓 Resumen

### Cuándo usar bufferToggle
- ✅ Cuando deseas controlar inicio y fin de forma independiente
- ✅ Cuando deseas recopilar datos por período limitado como mientras se mantiene presionado un botón
- ✅ Cuando deseas gestionar múltiples períodos de buffering simultáneamente
- ✅ Recopilación de datos con condiciones complejas como solo durante horario comercial

### Cuándo usar buffer/bufferTime/bufferCount
- ✅ Cuando es suficiente un buffering periódico simple
- ✅ Cuando puede controlarse con un solo trigger

### Cuándo usar bufferWhen
- ✅ Cuando solo deseas controlar dinámicamente la condición de fin
- ✅ Cuando necesitas períodos de buffering continuos

### Puntos de atención
- ⚠️ Si el trigger de inicio es frecuente, existirán muchos buffers simultáneamente consumiendo memoria
- ⚠️ Posibilidad de que los períodos de buffering se superpongan
- ⚠️ El control complejo puede hacer difícil la depuración en algunos casos


## 🚀 Próximos pasos

- **[buffer](./buffer)** - Aprender buffering básico
- **[bufferTime](./bufferTime)** - Aprender buffering basado en tiempo
- **[bufferCount](./bufferCount)** - Aprender buffering basado en cantidad
- **[bufferWhen](https://rxjs.dev/api/operators/bufferWhen)** - Aprender control dinámico de fin (documentación oficial)
- **[Ejemplos prácticos de operadores de transformación](./practical-use-cases)** - Aprender casos de uso reales
