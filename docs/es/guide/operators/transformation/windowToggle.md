---
description: windowToggle es un operador de transformación avanzado de RxJS que controla triggers de inicio y fin con Observables separados, permitiendo gestionar múltiples períodos de ventana de forma independiente. Es ideal para escenarios que requieren gestión dinámica de períodos como recopilación de datos durante horario comercial o registro de eventos mientras se presiona un botón. La inferencia de tipos de TypeScript permite procesamiento de división de ventanas con seguridad de tipos.
---

# windowToggle - Ventana con Trigger

El operador `windowToggle` controla el **trigger de inicio** y el **trigger de fin** con Observables separados, emitiendo cada período como un nuevo Observable. Es un operador de ventana avanzado que puede gestionar múltiples períodos de ventana simultáneamente.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500); // Emitir valores cada 0.5 segundos

// Trigger de inicio: cada 2 segundos
const opening$ = interval(2000);

// Trigger de fin: 1 segundo después del inicio
const closing = () => interval(1000);

source$.pipe(
  windowToggle(opening$, closing),
  mergeAll()
).subscribe(value => {
  console.log('Valor en ventana:', value);
});

// Inicia en 2s, termina en 3s → Valores: 4, 5
// Inicia en 4s, termina en 5s → Valores: 8, 9
// Inicia en 6s, termina en 7s → Valores: 12, 13
```

**Flujo de operación**:
1. `opening$` emite un valor → Inicio de ventana
2. El Observable devuelto por `closing()` emite un valor → Fin de ventana
3. Múltiples períodos de ventana pueden superponerse

[🌐 Documentación oficial de RxJS - `windowToggle`](https://rxjs.dev/api/operators/windowToggle)

## 💡 Patrones de uso típicos

- Recopilación de datos durante horario comercial
- Registro de eventos durante presión de botón
- Seguimiento de acciones durante sesión activa
- Procesamiento de streams que requiere gestión dinámica de períodos

## 🔍 Diferencia con bufferToggle

| Operador | Salida | Caso de uso |
|:---|:---|:---|
| `bufferToggle` | **Array (T[])** | Procesar valores agrupados juntos |
| `windowToggle` | **Observable\<T>** | Procesamiento de stream diferente por grupo |

```ts
import { interval } from 'rxjs';
import { bufferToggle, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(500);
const opening$ = interval(2000);
const closing = () => interval(1000);

// bufferToggle - emite como array
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Salida: Buffer (array): [4, 5]
});

// windowToggle - emite como Observable
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  console.log('Ventana (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valor en ventana:', value);
  });
});
```

## 🧠 Ejemplo de código práctico 1: Registro de eventos durante presión de botón

Ejemplo de registrar datos desde mousedown hasta mouseup.

```ts
import { fromEvent, interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

// Crear botón
const button = document.createElement('button');
button.textContent = 'Mantener';
document.body.appendChild(button);

// Área de salida
const display = document.createElement('div');
display.style.marginTop = '10px';
document.body.appendChild(display);

// Stream de datos (cada 100ms)
const data$ = interval(100);

// Inicio: mousedown
const mouseDown$ = fromEvent(button, 'mousedown');

// Fin: mouseup
const mouseUp = () => fromEvent(document, 'mouseup').pipe(take(1));

data$.pipe(
  windowToggle(mouseDown$, mouseUp),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(events => {
  display.textContent = `Eventos registrados durante mantención: ${events.length} eventos`;
  console.log('Datos registrados:', events);
});
```

## 🎯 Ejemplo de código práctico 2: Recopilación de datos durante horario comercial

Ejemplo de recopilar datos de sensores desde la apertura hasta el cierre del negocio.

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, mergeMap, scan, map } from 'rxjs';

// Datos de sensor (adquisición continua)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10, // 20-30 grados
    humidity: 40 + Math.random() * 20     // 40-60%
  }))
);

// Apertura del negocio: después de 2s, luego cada 10s
const businessOpen$ = timer(2000, 10000);

// Cierre del negocio: 5 segundos después del inicio
const businessClose = () => timer(5000);

let sessionNumber = 0;

sensorData$.pipe(
  windowToggle(businessOpen$, businessClose),
  mergeMap(window$ => {
    const current = ++sessionNumber;
    console.log(`Inicio de sesión comercial ${current}`);

    // Calcular información estadística de cada ventana
    return window$.pipe(
      scan((stats, data) => ({
        count: stats.count + 1,
        totalTemp: stats.totalTemp + data.temperature,
        totalHumidity: stats.totalHumidity + data.humidity
      }), { count: 0, totalTemp: 0, totalHumidity: 0 }),
      map(stats => ({
        session: current,
        count: stats.count,
        avgTemp: stats.totalTemp / stats.count,
        avgHumidity: stats.totalHumidity / stats.count
      }))
    );
  })
).subscribe(stats => {
  console.log(`Sesión ${stats.session}: ${stats.count} muestras`);
  console.log(`  Temperatura promedio: ${stats.avgTemp.toFixed(1)}°C`);
  console.log(`  Humedad promedio: ${stats.avgHumidity.toFixed(1)}%`);
});
```

## 🎯 Ejemplo de uso: Gestión de período de descarga

Ejemplo de gestión de período de descarga de datos con botones de inicio y parada.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { windowToggle, mergeMap, toArray, map } from 'rxjs';

// Crear elementos UI
const startButton = document.createElement('button');
startButton.textContent = 'Iniciar';
document.body.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Detener';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
document.body.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'En espera...';
document.body.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
document.body.appendChild(result);

// Stream de datos (generar datos de descarga cada segundo)
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

// Gestión de ventanas
downloadData$.pipe(
  windowToggle(start$, () => stop$),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Descarga completa</strong><br>
    Cantidad: ${downloads.length} archivos<br>
    Tamaño total: ${(totalSize / 1024).toFixed(2)} MB<br>
    Tamaño promedio: ${avgSize.toFixed(0)} KB
  `;
});
```

## 🎯 Períodos de ventana superpuestos

Como característica de `windowToggle`, puede gestionar múltiples períodos de ventana simultáneamente.

```ts
import { interval } from 'rxjs';
import { windowToggle, mergeMap, toArray, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Inicio: cada segundo
const opening$ = interval(1000);

// Fin: 1.5 segundos después del inicio
const closing = () => interval(1500);

source$.pipe(
  windowToggle(opening$, closing),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Ventana:', values);
});

// Salida:
// Ventana: [4, 5, 6, 7]       (Inicia en 1s → termina en 2.5s)
// Ventana: [9, 10, 11, 12]    (Inicia en 2s → termina en 3.5s)
// Ventana: [14, 15, 16, 17]   (Inicia en 3s → termina en 4.5s)
```

**Timeline**:
```
Fuente:    0--1--2--3--4--5--6--7--8--9--10-11-12-13-14-15-16-17-18-19
Inicio:    ----1s----2s----3s----4s
Período1:  [------1.5s-----]
           └→ Ventana1: [4,5,6,7]
Período2:         [------1.5s-----]
                  └→ Ventana2: [9,10,11,12]
Período3:                [------1.5s-----]
                         └→ Ventana3: [14,15,16,17]
```

## ⚠️ Puntos de atención

### 1. Gestión de suscripciones de ventanas

Como cada ventana es un Observable independiente, es necesario suscribirse explícitamente o aplanar con `mergeAll()`, etc.

```ts
source$.pipe(
  windowToggle(opening$, closing)
).subscribe(window$ => {
  // Si no se suscribe a la ventana misma, no fluyen los valores
  window$.subscribe(value => {
    console.log('Valor:', value);
  });
});
```

### 2. Atención a fugas de memoria

Si el trigger de inicio es demasiado frecuente, muchas ventanas existirán simultáneamente, consumiendo memoria.

```ts
// ❌ Mal ejemplo: inicio cada 100ms, fin después de 5s
const opening$ = interval(100); // Demasiado frecuente
const closing = () => interval(5000);

source$.pipe(
  windowToggle(opening$, closing)
).subscribe();
// Posibilidad de 50 ventanas simultáneas → Riesgo de memoria

// ✅ Buen ejemplo: configurar intervalo apropiado
const opening$ = interval(2000); // Cada 2 segundos
const closing = () => interval(1000); // 1 segundo
```

### 3. Superposición de períodos de ventana

Cuando los períodos de ventana se superponen, el mismo valor se incluye en múltiples ventanas. Confirmar si este es el comportamiento deseado.

```ts
// Con superposición
opening$ = interval(1000);    // Inicia cada segundo
closing = () => interval(1500); // 1.5 segundos

// Sin superposición
opening$ = interval(2000);    // Inicia cada 2 segundos
closing = () => interval(1000); // 1 segundo
```

## 🆚 Comparación de operadores de la familia window

| Operador | Control | Período de ventana | Caso de uso |
|:---|:---|:---|:---|
| `window` | Emisión de otro Observable | Continuo | División basada en eventos |
| `windowTime` | Tiempo fijo | Continuo | División basada en tiempo |
| `windowCount` | Cantidad fija | Continuo | División basada en cantidad |
| `windowToggle` | **Control separado inicio/fin** | **Superposición posible** | **Condiciones inicio/fin complejas** |
| `windowWhen` | Solo control de fin | Continuo | Control periódico simple |

## 🔄 Diferencia con windowWhen

```ts
import { interval, timer } from 'rxjs';
import { windowToggle, windowWhen, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowToggle: controlar inicio y fin por separado
source$.pipe(
  windowToggle(
    interval(1000),          // Trigger de inicio
    () => timer(500)         // Trigger de fin (500ms después del inicio)
  ),
  mergeAll()
).subscribe();

// windowWhen: controlar solo el timing de fin (siguiente inicia inmediatamente después del fin)
source$.pipe(
  windowWhen(() => timer(1000)), // Ventana cada segundo
  mergeAll()
).subscribe();
```

| Operador | Control | Período de ventana | Caso de uso |
|:---|:---|:---|:---|
| `windowToggle(open$, close)` | Control separado inicio/fin | Superposición posible | Condiciones inicio/fin complejas |
| `windowWhen(closing)` | Solo control de fin | Continuo | Ventana periódica simple |

## 📚 Operadores relacionados

- [`bufferToggle`](./bufferToggle) - Agrupar valores como array (versión de array de windowToggle)
- [`window`](./window) - División de ventana según timing de otro Observable
- [`windowTime`](./windowTime) - División de ventana basada en tiempo
- [`windowCount`](./windowCount) - División de ventana basada en cantidad
- [`windowWhen`](./windowWhen) - División de ventana con condición de cierre dinámica

## Resumen

El operador `windowToggle` es una herramienta avanzada que controla inicio y fin de forma independiente y puede procesar cada período como un Observable independiente.

- ✅ Control separado de inicio y fin posible
- ✅ Puede gestionar múltiples ventanas simultáneamente
- ✅ Puede aplicar diferentes procesos a cada ventana
- ⚠️ Requiere gestión de suscripciones
- ⚠️ Trigger de inicio frecuente consume memoria
- ⚠️ Atención a superposición de períodos de ventana
