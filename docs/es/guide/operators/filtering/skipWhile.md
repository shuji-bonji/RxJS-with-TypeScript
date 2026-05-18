---
description: "El operador skipWhile omite valores mientras se cumple la condición especificada y emite todos los valores posteriores a partir del punto en que la condición se convierte en falsa. Esto resulta útil cuando se desea controlar un flujo con una condición de inicio dinámica."
---

# skipWhile - saltar valores mientras se cumplen las condiciones

El operador `skipWhile` continúa saltando valores **mientras se cumple la condición especificada** y emite **todos los valores subsiguientes** desde el punto en que la condición se convierte en `false`.

## 🔰 Sintaxis básica y uso

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9
```

**Flujo de operación**:.
1. 0 emitido → `0 < 5` es `true` → saltar
2. 1 emitido → `1 < 5` es `true` → saltar
3. 2 emisiones → "2 < 5" es "verdadero" → omitir
4. 3 emisiones → `3 < 5` es `true` → omitir
5. 4 emisiones → `4 < 5` es `verdadero` → omitir
6. 5 salidas → `5 < 5` es `false` → salida comienza
7. 6 en adelante → toda la salida (condición no reevaluada)

[🌐 Documentación oficial de RxJS - `skipWhile`](https://rxjs.dev/api/operators/skipWhile)

## 💡 Patrón de utilización típico.

- **Skip inicial de datos no deseados**: excluir datos del periodo de calentamiento.
- **Skipping until a threshold is reached**: esperar hasta que se cumplan ciertas condiciones.
- Omitir cabeceras**: excluir CSV y otras cabeceras.
- **Omitir periodo de preparación**: esperar hasta que el sistema esté listo

## 🧠 Ejemplo práctico de código 1: Omitir el periodo de calentamiento del sensor

Este es un ejemplo de omisión de los datos iniciales hasta que el sensor se haya estabilizado.

```ts
import { interval } from 'rxjs';
import { skipWhile, map, take } from 'rxjs';

// UICreado por
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Supervisión del sensor de temperatura';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginBottom = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#fff3e0';
status.style.border = '1px solid #FF9800';
status.textContent = '🔄 Sensor en preparación...(la temperatura está20°Cpor encima, comienza la medición).';
container.appendChild(status);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

let isWarmedUp = false;

// Simulación del sensor de temperatura (calentándose lentamente)々(calentando gradualmente)
interval(500).pipe(
  take(20),
  map(i => {
    // Temperatura inicial baja, aumenta gradualmente々Aumenta lentamente
    const baseTemp = 15 + i * 0.5;
    const noise = (Math.random() - 0.5) * 2;
    return baseTemp + noise;
  }),
  skipWhile(temp => temp < 20) // 20°COmitir si es inferior a
).subscribe({
  next: temp => {
    // Actualización de estado cuando llega el primer valor
    if (!isWarmedUp) {
      isWarmedUp = true;
      status.textContent = '✅ Sensor listo (medición iniciada)';
      status.style.backgroundColor = '#e8f5e9';
      status.style.borderColor = '#4CAF50';
    }

    const log = document.createElement('div');
    log.style.padding = '5px';
    log.style.marginBottom = '3px';
    log.style.backgroundColor = temp > 25 ? '#ffebee' : '#f1f8e9';
    log.textContent = `[${new Date().toLocaleTimeString()}] Temperatura: ${temp.toFixed(1)}°C`;
    output.insertBefore(log, output.firstChild);

    // Máx.10Visualizar hasta
    while (output.children.length > 10) {
      output.removeChild(output.lastChild!);
    }
  },
  complete: () => {
    const summary = document.createElement('div');
    summary.style.marginTop = '10px';
    summary.style.padding = '10px';
    summary.style.backgroundColor = '#e3f2fd';
    summary.textContent = 'Medición finalizada';
    container.appendChild(summary);
  }
});
```

- Los datos se omiten mientras el sensor está por debajo de 20°C.
- A partir del momento en que la temperatura es superior a 20°C, se registran todos los datos.

## 🎯 Ejemplo práctico de código 2: Procesamiento de eventos después de la preparación.

Este es un ejemplo de omisión de eventos hasta que se complete la inicialización del sistema.

```ts
import { fromEvent, merge, Subject } from 'rxjs';
import { skipWhile, map, tap } from 'rxjs';

// UICreado por
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Sistema de procesamiento de eventos';
container.appendChild(title);

const initButton = document.createElement('button');
initButton.textContent = 'Inicialización completada';
initButton.style.marginRight = '10px';
container.appendChild(initButton);

const eventButton = document.createElement('button');
eventButton.textContent = 'Encendido del evento';
container.appendChild(eventButton);

const statusDiv = document.createElement('div');
statusDiv.style.marginTop = '10px';
statusDiv.style.padding = '10px';
statusDiv.style.backgroundColor = '#ffebee';
statusDiv.style.border = '1px solid #f44336';
statusDiv.innerHTML = '<strong>⏸️ Sistema no inicializado</strong><br>Evento omitido';
container.appendChild(statusDiv);

const eventLog = document.createElement('div');
eventLog.style.marginTop = '10px';
eventLog.style.border = '1px solid #ccc';
eventLog.style.padding = '10px';
eventLog.style.minHeight = '100px';
container.appendChild(eventLog);

// Estado de inicialización
let isInitialized = false;
const initSubject = new Subject<boolean>();

// Botón de inicialización
fromEvent(initButton, 'click').subscribe(() => {
  if (!isInitialized) {
    isInitialized = true;
    initSubject.next(true);
    statusDiv.style.backgroundColor = '#e8f5e9';
    statusDiv.style.borderColor = '#4CAF50';
    statusDiv.innerHTML = '<strong>✅ Inicialización del sistema completada</strong><br>Evento procesado';
    initButton.disabled = true;
  }
});

// Procesamiento del suceso (omitido hasta que finalice la inicialización)
let eventCount = 0;
fromEvent(eventButton, 'click').pipe(
  map(() => {
    eventCount++;
    return {
      id: eventCount,
      timestamp: new Date(),
      initialized: isInitialized
    };
  }),
  tap(event => {
    if (!event.initialized) {
      const skipLog = document.createElement('div');
      skipLog.style.padding = '5px';
      skipLog.style.marginBottom = '3px';
      skipLog.style.color = '#999';
      skipLog.textContent = `⏭️ Evento #${event.id} Omitir (no inicializado)`;
      eventLog.insertBefore(skipLog, eventLog.firstChild);
    }
  }),
  skipWhile(event => !event.initialized)
).subscribe(event => {
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.marginBottom = '3px';
  log.style.backgroundColor = '#e8f5e9';
  log.style.border = '1px solid #4CAF50';
  log.innerHTML = `
    <strong>✅ Evento #${event.id} Procesamiento</strong>
    [${event.timestamp.toLocaleTimeString()}]
  `;
  eventLog.insertBefore(log, eventLog.firstChild);

  // Máx.10Visualizar hasta
  while (eventLog.children.length > 10) {
    eventLog.removeChild(eventLog.lastChild!);
  }
});
```

- Se omiten todos los eventos hasta que se inicializa el sistema.
- Una vez finalizada la inicialización, se procesan todos los eventos.

## 🆚 Comparación con operadores similares

### skipWhile vs takeWhile vs skip vs filter

```ts
import { range } from 'rxjs';
import { skipWhile, takeWhile, skip, filter } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

// skipWhile: Omitir mientras se cumplan las condiciones, toda la salida a partir de entonces
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9

// takeWhile: Adquirir sólo mientras se cumplan las condiciones
numbers$.pipe(
  takeWhile(n => n < 5)
).subscribe(console.log);
// Salida: 0, 1, 2, 3, 4

// skip: Omite el primeroNOmite la primera
numbers$.pipe(
  skip(5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9

// filter: Pasa sólo los valores que satisfacen la condición (evalúa el conjunto)
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9
```

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9
```

```3___

**diferencias visuales**:.

```
Entrada: 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0

skipWhile(n => n < 5):
[0,1,2,3,4 Saltar] | 5, 4, 3, 2, 1, 0
                      ^Todas las salidas después de la condiciónfalseTodas las salidas después de la

filter(n => n >= 5):
[0,1,2,3,4 Exclusión] 5 [4,3,2,1,0 Exclusión]
                 ^Sólo los valores que cumplen la condición (evaluados cada vez)

takeWhile(n => n < 5):
0, 1, 2, 3, 4 | [5Ignorar todo lo posterior y completar]
```

## ⚠️ Notas.

### 1. una vez que una condición es falsa, no se vuelve a evaluar.

Esta es la mayor diferencia con `filter`.

```

```ts
import { from } from 'rxjs';
import { skipWhile, filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 4, 3, 2, 1]);

// skipWhile: Una vez que la condiciónfalsese convierte en "1", da salida a todo lo posterior.
numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(val => console.log('skipWhile:', val));
// Salida: skipWhile: 5, 4, 3, 2, 1(Da salida a todos los valores después de5Da salida a todos los valores después)

// filter: Evalúa la condición cada vez
numbers$.pipe(
  filter(n => n >= 5)
).subscribe(val => console.log('filter:', val));
// Salida: filter: 5(Da salida a todos los valores después de5(sólo salida)
```

### 2. si la condición es falsa desde el principio

Si la condición es `false` desde el principio, se emiten todos los valores.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(5, 5).pipe( // 5de (a)9a
  skipWhile(n => n < 3) // Si la condición se evalúa desde el principiofalse
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9(Todas las salidas)
```

### 3. si todos los valores cumplen la condición

Si todos los valores cumplen la condición, no sale nada.

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

range(0, 5).pipe( // 0de (a)4a
  skipWhile(n => n < 10) // Todos los valores satisfacen la condición
).subscribe({
  next: console.log,
  complete: () => console.log('Completado (nada salida)')
});
// Salida: Completado (nada salida)
```

### 4. Tipos en TypeScript

skipWhile` no cambia el tipo.

```ts
import { Observable, from } from 'rxjs';
import { skipWhile } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

const users$: Observable<User> = from([
  { id: 1, name: 'Alice', isActive: false },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true },
  { id: 4, name: 'Dave', isActive: true }
]);

// El tipo sigue Observable<User> sigue siendo el mismo que antes
const activeUsers$: Observable<User> = users$.pipe(
  skipWhile(user => !user.isActive)
);

activeUsers$.subscribe(user => {
  console.log(`${user.name} (ID: ${user.id})`);
});
// Salida: Charlie (ID: 3), Dave (ID: 4)
```

## 💡 Patrones de combinación prácticos

### Patrón 1: Omitir las líneas de cabecera.

Omitir líneas de encabezado, por ejemplo, CSV

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9
```

```0___.

### Patrón 2: Filtrado basado en marcas de tiempo

Sólo procesa datos después de una hora específica

```

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9
```

```1___.

### Patrón 3: Omisión basada en el estado

Omitir hasta que el sistema esté listo

```

```ts
import { range } from 'rxjs';
import { skipWhile } from 'rxjs';

const numbers$ = range(0, 10); // 0de (a)9a

numbers$.pipe(
  skipWhile(n => n < 5)
).subscribe(console.log);
// Salida: 5, 6, 7, 8, 9
```

```2___.

## 📚 Operadores relacionados.

- takeWhile](. /takeWhile)** - toma valor sólo mientras se cumple la condición.
- skip](. /skip)** - salta los N primeros valores.
- skipLast](. /skipLast)** - salta los N últimos valores.
- skipUntil](. /skipUntil)** - omitir hasta que se dispare otro Observable
- filter](. /filter)** - sólo pasa los valores que cumplen la condición

## Resumen.

El operador `skipWhile` omite los valores que satisfacen una condición y emite todos los valores posteriores a partir del punto en que la condición se convierte en falsa.

- ✅ Ideal para saltarse datos iniciales no deseados.
- ✅ Las condiciones no se vuelven a evaluar una vez que son falsas.
- ✅ Útil para saltarse los periodos de calentamiento y preparación.
- ✅ Se puede utilizar para omitir filas de encabezado
- ⚠️ A diferencia de `filter`, la condición se evalúa una sola vez
- ⚠️ Si todos los valores cumplen la condición, no se emite nada
- ⚠️ Dura hasta que finaliza el flujo original
