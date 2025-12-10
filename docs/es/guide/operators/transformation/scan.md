---
description: "El operador scan es un operador de RxJS que acumula valores secuencialmente y emite resultados intermedios. A diferencia de reduce(), emite resultados cada vez que llega un valor, por lo que se utiliza para agregación en tiempo real, gestión de estado, contadores acumulativos y cálculo de streaming. Explica implementación segura en TypeScript."
---

# scan - Generar valores acumulativamente

El operador `scan` aplica una función acumulativa a cada valor del stream y emite **resultados intermedios secuenciales**.
Es similar a `Array.prototype.reduce` de arrays, pero difiere en que emite resultados intermedios secuencialmente antes de que lleguen todos los valores.

## 🔰 Sintaxis básica y uso

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(scan((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Salida: 1, 3, 6, 10, 15

```

- `acc` es el valor acumulativo, `curr` es el valor actual.
- Comienza desde el valor inicial (en este caso `0`) y acumula secuencialmente.

[🌐 Documentación oficial de RxJS - `scan`](https://rxjs.dev/api/operators/scan)

## 💡 Patrones de uso típicos

- Incremento de conteo y agregación de puntaje
- Gestión de estado de validación de formularios en tiempo real
- Procesamiento acumulativo de eventos almacenados en buffer
- Construcción de datos para gráficos de agregación en tiempo real

## 🧠 Ejemplo de código práctico (con UI)

Cada vez que se hace clic en un botón, se muestra el total de clics acumulado.

```ts
import { fromEvent } from 'rxjs';
import { scan, tap } from 'rxjs';

// Crear botón
const button = document.createElement('button');
button.textContent = 'Clic';
document.body.appendChild(button);

// Crear área de salida
const counter = document.createElement('div');
counter.style.marginTop = '10px';
document.body.appendChild(counter);

// Acumular eventos de clic
fromEvent(button, 'click')
  .pipe(
    tap((v) => console.log(v)),
    scan((count) => count + 1, 0)
  )
  .subscribe((count) => {
    counter.textContent = `Cantidad de clics: ${count}`;
  });
```

- El contador aumenta en 1 cada vez que se hace clic en el botón.
- Usando `scan`, se puede escribir **lógica de conteo simple sin gestión de estado**.

## 🎯 Agregación en tiempo real

Ejemplo de cálculo de suma y promedio en tiempo real.

```ts
import { interval } from 'rxjs';
import { scan, map, take } from 'rxjs';

interface Stats {
  sum: number;
  count: number;
  avg: number;
}

interval(500).pipe(
  take(10),
  scan((acc, curr) => {
    const sum = acc.sum + curr;
    const count = acc.count + 1;
    return {
      sum,
      count,
      avg: sum / count
    };
  }, { sum: 0, count: 0, avg: 0 } as Stats)
).subscribe(stats => {
  console.log(`Suma: ${stats.sum}, Promedio: ${stats.avg.toFixed(2)}`);
});

// Salida:
// Suma: 0, Promedio: 0.00
// Suma: 1, Promedio: 0.50
// Suma: 3, Promedio: 1.00
// Suma: 6, Promedio: 1.50
// ...
```

## 🎯 Gestión de historial de acciones

Ejemplo de acumulación de acciones del usuario en un historial.

```ts
import { fromEvent } from 'rxjs';
import { scan, map } from 'rxjs';

interface Action {
  type: string;
  timestamp: number;
}

// Crear botón
const actionButton = document.createElement('button');
actionButton.textContent = 'Ejecutar acción';
document.body.appendChild(actionButton);

// Crear visualización de historial
const historyDiv = document.createElement('div');
historyDiv.style.marginTop = '10px';
document.body.appendChild(historyDiv);

// Acumular historial de acciones
fromEvent(actionButton, 'click').pipe(
  map(() => ({
    type: 'CLICK',
    timestamp: Date.now()
  } as Action)),
  scan((history, action) => {
    // Mantener solo las últimas 5 acciones
    const newHistory = [...history, action];
    return newHistory.slice(-5);
  }, [] as Action[])
).subscribe(history => {
  historyDiv.innerHTML = `
    <h3>Historial de acciones (últimas 5)</h3>
    <ul>
      ${history.map(a => `<li>${a.type} - ${new Date(a.timestamp).toLocaleTimeString()}</li>`).join('')}
    </ul>
  `;
});
```

## 🎯 Construcción de objeto de estado

Ejemplo de gestión de estado complejo como un objeto.

```ts
import { fromEvent } from 'rxjs';
import { scan, map } from 'rxjs';

interface AppState {
  clickCount: number;
  lastClickTime: number | null;
  totalDuration: number;
}

const button = document.createElement('button');
button.textContent = 'Clic';
document.body.appendChild(button);

const stateDiv = document.createElement('div');
stateDiv.style.marginTop = '10px';
document.body.appendChild(stateDiv);

fromEvent(button, 'click').pipe(
  map(() => Date.now()),
  scan((state, timestamp) => {
    const duration = state.lastClickTime
      ? timestamp - state.lastClickTime
      : 0;

    return {
      clickCount: state.clickCount + 1,
      lastClickTime: timestamp,
      totalDuration: state.totalDuration + duration
    };
  }, {
    clickCount: 0,
    lastClickTime: null,
    totalDuration: 0
  } as AppState)
).subscribe(state => {
  const avgInterval = state.clickCount > 1
    ? (state.totalDuration / (state.clickCount - 1)).toFixed(0)
    : 0;

  stateDiv.innerHTML = `
    <div>Clics totales: ${state.clickCount}</div>
    <div>Intervalo promedio: ${avgInterval}ms</div>
  `;
});
```

## 🔍 Diferencia con reduce

| Característica | `scan` | `reduce` |
|:---|:---|:---|
| **Momento de emisión** | Cada vez que llega un valor | Solo una vez al completar el stream |
| **Resultados intermedios** | Emite | No emite |
| **Stream infinito** | Funciona | No funciona (nunca emite) |
| **Caso de uso** | Agregación en tiempo real, gestión de estado | Agregación solo con resultado final |

```ts
import { of } from 'rxjs';
import { scan, reduce } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== scan (emite resultados intermedios) ===');
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Salida: 1, 3, 6, 10, 15

console.log('=== reduce (solo resultado final) ===');
source$.pipe(
  reduce((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Salida: 15
```

## ⚠️ Puntos de atención

### Gestión de memoria

Ten cuidado de que el valor acumulativo no crezca demasiado.

```ts
// Ejemplo problemático: el array crece infinitamente
source$.pipe(
  scan((acc, curr) => [...acc, curr], [])
)

// Mejora: limitar cantidad
source$.pipe(
  scan((acc, curr) => {
    const newAcc = [...acc, curr];
    return newAcc.slice(-100); // mantener solo los últimos 100 elementos
  }, [])
)
```

### Valor inicial

Si no se especifica un valor inicial, la primera emisión será el primer valor del stream.

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

// Sin valor inicial
of(1, 2, 3).pipe(
  scan((acc, curr) => acc + curr)
).subscribe(console.log);
// Salida: 1, 3, 6 (comienza desde el primer valor)

// Con valor inicial
of(1, 2, 3).pipe(
  scan((acc, curr) => acc + curr, 0)
).subscribe(console.log);
// Salida: 1, 3, 6 (mismo resultado, pero comienza desde 0)
```

## 📚 Operadores relacionados

- [`reduce`](./reduce) - Emitir solo el resultado final al completar
- [`mergeScan`](./mergeScan) - Acumulación asíncrona
- [`expand`](./expand) - Expansión recursiva
- [`toArray`](../utility/toArray) - Recopilar todos los valores en un array

## Resumen

El operador `scan` es una herramienta poderosa para **acumular valores secuencialmente y emitir resultados intermedios**.
Es indispensable para agregación en tiempo real, gestión de estado, contadores y construcción de datos de streaming.

A diferencia de `reduce`, `scan` emite resultados cada vez que llega un valor, por lo que también funciona con streams infinitos.
