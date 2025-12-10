---
description: "El operador pairwise emite dos valores consecutivos como un par de array [valor anterior, valor actual]. Se utiliza para detección de cambios de valores, cálculo de diferencias, análisis de tendencias, interpolación de animación y otros procesamientos que comparan valores anteriores y posteriores. Explica implementación segura en TypeScript y ejemplos prácticos."
---

# pairwise - Procesar dos valores consecutivos en pares

El operador `pairwise` **agrupa dos valores consecutivos emitidos del stream como un array `[valor anterior, valor actual]` y lo emite**.
Es útil para comparar el valor anterior con el actual o calcular la cantidad de cambio.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { pairwise, take } from 'rxjs';

interval(1000).pipe(
  take(6),
  pairwise()
).subscribe(console.log);

// Salida:
// [0, 1]
// [1, 2]
// [2, 3]
// [3, 4]
// [4, 5]
```

- El primer valor (0) no se emite solo, sino que se emite como `[0, 1]` cuando llega el segundo valor (1).
- Siempre se emite un par del **valor inmediatamente anterior y el valor actual**.

[🌐 Documentación oficial de RxJS - `pairwise`](https://rxjs.dev/api/operators/pairwise)

## 💡 Patrones de uso típicos

- Cálculo de cantidad de movimiento de mouse o touch
- Cálculo de cantidad de cambio (diferencia) de precio o valor numérico
- Detección de cambio de estado (comparación de estado anterior y actual)
- Determinación de dirección de desplazamiento

## 🧠 Ejemplo de código práctico (con UI)

Ejemplo que muestra la dirección y cantidad de movimiento del mouse.

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.fontFamily = 'monospace';
document.body.appendChild(output);

// Evento de movimiento del mouse
fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY })),
  pairwise()
).subscribe(([prev, curr]) => {
  const deltaX = curr.x - prev.x;
  const deltaY = curr.y - prev.y;
  const direction = deltaX > 0 ? 'derecha' : deltaX < 0 ? 'izquierda' : 'detenido';

  output.innerHTML = `
    Anterior: (${prev.x}, ${prev.y})<br>
    Actual: (${curr.x}, ${curr.y})<br>
    Cantidad de movimiento: Δx=${deltaX}, Δy=${deltaY}<br>
    Dirección: ${direction}
  `;
});
```

- Al mover el mouse, se muestran las coordenadas anterior y actual, y la cantidad de movimiento.
- Con `pairwise`, se pueden obtener automáticamente las coordenadas anterior y actual como un par.

## 🎯 Ejemplo de cálculo de cantidad de cambio de valores numéricos

Ejemplo práctico de cálculo de cantidad de cambio (diferencia) de un stream numérico.

```ts
import { interval } from 'rxjs';
import { map, pairwise, take } from 'rxjs';

// 0, 1, 4, 9, 16, 25 (números cuadrados)
interval(500).pipe(
  take(6),
  map(n => n * n),
  pairwise(),
  map(([prev, curr]) => ({
    prev,
    curr,
    diff: curr - prev
  }))
).subscribe(result => {
  console.log(`${result.prev} → ${result.curr} (diferencia: +${result.diff})`);
});

// Salida:
// 0 → 1 (diferencia: +1)
// 1 → 4 (diferencia: +3)
// 4 → 9 (diferencia: +5)
// 9 → 16 (diferencia: +7)
// 16 → 25 (diferencia: +9)
```

## 🎯 Determinación de dirección de desplazamiento

Ejemplo de determinación de dirección de desplazamiento (arriba/abajo).

```ts
import { fromEvent } from 'rxjs';
import { map, pairwise, throttleTime } from 'rxjs';

// Crear área de salida con visualización fija
const output = document.createElement('div');
output.style.position = 'fixed';
output.style.top = '10px';
output.style.right = '10px';
output.style.padding = '15px';
output.style.backgroundColor = 'rgba(0, 0, 0, 0.8)';
output.style.color = 'white';
output.style.fontFamily = 'monospace';
output.style.fontSize = '14px';
output.style.borderRadius = '5px';
output.style.zIndex = '9999';
document.body.appendChild(output);

// Contenido ficticio para hacer la página desplazable
const content = document.createElement('div');
content.style.height = '200vh'; // Hacer la altura de la página el doble
content.innerHTML = '<h1>Por favor, desplácese hacia abajo</h1>';
document.body.appendChild(content);

// Obtener posición de desplazamiento
fromEvent(window, 'scroll').pipe(
  throttleTime(100), // Reducir cada 100ms
  map(() => window.scrollY),
  pairwise()
).subscribe(([prevY, currY]) => {
  const diff = currY - prevY;
  const direction = diff > 0 ? '↓ abajo' : '↑ arriba';
  const arrow = diff > 0 ? '⬇️' : '⬆️';

  output.innerHTML = `
    ${arrow} Dirección de desplazamiento: ${direction}<br>
    Posición anterior: ${prevY.toFixed(0)}px<br>
    Posición actual: ${currY.toFixed(0)}px<br>
    Cantidad de movimiento: ${Math.abs(diff).toFixed(0)}px
  `;
});
```

- Al desplazarse por la página, se muestra información de dirección y posición en un área fija en la esquina superior derecha.
- Con `pairwise`, se pueden obtener automáticamente las posiciones de desplazamiento anterior y actual como un par.

## 🎯 Uso de pairwise con seguridad de tipos

Ejemplo aprovechando la inferencia de tipos de TypeScript.

```ts
import { from } from 'rxjs';
import { pairwise } from 'rxjs';

interface Stock {
  symbol: string;
  price: number;
  timestamp: number;
}

const stockPrices: Stock[] = [
  { symbol: 'AAPL', price: 150, timestamp: 1000 },
  { symbol: 'AAPL', price: 152, timestamp: 2000 },
  { symbol: 'AAPL', price: 148, timestamp: 3000 },
  { symbol: 'AAPL', price: 155, timestamp: 4000 },
];

from(stockPrices).pipe(
  pairwise()
).subscribe(([prev, curr]) => {
  const change = curr.price - prev.price;
  const changePercent = ((change / prev.price) * 100).toFixed(2);
  const trend = change > 0 ? '📈' : change < 0 ? '📉' : '➡️';

  console.log(
    `${curr.symbol}: $${prev.price} → $${curr.price} ` +
    `(${changePercent}%) ${trend}`
  );
});

// Salida:
// AAPL: $150 → $152 (1.33%) 📈
// AAPL: $152 → $148 (-2.63%) 📉
// AAPL: $148 → $155 (4.73%) 📈
```

## 🔍 Comparación con bufferCount(2, 1)

`pairwise()` tiene el mismo comportamiento que `bufferCount(2, 1)`.

```ts
import { of } from 'rxjs';
import { pairwise, bufferCount } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

console.log('=== pairwise ===');
source$.pipe(pairwise()).subscribe(console.log);
// Salida: [1,2], [2,3], [3,4], [4,5]

console.log('=== bufferCount(2, 1) ===');
source$.pipe(bufferCount(2, 1)).subscribe(console.log);
// Salida: [1,2], [2,3], [3,4], [4,5]
```

**Diferenciación de uso**:
- `pairwise()`: Es explícito para manejar pares de dos valores consecutivos, y la intención del código es clara
- `bufferCount(2, 1)`: Más flexible (también puede manejar tamaños de ventana de 3 o más)

## ⚠️ Puntos de atención

### El primer valor no se emite

Como `pairwise` no emite nada hasta que se reúnen dos valores, el primer valor no se puede obtener solo.

```ts
import { of } from 'rxjs';
import { pairwise } from 'rxjs';

of(1).pipe(pairwise()).subscribe({
  next: console.log,
  complete: () => console.log('completado')
});

// Salida:
// completado
// (no se emite ningún valor)
```

**Solución**: Si también se quiere procesar el primer valor, agregar un valor inicial con `startWith`.

```ts
import { of } from 'rxjs';
import { startWith, pairwise } from 'rxjs';

of(10, 20, 30).pipe(
  startWith(0),
  pairwise()
).subscribe(console.log);

// Salida:
// [0, 10]
// [10, 20]
// [20, 30]
```

### Uso de memoria

Como `pairwise` solo mantiene el valor inmediatamente anterior, el uso de memoria es bueno.

## 📚 Operadores relacionados

- [`scan`](./scan) - Procesamiento acumulativo más complejo
- [`bufferCount`](./bufferCount) - Agrupar valores por cantidad especificada
- [`distinctUntilChanged`](../filtering/distinctUntilChanged) - Eliminar valores duplicados consecutivos
- [`startWith`](../utility/startWith) - Añadir valor inicial

## Resumen

El operador `pairwise` emite dos valores consecutivos como un par `[valor anterior, valor actual]`. Es muy útil en **situaciones donde se necesita comparar el valor anterior y el actual**, como seguimiento de movimiento del mouse, cálculo de fluctuación de precios y detección de transición de estado. Es importante tener en cuenta que el primer valor no se emite hasta que llega el segundo valor, pero se puede manejar con `startWith` agregando un valor inicial.
