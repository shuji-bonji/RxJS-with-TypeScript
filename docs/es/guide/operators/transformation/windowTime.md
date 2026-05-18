---
description: "El operador windowTime divide un Observable en intervalos de tiempo fijos, permitiendo procesar los valores emitidos en cada marco temporal como Observables individuales. Se explica la división de streams basada en tiempo, procesamiento por lotes, diferencias con bufferTime e implementación con seguridad de tipos en TypeScript."
---

# windowTime - Ventana por Tiempo

El operador `windowTime` agrupa los valores del Observable fuente **en intervalos de tiempo fijos** y emite ese grupo como **un nuevo Observable**.
Mientras que `bufferTime` devuelve un array, `windowTime` **devuelve un Observable\<T>**, lo que permite aplicar operadores adicionales a cada ventana.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// Emitir valores cada 100ms
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // Crear ventana cada 1 segundo
  take(3),          // Solo las primeras 3 ventanas
  mergeAll()        // Aplanar cada ventana
).subscribe(value => {
  console.log('Valor:', value);
});

// Salida:
// 1er segundo: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2do segundo: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3er segundo: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- Se crea una nueva ventana (Observable) cada tiempo especificado (1000ms).
- Cada ventana puede procesarse como un Observable independiente.

[🌐 Documentación oficial de RxJS - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 Patrones de uso típicos

- **Procesamiento por lotes basado en tiempo**: Procesar datos agrupados en intervalos fijos
- **Agregación de datos en tiempo real**: Contar eventos por segundo
- **Monitoreo de rendimiento**: Recopilación de métricas en intervalos fijos
- **Análisis de datos de series temporales**: Procesamiento estadístico por marco temporal

## 🔍 Diferencia con bufferTime

| Operador | Salida | Caso de uso |
|:---|:---|:---|
| `bufferTime` | **Array (T[])** | Procesar valores agrupados juntos |
| `windowTime` | **Observable\<T>** | Procesamiento de stream diferente por marco temporal |

```ts
import { interval } from 'rxjs';
import { bufferTime, windowTime, take } from 'rxjs';

const source$ = interval(100);

// bufferTime - emite como array
source$.pipe(
  bufferTime(1000),
  take(2)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Salida: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// windowTime - emite como Observable
source$.pipe(
  windowTime(1000),
  take(2)
).subscribe(window$ => {
  console.log('Ventana (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valor:', value);
  });
});
```

## 🧠 Ejemplo de código práctico 1: Contar clics por segundo

Ejemplo de agregar el número de clics del botón cada segundo.

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// Crear botón
const button = document.createElement('button');
button.textContent = 'Clic';
document.body.appendChild(button);

// Área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento de clic
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // Crear ventana cada 1 segundo
  map(window$ => {
    ++windowNumber;

    // Contar clics en cada ventana
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] Ventana ${windowNumber}: ${count} clics`;
});
```

- Se crea una nueva ventana cada segundo.
- El número de clics en cada ventana se cuenta en tiempo real.

## 🎯 Ejemplo de código práctico 2: Procesamiento estadístico por marco temporal

Ejemplo de calcular la suma y el promedio de valores en cada marco temporal.

```ts
import { interval } from 'rxjs';
import { windowTime, map, mergeMap, toArray, take } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Procesamiento estadístico por marco temporal (cada segundo)</h3>';
document.body.appendChild(output);

const table = document.createElement('table');
table.style.borderCollapse = 'collapse';
table.style.marginTop = '10px';
table.innerHTML = `
  <thead>
    <tr style="background: #f0f0f0;">
      <th style="border: 1px solid #ccc; padding: 8px;">Ventana</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Cantidad</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Suma</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Promedio</th>
    </tr>
  </thead>
  <tbody id="stats-body"></tbody>
`;
output.appendChild(table);

const source$ = interval(100).pipe(
  map(() => Math.floor(Math.random() * 100)) // Valor aleatorio
);

let windowNumber = 0;

source$.pipe(
  windowTime(1000), // Cada segundo
  take(5),          // Solo 5 ventanas
  mergeMap(window$ => {
    const current = ++windowNumber;

    // Convertir valores de cada ventana a array y procesar estadísticas
    return window$.pipe(
      toArray(),
      map(values => ({
        window: current,
        count: values.length,
        sum: values.reduce((a, b) => a + b, 0),
        avg: values.length > 0
          ? (values.reduce((a, b) => a + b, 0) / values.length).toFixed(2)
          : 0
      }))
    );
  })
).subscribe(stats => {
  const tbody = document.getElementById('stats-body')!;
  const row = document.createElement('tr');
  row.innerHTML = `
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.window}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.count}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.sum}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.avg}</td>
  `;
  tbody.appendChild(row);
});
```

- Se pueden calcular estadísticas individuales para cada ventana.
- Es posible aplicar diferentes procesos por ventana.
- Los datos estadísticos se muestran visualmente en formato de tabla.

## 📊 Ventanas superpuestas (windowCreationInterval)

Especificando `windowCreationInterval` como segundo argumento, se pueden superponer ventanas.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Ventanas superpuestas</h3>';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.marginTop = '10px';
document.body.appendChild(output);

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // Longitud de ventana: 2 segundos
    1000   // Intervalo de creación de ventana: 1 segundo
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  const div = document.createElement('div');
  div.style.marginTop = '10px';
  div.style.padding = '5px';
  div.style.backgroundColor = '#f5f5f5';
  div.style.borderLeft = '3px solid #4CAF50';

  const title = document.createElement('strong');
  title.textContent = `Ventana ${result.window}:`;
  div.appendChild(title);

  div.appendChild(document.createElement('br'));

  const values = document.createElement('span');
  values.textContent = `Valores: [${result.values.join(', ')}]`;
  div.appendChild(values);

  div.appendChild(document.createElement('br'));

  const info = document.createElement('span');
  info.style.color = '#666';
  info.textContent = `(${result.values.length} valores, ${(result.window - 1)}s〜${(result.window + 1)}s)`;
  div.appendChild(info);

  output.appendChild(div);

  // Contramedida para Chrome: forzar renderizado
  void output.offsetHeight;
});
```

**Explicación del comportamiento:**
- **Ventana 1**: Valores de 0s〜2s `[0, 1, 2, ..., 19]` (20 valores)
- **Ventana 2**: Valores de 1s〜3s `[10, 11, 12, ..., 29]` (20 valores) ← Valores 10-19 se superponen con Ventana 1
- **Ventana 3**: Valores de 2s〜4s `[20, 21, 22, ..., 39]` (20 valores) ← Valores 20-29 se superponen con Ventana 2

- Crear nuevas ventanas en intervalos (1s) más cortos que la longitud de la ventana (2s) causa superposición.
- Conveniente para implementación de ventanas deslizantes.

## 🎯 Ejemplo de uso: Monitoreo de eventos en tiempo real

```ts
import { fromEvent } from 'rxjs';
import { windowTime, mergeMap, toArray, map } from 'rxjs';

// Área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Monitoreo de movimiento del ratón (cada 5 segundos)</h3>';
document.body.appendChild(output);

const list = document.createElement('ul');
output.appendChild(list);

// Evento de movimiento del ratón
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  windowTime(5000), // Cada 5 segundos
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(events => ({
        count: events.length,
        timestamp: new Date().toLocaleTimeString()
      }))
    )
  )
).subscribe(result => {
  const item = document.createElement('li');
  item.textContent = `[${result.timestamp}] Movimiento del ratón: ${result.count} veces`;
  list.insertBefore(item, list.firstChild);

  // Mostrar máximo 10 elementos
  while (list.children.length > 10) {
    list.removeChild(list.lastChild!);
  }
});
```

## ⚠️ Puntos de atención

### 1. Gestión de suscripciones de ventanas

Como cada ventana es un Observable independiente, es necesario suscribirse explícitamente.

```ts
source$.pipe(
  windowTime(1000)
).subscribe(window$ => {
  // Si no se suscribe a la ventana misma, no fluyen los valores
  window$.subscribe(value => {
    console.log('Valor:', value);
  });
});
```

O usar `mergeAll()`, `concatAll()`, `switchAll()`, etc. para aplanar.

```ts
source$.pipe(
  windowTime(1000),
  mergeAll() // Fusionar todas las ventanas
).subscribe(value => {
  console.log('Valor:', value);
});
```

### 2. Gestión de memoria

Para ejecuciones de larga duración, es importante cancelar la suscripción adecuadamente.

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // Cancelar suscripción al destruir
).subscribe();

// Al destruir componente, etc.
destroy$.next();
destroy$.complete();
```

### 3. Especificar valor máximo (maxWindowSize)

El tercer argumento permite limitar el número máximo de valores en cada ventana.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray } from 'rxjs';

interval(100).pipe(
  windowTime(
    2000,      // Longitud de ventana: 2 segundos
    undefined, // Intervalo de creación: por defecto (sin superposición)
    5          // Número máximo de valores: hasta 5
  ),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Ventana:', values);
  // Solo se incluyen máximo 5 valores
});
```

## 🆚 Comparación de operadores de la familia window

| Operador | Timing de delimitación | Caso de uso |
|:---|:---|:---|
| `window` | Emisión de otro Observable | División basada en eventos |
| `windowTime` | **Tiempo fijo** | **División basada en tiempo** |
| `windowCount` | Cantidad fija | División basada en cantidad |
| `windowToggle` | Observable de inicio/fin | Control dinámico de inicio/fin |
| `windowWhen` | Condición de cierre dinámica | Condición de fin diferente por ventana |

## 📚 Operadores relacionados

- **[bufferTime](./bufferTime)** - Agrupar valores como array (versión de array de windowTime)
- **[window](./window)** - División de ventana por emisión de Observable
- **[windowCount](./windowCount)** - División de ventana basada en cantidad
- **[windowToggle](./windowToggle)** - Control de ventana con Observable de inicio/fin
- **[windowWhen](./windowWhen)** - División de ventana con condición de cierre dinámica

## Resumen

El operador `windowTime` es una herramienta poderosa que divide streams por tiempo y puede procesar cada marco temporal como un Observable independiente.

- ✅ Crea ventanas automáticamente en intervalos fijos
- ✅ Puede aplicar diferentes procesos a cada ventana
- ✅ Compatible con ventanas deslizantes (superposición)
- ✅ Ideal para agregación y análisis de datos en tiempo real
- ⚠️ Requiere gestión de suscripciones
- ⚠️ Atención a la gestión de memoria
