---
description: windowCount es un operador de transformación de RxJS que divide un Observable en grupos de un número especificado. Es ideal para procesamiento de streams basado en cantidad, agregación por número fijo, y procesamiento de paginación. A diferencia de bufferCount, permite aplicar procesos independientes a cada ventana. La inferencia de tipos de TypeScript permite división de ventanas y operaciones de stream con seguridad de tipos.
---

# windowCount - Dividir Observable por cantidad especificada

El operador `windowCount` divide los valores emitidos en **nuevos Observables** por cada cantidad especificada.
Mientras que `bufferCount` devuelve un array, `windowCount` **devuelve un Observable\<T>**, lo que permite aplicar operadores adicionales a cada ventana.

## 🔰 Sintaxis básica y uso

```ts
import { interval } from 'rxjs';
import { windowCount, mergeAll } from 'rxjs';

// Emitir valores cada 100ms
const source$ = interval(100);

source$.pipe(
  windowCount(5),
  mergeAll() // Aplanar cada ventana
).subscribe(value => {
  console.log('Valor en ventana:', value);
});

// Salida:
// Valor en ventana: 0
// Valor en ventana: 1
// Valor en ventana: 2
// Valor en ventana: 3
// Valor en ventana: 4
// (Nueva ventana comienza)
// Valor en ventana: 5
// ...
```

- Se crea una nueva ventana (Observable) cada 5 valores.
- La característica es la división basada en cantidad.

[🌐 Documentación oficial de RxJS - `windowCount`](https://rxjs.dev/api/operators/windowCount)

## 💡 Patrones de uso típicos

- Procesamiento de agregación por cantidad fija
- Envío por lotes de datos (diferentes procesos por ventana)
- Procesamiento de paginación
- Calcular información estadística por ventana

## 🔍 Diferencia con bufferCount

| Operador | Salida | Caso de uso |
|:---|:---|:---|
| `bufferCount` | **Array (T[])** | Procesar valores agrupados juntos |
| `windowCount` | **Observable\<T>** | Procesamiento de stream diferente por grupo |

```ts
import { interval } from 'rxjs';
import { bufferCount, windowCount, mergeAll } from 'rxjs';

const source$ = interval(100);

// bufferCount - emite como array
source$.pipe(
  bufferCount(5)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Salida: Buffer (array): [0, 1, 2, 3, 4]
});

// windowCount - emite como Observable
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  console.log('Ventana (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valor en ventana:', value);
  });
});
```

## 🧠 Ejemplo de código práctico 1: Suma total por ventana

Ejemplo de calcular la suma de valores cada 5.

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, reduce } from 'rxjs';

// Crear área de salida
const output = document.createElement('div');
output.innerHTML = '<h3>Suma total cada 5</h3>';
document.body.appendChild(output);

const source$ = interval(200);

let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;
    console.log(`Inicio de ventana ${current}`);

    // Calcular suma de cada ventana
    return window$.pipe(
      reduce((sum, value) => sum + value, 0),
      map(sum => ({ windowNum: current, sum }))  // Incluir número de ventana
    );
  }),
  mergeAll()
).subscribe(result => {
  const div = document.createElement('div');
  div.textContent = `Suma de ventana ${result.windowNum}: ${result.sum}`;
  output.appendChild(div);
});

// Salida:
// Suma de ventana 1: 10  (0+1+2+3+4)
// Suma de ventana 2: 35  (5+6+7+8+9)
// Suma de ventana 3: 60  (10+11+12+13+14)
```

## 🎯 Ejemplo de código práctico 2: Especificar índice de inicio

El segundo argumento permite especificar el índice de inicio. Se pueden crear ventanas superpuestas.

```ts
import { range } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Emitir valores de 0 a 9
range(0, 10).pipe(
  windowCount(3, 2), // 3 a la vez, comenzar desplazando 2
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Ventana:', values);
});

// Salida:
// Ventana: [0, 1, 2]
// Ventana: [2, 3, 4]    ← Comienza desplazando 2 (desde 2)
// Ventana: [4, 5, 6]    ← Comienza desplazando 2 (desde 4)
// Ventana: [6, 7, 8]
// Ventana: [8, 9]       ← Últimos 2
```

### Patrones de comportamiento del índice de inicio

```ts
// windowCount(bufferSize, startBufferEvery)

windowCount(3, 3) // Continuo (por defecto): [0,1,2], [3,4,5], [6,7,8]
windowCount(3, 2) // Superposición: [0,1,2], [2,3,4], [4,5,6]
windowCount(3, 4) // Con espacio: [0,1,2], [4,5,6], [8,9,10]
```

## 🎯 Ejemplo de uso: Aplicar diferentes procesos a cada ventana

```ts
import { interval } from 'rxjs';
import { windowCount, map, mergeAll, take } from 'rxjs';

const source$ = interval(100);
let windowNumber = 0;

source$.pipe(
  windowCount(5),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Ventana par: obtener solo los primeros 2
      console.log(`Ventana ${current}: obtener los primeros 2`);
      return window$.pipe(take(2));
    } else {
      // Ventana impar: obtener todos
      console.log(`Ventana ${current}: obtener todos`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Valor: ${value} (Ventana ${windowNumber})`);
});
```

## 🧠 Ejemplo de código práctico 3: Procesamiento tipo paginación

```ts
import { from } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

// Datos de 1-20
const data$ = from(Array.from({ length: 20 }, (_, i) => i + 1));

// Dividir en páginas de 5
data$.pipe(
  windowCount(5),
  mergeMap((window$, index) => {
    const pageNumber = index + 1;
    return window$.pipe(
      toArray(),
      map(items => ({ page: pageNumber, items }))
    );
  })
).subscribe(page => {
  console.log(`Página ${page.page}:`, page.items);
});

// Salida:
// Página 1: [1, 2, 3, 4, 5]
// Página 2: [6, 7, 8, 9, 10]
// Página 3: [11, 12, 13, 14, 15]
// Página 4: [16, 17, 18, 19, 20]
```

## ⚠️ Puntos de atención

### 1. Gestión de suscripciones de ventanas

Como cada ventana es un Observable independiente, es necesario suscribirse explícitamente.

```ts
source$.pipe(
  windowCount(5)
).subscribe(window$ => {
  // Si no se suscribe a la ventana misma, no fluyen los valores
  window$.subscribe(value => {
    console.log('Valor:', value);
  });
});
```

O usar `mergeAll()`, `concatAll()`, `switchAll()`, etc. para aplanar.

### 2. Última ventana

Al completarse el Observable fuente, la última ventana se emite incluso si tiene menos de la cantidad especificada.

```ts
import { of } from 'rxjs';
import { windowCount, mergeMap, toArray } from 'rxjs';

of(1, 2, 3, 4, 5, 6, 7).pipe(
  windowCount(3),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Ventana:', values);
});

// Salida:
// Ventana: [1, 2, 3]
// Ventana: [4, 5, 6]
// Ventana: [7]  ← Solo 1
```

### 3. Uso de memoria por índice de inicio

Cuando `startBufferEvery` es menor que `bufferSize` (superposición), múltiples ventanas estarán activas simultáneamente, aumentando el uso de memoria.

```ts
// Superposición: máximo 2 ventanas activas simultáneamente
windowCount(5, 3)

// Contramedida: limitar con take() si es necesario
source$.pipe(
  take(100), // Máximo 100
  windowCount(5, 3)
)
```

## 🆚 Comparación de operadores de la familia window

| Operador | Timing de delimitación | Caso de uso |
|:---|:---|:---|
| `window` | Emisión de otro Observable | División basada en eventos |
| `windowTime` | Tiempo fijo | División basada en tiempo |
| `windowCount` | **Cantidad fija** | **División basada en cantidad** |
| `windowToggle` | Observable de inicio/fin | Control dinámico de inicio/fin |
| `windowWhen` | Condición de cierre dinámica | Condición de fin diferente por ventana |

## 📚 Operadores relacionados

- [`bufferCount`](./bufferCount) - Agrupar valores como array (versión de array de windowCount)
- [`window`](./window) - División de ventana según timing de otro Observable
- [`windowTime`](./windowTime) - División de ventana basada en tiempo
- [`windowToggle`](./windowToggle) - Control de ventana con Observable de inicio/fin
- [`windowWhen`](./windowWhen) - División de ventana con condición de cierre dinámica

## Resumen

El operador `windowCount` es una herramienta conveniente que divide streams por cantidad y puede procesar cada grupo como un Observable independiente.

- ✅ Ideal para agregación y procesamiento por cantidad fija
- ✅ Puede aplicar diferentes procesos a cada ventana
- ✅ Superposición posible con índice de inicio
- ⚠️ Requiere gestión de suscripciones
- ⚠️ Atención al uso de memoria durante superposición
