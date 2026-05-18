---
description: window es un operador de RxJS que divide el Observable fuente en Observables anidados según el momento en que otro Observable emite valores, ideal para procesamiento avanzado de streams basado en eventos.
---

# window - Ventana por Evento

El operador `window` agrupa los valores del Observable fuente **hasta que otro Observable emite un valor**, y emite ese grupo como **un nuevo Observable**.
Mientras que `buffer` devuelve un array, `window` **devuelve un Observable\\<T>**, lo que permite aplicar operadores adicionales a cada ventana.

## 🔰 Sintaxis básica y uso

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// Emitir valores cada 100ms
const source$ = interval(100);

// Usar evento de clic como trigger
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Aplanar cada ventana
).subscribe(value => {
  console.log('Valor en ventana:', value);
});

// Se inicia una nueva ventana cada vez que se hace clic
```

- Cada vez que `clicks$` emite un valor, se crea una nueva ventana (Observable).
- Cada ventana puede procesarse como un Observable independiente.

[🌐 Documentación oficial de RxJS - `window`](https://rxjs.dev/api/operators/window)

## 💡 Patrones de uso típicos

- División de streams basada en eventos
- Aplicar diferentes procesos a cada ventana
- Agrupar datos con delimitadores dinámicos
- Procesamiento de agregación para cada ventana

## 🔍 Diferencia con buffer

| Operador | Salida | Caso de uso |
|:---|:---|:---|
| `buffer` | **Array (T[])** | Procesar valores agrupados juntos |
| `window` | **Observable\\<T>** | Procesamiento de stream diferente por grupo |

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - emite como array
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Salida: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// window - emite como Observable
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('Ventana (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valor en ventana:', value);
  });
});
```

## 🧠 Ejemplo de código práctico 1: Contar por ventana

Ejemplo de contar el número de eventos hasta el clic del botón.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// Crear botón
const button = document.createElement('button');
button.textContent = 'Delimitar ventana';
document.body.appendChild(button);

// Área de salida
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Emitir valores cada 100ms
const source$ = interval(100);

// Usar clic del botón como trigger
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`Inicio de ventana ${currentWindow}`);

    // Contar valores en cada ventana
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `Ventana actual: ${windowCount}, Conteo: ${count}`;
});
```

- Se crea una nueva ventana cada vez que se hace clic en el botón.
- El número de valores en cada ventana se cuenta en tiempo real.

## 🎯 Ejemplo de código práctico 2: Aplicar diferentes procesos a cada ventana

Ejemplo avanzado que aplica diferentes procesos a cada ventana.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, take, mergeAll, map } from 'rxjs';

const source$ = interval(200);
const clicks$ = fromEvent(document, 'click');

let windowNumber = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Ventana par: obtener solo los primeros 3
      console.log(`Ventana ${current}: obtener los primeros 3`);
      return window$.pipe(take(3));
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

- Se pueden aplicar diferentes procesos mediante bifurcación condicional por ventana.
- Como cada ventana es un Observable independiente, se pueden combinar operadores libremente.

## 🎯 Ejemplo de uso: Control con múltiples triggers

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { window, mergeAll, scan, map } from 'rxjs';

const source$ = interval(100);

// Múltiples triggers: clic o 3 segundos transcurridos
const clicks$ = fromEvent(document, 'click');
const threeSeconds$ = timer(3000, 3000);
const trigger$ = merge(clicks$, threeSeconds$);

source$.pipe(
  window(trigger$),
  map((window$, index) => {
    console.log(`Inicio de ventana ${index + 1}`);

    // Calcular valor total de cada ventana
    return window$.pipe(
      scan((sum, value) => sum + value, 0)
    );
  }),
  mergeAll()
).subscribe(sum => {
  console.log('Total actual:', sum);
});
```

## ⚠️ Puntos de atención

### 1. Gestión de suscripciones de ventanas

Como cada ventana es un Observable independiente, es necesario suscribirse explícitamente.

```ts
source$.pipe(
  window(trigger$)
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
  window(trigger$),
  mergeAll() // Fusionar todas las ventanas
).subscribe(value => {
  console.log('Valor:', value);
});
```

### 2. Atención a fugas de memoria

**Problema**: Si el Observable trigger no emite valores, la primera ventana permanecerá abierta para siempre y los valores se acumularán infinitamente.

#### ❌ Mal ejemplo: El trigger no ocurre

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100); // Continúa emitiendo valores cada 100ms

// El botón no existe o el usuario no hace clic
const button = document.querySelector('#start-button'); // Posibilidad de null
const clicks$ = fromEvent(button, 'click'); // Error o nunca se dispara

source$.pipe(
  window(clicks$), // Si clicks$ no se dispara, la primera ventana no se cierra
  mergeAll()
).subscribe();

// Problemas:
// - Si clicks$ no emite valores, la primera ventana permanece abierta
// - Los valores de source$ (0, 1, 2, 3...) se acumulan en memoria
// - Causa de fuga de memoria
```

#### ✅ Buen ejemplo 1: Configurar timeout

Configurar un timeout para que la primera ventana no permanezca abierta demasiado tiempo.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = button ? fromEvent(button, 'click') : interval(0); // fallback a observable ficticio si button es null

// Cerrar ventana con lo que ocurra primero: clic o 5 segundos transcurridos
const autoClose$ = timer(5000); // Emitir valor automáticamente después de 5 segundos
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // La ventana se cierra definitivamente dentro de 5 segundos
  mergeAll()
).subscribe();
```

#### ✅ Buen ejemplo 2: Cerrar ventanas periódicamente

Cerrar ventanas periódicamente e iniciar nuevas ventanas incluso sin clics.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = fromEvent(button, 'click');

// Cerrar ventana con clics o cada 3 segundos
const autoClose$ = timer(3000, 3000); // Después de los primeros 3 segundos, cada 3 segundos
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // La ventana se cierra cada 3 segundos incluso sin clics
  mergeAll()
).subscribe();

// Resultado:
// - Incluso si el usuario no hace clic, la ventana se cierra automáticamente cada 3 segundos
// - Previene la acumulación infinita de valores en memoria
```

### 3. Superposición de ventanas

Por defecto, las ventanas no se superponen (la siguiente comienza después de que se cierra la anterior).
Si se necesita superposición, usar `windowToggle` o `windowWhen`.

## 🆚 Comparación de operadores de la familia window

| Operador | Timing de delimitación | Caso de uso |
|:---|:---|:---|
| `window` | Emisión de otro Observable | División basada en eventos |
| `windowTime` | Tiempo fijo | División basada en tiempo |
| `windowCount` | Cantidad fija | División basada en cantidad |
| `windowToggle` | Observable de inicio/fin | Control dinámico de inicio/fin |
| `windowWhen` | Condición de cierre dinámica | Condición de fin diferente por ventana |

## 📚 Operadores relacionados

- [`buffer`](./buffer) - Agrupar valores como array (versión de array de window)
- [`windowTime`](./windowTime) - División de ventana basada en tiempo
- [`windowCount`](./windowCount) - División de ventana basada en cantidad
- [`windowToggle`](./windowToggle) - Control de ventana con Observable de inicio/fin
- [`windowWhen`](./windowWhen) - División de ventana con condición de cierre dinámica
- [`groupBy`](./groupBy) - Agrupar Observable por clave

## Resumen

El operador `window` es una herramienta poderosa que divide streams usando Observables externos como triggers y puede procesar cada grupo como un Observable independiente.

- ✅ Puede aplicar diferentes procesos a cada ventana
- ✅ Control flexible basado en eventos
- ✅ Compatible con operaciones de stream avanzadas
- ⚠️ Requiere gestión de suscripciones
- ⚠️ Atención a fugas de memoria
