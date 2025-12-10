---
description: "El operador buffer agrupa valores acumulados en un array cada vez que otro Observable emite un valor. Es ideal para procesamiento por lotes basado en eventos como envío masivo al hacer clic en un botón o guardar datos al cerrar una ventana. Explicamos su implementación con seguridad de tipos en TypeScript."
---

# buffer - Agrupar valores según el timing de otro Observable

El operador `buffer` acumula valores del Observable fuente **hasta que otro Observable emite un valor**, y en ese momento emite todos los valores acumulados **como un array**.
Es útil cuando quieres controlar el buffering basándote en eventos externos o señales, en lugar de tiempo o cantidad.

## 🔰 Sintaxis básica y uso

```ts
import { interval, fromEvent } from 'rxjs';
import { buffer } from 'rxjs';

// Emite valores cada 100ms
const source$ = interval(100);

// Usa eventos de clic como trigger
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  buffer(clicks$)
).subscribe(bufferedValues => {
  console.log('Valores acumulados hasta el clic:', bufferedValues);
});

// Ejemplo de salida (se emite cada vez que se hace clic):
// Valores acumulados hasta el clic: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
// Valores acumulados hasta el clic: [11, 12, 13, 14, 15, 16, 17]
// ...
```

- Cada vez que `clicks$` emite un valor, los valores acumulados se emiten como un array.
- La característica distintiva es que el delimitador del buffer se controla mediante un Observable externo.

[🌐 Documentación Oficial RxJS - `buffer`](https://rxjs.dev/api/operators/buffer)

## 💡 Patrones de uso típicos

- Procesamiento por lotes activado por acciones del usuario
- Recolección y envío de datos basados en señales externas
- Agrupación de eventos con delimitadores dinámicos
- Envío masivo al establecer conexión WebSocket o API

## 🔍 Diferencias con bufferTime / bufferCount

| Operador | Timing del delimitador | Uso |
|:---|:---|:---|
| `buffer` | **Emisión de otro Observable** | Control basado en eventos |
| `bufferTime` | **Tiempo fijo** | Procesamiento por lotes basado en tiempo |
| `bufferCount` | **Cantidad fija** | Procesamiento por lotes basado en cantidad |

```ts
import { interval, timer } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);
// Trigger cada 1 segundo
const trigger$ = timer(1000, 1000);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Valores cada 1 segundo:', values);
});

// Salida:
// Valores cada 1 segundo: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// Valores cada 1 segundo: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
```

## 🧠 Ejemplo de código práctico (con UI)

Ejemplo que registra los eventos de movimiento del mouse hasta que se hace clic en un botón.

```ts
import { fromEvent } from 'rxjs';
import { map, buffer } from 'rxjs';

// Crear botón y área de salida
const button = document.createElement('button');
button.textContent = 'Registrar movimiento del mouse';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Eventos de movimiento del mouse
const mouseMoves$ = fromEvent<MouseEvent>(document, 'mousemove').pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
);

// Usar clic del botón como trigger
const clicks$ = fromEvent(button, 'click');

mouseMoves$.pipe(
  buffer(clicks$)
).subscribe(positions => {
  const message = `Eventos detectados: ${positions.length} elementos`;
  console.log(message);
  console.log('Datos de coordenadas:', positions.slice(0, 5)); // Mostrar solo los primeros 5
  output.textContent = message;
});
```

- Todos los movimientos del mouse se acumulan en el buffer hasta que se hace clic en el botón.
- El procesamiento por lotes ocurre al hacer clic, permitiendo procesamiento por lotes en cualquier momento.

## 🎯 Ejemplo avanzado con múltiples triggers

Se puede lograr un control más flexible combinando múltiples Observables de trigger.

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { buffer, mapTo } from 'rxjs';

const source$ = interval(100);

// Múltiples triggers: clic o después de 5 segundos
const clicks$ = fromEvent(document, 'click').pipe(mapTo('click'));
const fiveSeconds$ = timer(5000, 5000).pipe(mapTo('timer'));
const trigger$ = merge(clicks$, fiveSeconds$);

source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log(`Salida del buffer (${values.length} elementos):`, values);
});
```

## ⚠️ Precauciones

### Cuidado con las fugas de memoria

`buffer` continúa acumulando valores hasta el siguiente trigger, por lo que si el trigger no se dispara durante mucho tiempo, puede consumir memoria excesiva.

```ts
// Mal ejemplo: el trigger puede no dispararse nunca
const neverTrigger$ = fromEvent(document.querySelector('.non-existent'), 'click');

source$.pipe(
  buffer(neverTrigger$) // El trigger no se dispara y el buffer se acumula infinitamente
).subscribe();
```

**Soluciones**:
- Combinar con `bufferTime` o `bufferCount` para limitar el tamaño máximo del buffer
- Agregar procesamiento de timeout

```ts
import { interval, fromEvent, timer, race } from 'rxjs';
import { buffer } from 'rxjs';

const source$ = interval(100);

// Múltiples triggers: clic o timeout de 5 segundos
const clicks$ = fromEvent(document, 'click');
const timeout$ = timer(10000); // Timeout máximo de 10 segundos

source$.pipe(
  buffer(race(clicks$, timeout$)) // Emite con el que ocurra primero
).subscribe(values => {
  console.log('Buffer:', values);
});
```

## 📚 Operadores relacionados

- [`bufferTime`](./bufferTime) - Buffering basado en tiempo
- [`bufferCount`](./bufferCount) - Buffering basado en cantidad
- [`bufferToggle`](https://rxjs.dev/api/operators/bufferToggle) - Control de buffering con Observables de inicio/fin
- [`bufferWhen`](https://rxjs.dev/api/operators/bufferWhen) - Buffering con condiciones de cierre dinámicas
- [`window`](./windowTime) - Devuelve Observables en lugar de buffers

## Resumen

El operador `buffer` es una herramienta poderosa para procesar valores en lotes usando un Observable externo como trigger. Permite procesamiento por lotes **basado en eventos** en lugar de tiempo o cantidad. Sin embargo, es necesario tener cuidado con las fugas de memoria cuando el trigger no se dispara.
